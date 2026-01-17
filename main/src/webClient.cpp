

#include "driver/i2c.h"
#include "nvs_flash.h"

#include "webClient.h"
#include "dshot600.h"
#include "common_data.h"
#include "display.h"



#define log_tag "mpu6050"

//#define CONFIG_ESP_WIFI_SSID "Ubiquity 2"
//#define CONFIG_ESP_WIFI_PASSWORD "#SuperDuper66!"


//#define CONFIG_ESP_WIFI_SSID "Wavy"
//#define CONFIG_ESP_WIFI_PASSWORD "##SnabbtSkit555!!"

static const char *TAG = "scan";



#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "esp_log.h"
#include "esp_system.h"

WebClient* WebClient::client = nullptr;



void format_quotes(std::string& str){
    str = "\"" + str + "\"";
}


std::string format_list(std::vector<std::string>& values){

    std::string list{"["};

    for (size_t i = 0; i < values.size(); i++) {

        format_quotes(values[i]);

        list += values[i];

        if (i < values.size() - 1) { 
            list += ", ";
        }
    }
    list += "]";

    return list;
}


std::string construct_json_item(std::string& key, std::vector<std::string>& values){
    format_quotes(key);
    return key + " : " + format_list(values);
}

std::string construct_json_item(std::string& key, std::string& value){
    format_quotes(key);
    format_quotes(value);
    return key + " : " + value;
}

esp_err_t construct_json_item(char* key, char* value, char* buffer, size_t bufSize){
    int bytes = snprintf(buffer, bufSize, "\"%s\" : [\"%s\"]", key, value);
    if(bytes==-1)
        return ESP_FAIL;
    else
        return ESP_OK;
}


std::string create_json(std::vector<std::string>& items){


    // Compute exact-ish size to avoid repeated reallocs
    size_t total = 2; // '{' + '}'
    if (!items.empty()) total += 2 * (items.size() - 1); // ", " separators
    for (const auto& s : items) total += s.size();

    std::string list;
    
    list.reserve(total);
    list.append("{");

    for (size_t i = 0; i < items.size(); i++) {

        list += items[i];

        if (i < items.size() - 1) { 
            list += ", ";
        }
    }
    list += "}";

    return list;

}


esp_err_t create_json(const std::vector<char*>& items, char* buffer, size_t bufSize)
{
    if (!buffer || bufSize < 3) { // at least "{}\0"
        return ESP_ERR_INVALID_ARG;
    }

    size_t pos = 0;

    buffer[pos++] = '{';
    buffer[pos] = '\0';

    for (size_t i = 0; i < items.size(); i++) {
        const char* s = items[i] ? items[i] : "";

        // Append item
        int n = snprintf(buffer + pos, bufSize - pos, "%s", s);
        if (n < 0 || (size_t)n >= bufSize - pos) return ESP_ERR_NO_MEM;
        pos += (size_t)n;

        // Append separator to match top: ", "
        if (i + 1 < items.size()) {
            n = snprintf(buffer + pos, bufSize - pos, ", ");
            if (n < 0 || (size_t)n >= bufSize - pos) return ESP_ERR_NO_MEM;
            pos += (size_t)n;
        }
    }

    // Append closing brace
    int n = snprintf(buffer + pos, bufSize - pos, "}");
    if (n < 0 || (size_t)n >= bufSize - pos) return ESP_ERR_NO_MEM;

    return ESP_OK;
}






esp_err_t WebClient::sendDataToServer(const std::string& server_ip, int port, char* message /*const std::string& message*/) {
    static struct sockaddr_in server_addr;

    if(this->socketFd == -1){
        // Create socket
        this->socketFd = socket(AF_INET, SOCK_DGRAM, 0);
        if (this->socketFd < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            return ESP_FAIL;
        }

        // Configure server address
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr);
    }

    int sent = sendto(this->socketFd, message, strlen(message), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

    if (sent <= 0) {
        close(this->socketFd);
        this->socketFd = -1;
    } 
    

    return ESP_OK;
}

WebClient::WebClient(std::string wifiName, std::string wifiPassword,  std::string serverIp, uint16_t serverPort){

    this->wifiName = wifiName;
    this->wifiPassword = wifiPassword;
    this->serverIp = serverIp;
    this->serverPort = serverPort;
    this->socketFd = -1;
}


esp_err_t WebClient::init(RingbufHandle_t dmp_buf_handle, RingbufHandle_t web_buf_handle){

    esp_err_t status = 0;

    this->dmp_buf_handle = dmp_buf_handle;
    this->web_buf_handle = web_buf_handle;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();

    wifi_config_t wifi_config = {
        .sta = {
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = { .rssi = -127,
                           .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                            }
        },
    };

    strncpy((char*)wifi_config.sta.ssid, this->wifiName.c_str(), sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, this->wifiPassword.c_str(), sizeof(wifi_config.sta.password));

    printf("<<<<<< %s %s >>>>>>>>\n", wifi_config.sta.ssid, wifi_config.sta.password);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    status = esp_wifi_init(&init_config);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to init wifi");

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    status = esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set wifi mode");

    status = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set wifi mode");

    status = esp_wifi_start();
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to start wifi");

    vTaskDelay(100/portTICK_PERIOD_MS);

    return status;
}

void WebClient::event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        Display::set_wifi_status(true);
    }
}

WebClient* WebClient::GetInstance(std::string wifiName, std::string wifiPassword, std::string serverIp, uint16_t serverPort){
  if(client==nullptr){
        client = new WebClient(wifiName, wifiPassword, serverIp, serverPort);
    }
    return client;
}

WebClient* WebClient::GetInstance(){
    return client;
}

void WebClient::web_task2(void* args){

    esp_err_t status{};
    
    size_t item_size{sizeof(TelemetryData)};
    constexpr size_t itemBufferSize{80};
    constexpr size_t itemSize{90};
    constexpr size_t finalJsonSize{itemSize * 20};

    char pid_pitch_message[itemBufferSize]{};
    char pid_roll_message[itemBufferSize]{};
    char pid_yaw_message[itemBufferSize]{};
    char ypr_message1[itemBufferSize]{};
    char ypr_message2[itemBufferSize]{};
    char fl_message[itemBufferSize]{};
    char fr_message[itemBufferSize]{};
    char rl_message[itemBufferSize]{};
    char rr_message[itemBufferSize]{};
    char ch_message[itemBufferSize]{};
    char tel_message0[itemBufferSize]{};
    char tel_message1[itemBufferSize]{};
    char tel_message2[itemBufferSize]{};
    char tel_message3[itemBufferSize]{};
    char curr_message[itemBufferSize]{};

    
    char pid_pitch_item[itemSize]{};
    char pid_pitch_key[] = "pidPitch";
    char pid_roll_item[itemSize]{};
    char pid_roll_key[] = "pidRoll";
    char pid_yaw_item[itemSize]{};
    char pid_yaw_key[] = "pidYaw"; 
    char ypr_item1[itemSize]{};
    char ypr_key1[] = "ypr1";
    char ypr_item2[itemSize]{};
    char ypr_key2[] = "ypr2";
    char fl_item[itemSize]{};
    char fl_key[] = "fl";
    char fr_item[itemSize]{};
    char fr_key[] = "fr";
    char rl_item[itemSize]{};
    char rl_key[] = "rl";
    char rr_item[itemSize]{};
    char rr_key[] = "rr";
    char ch_item[itemSize]{};
    char ch_key[] = "channel";
    char tel0_item[itemSize]{};
    char tel0_key[] = "telemetry0";
    char tel1_item[itemSize]{};
    char tel1_key[] = "telemetry1";
    char tel2_item[itemSize]{};
    char tel2_key[] = "telemetry2";
    char tel3_item[itemSize]{};
    char tel3_key[] = "telemetry3";
    char curr_item[itemSize]{};
    char curr_key[] = "current";

    char finalJson[finalJsonSize]{};
    
    while(true){

        for (int i = 0; i < 1; i++) {
            TelemetryData* received_data = (TelemetryData*)xRingbufferReceive(this->web_buf_handle, &item_size, pdMS_TO_TICKS(2000));
            if (received_data != nullptr) {

                // PID (already converted)
                status = received_data->drone.pid[PITCH].to_str(pid_pitch_message, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.pid[ROLL ].to_str(pid_roll_message,  itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.pid[YAW  ].to_str(pid_yaw_message,   itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);

                // YPR (convert from std::string-returning to buffer-filling)
                status = received_data->ypr1.to_str(ypr_message1, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->ypr2.to_str(ypr_message2, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);

                // Throttles (same deal)
                
                status = received_data->drone.throttle_fl_to_str(fl_message, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.throttle_fr_to_str(fr_message, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.throttle_rl_to_str(rl_message, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.throttle_rr_to_str(rr_message, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);

                // Channel
                status = received_data->channel.to_str(ch_message, itemBufferSize);

                // ESC telemetry
                status = received_data->drone.escState[0].to_str(tel_message0, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.escState[1].to_str(tel_message1, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.escState[2].to_str(tel_message2, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);
                status = received_data->drone.escState[3].to_str(tel_message3, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);

                // Current
                status = received_data->drone.current_to_str(curr_message, itemBufferSize);
                ESP_ERROR_CHECK_WITHOUT_ABORT(status);

                vRingbufferReturnItem(this->web_buf_handle, (void*)received_data);
            }
            else {
                break;
            }
        }   


        /* pid  pitch */
        status = construct_json_item(pid_pitch_key, pid_pitch_message, pid_pitch_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* pid  roll */
        status = construct_json_item(pid_roll_key, pid_roll_message, pid_roll_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* pid  yaw */
        status = construct_json_item(pid_yaw_key, pid_yaw_message, pid_yaw_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* ypr json */
        status = construct_json_item(ypr_key1, ypr_message1, ypr_item1, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(ypr_key2, ypr_message2, ypr_item2, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* throttle json */
        status = construct_json_item(fl_key, fl_message, fl_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(fr_key, fr_message, fr_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(rl_key, rl_message, rl_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(rr_key, rr_message, rr_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* channels json */
        status = construct_json_item(ch_key, ch_message, ch_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* telemetry json */
        status = construct_json_item(tel0_key, tel_message0, tel0_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(tel1_key, tel_message1, tel1_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(tel2_key, tel_message2, tel2_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = construct_json_item(tel3_key, tel_message3, tel3_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        /* current json */
        status = construct_json_item(curr_key, curr_message, curr_item, itemSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        //std::vector<std::string> jsonItems;
        std::vector<char*> jsonItems;
        jsonItems.emplace_back(&pid_pitch_item[0]);
        jsonItems.emplace_back(&pid_roll_item[0]);
        jsonItems.emplace_back(&pid_yaw_item[0]);
        jsonItems.emplace_back(&ypr_item1[0]);
        jsonItems.emplace_back(&ypr_item2[0]);
        jsonItems.emplace_back(&fl_item[0]);
        jsonItems.emplace_back(&fr_item[0]);
        jsonItems.emplace_back(&rl_item[0]);
        jsonItems.emplace_back(&rr_item[0]);
        jsonItems.emplace_back(&ch_item[0]);
        jsonItems.emplace_back(&tel0_item[0]);
        jsonItems.emplace_back(&tel1_item[0]);
        jsonItems.emplace_back(&tel2_item[0]);
        jsonItems.emplace_back(&tel3_item[0]);
        jsonItems.emplace_back(&curr_item[0]);

        status = create_json(jsonItems, finalJson, finalJsonSize);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = this->sendDataToServer(this->serverIp, this->serverPort, finalJson);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    }
}