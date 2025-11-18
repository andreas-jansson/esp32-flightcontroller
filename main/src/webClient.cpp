

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


std::string create_json(std::vector<std::string>& items){

    std::string list{"{"};

    for (size_t i = 0; i < items.size(); i++) {

        list += items[i];

        if (i < items.size() - 1) { 
            list += ", ";
        }
    }
    list += "}";

    return list;

}






esp_err_t WebClient::sendDataToServer(const std::string& server_ip, int port, const std::string& message) {
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

    int sent = sendto(this->socketFd, message.c_str(), message.length(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));

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

    strncpy((char*)wifi_config.sta.ssid, this->wifiName.c_str(), sizeof(wifi_config.sta.ssid)+1);
    strncpy((char*)wifi_config.sta.password, this->wifiPassword.c_str(), sizeof(wifi_config.sta.password)+1);

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

    size_t item_size = sizeof(TelemetryData);

    while(true){

        std::vector<std::string> ypr_messages1;
        std::vector<std::string> ypr_messages2;
        std::vector<std::string> fl_messages;
        std::vector<std::string> fr_messages;
        std::vector<std::string> rl_messages;
        std::vector<std::string> rr_messages;
        std::vector<std::string> ch_messages;
        std::vector<std::string> tel_message0;
        std::vector<std::string> tel_message1;
        std::vector<std::string> tel_message2;
        std::vector<std::string> tel_message3;
        std::vector<std::string> curr_message;

        for (int i = 0; i < 1; i++) {
            TelemetryData* received_data = (TelemetryData*)xRingbufferReceive(this->web_buf_handle, &item_size, pdMS_TO_TICKS(2000));
            if (received_data != nullptr) {
                ypr_messages1.insert(ypr_messages1.begin(), received_data->ypr1.to_str());
                ypr_messages2.insert(ypr_messages2.begin(), received_data->ypr2.to_str());

                fl_messages.insert(fl_messages.begin(), received_data->drone.throttle_fl_to_str());
                fr_messages.insert(fr_messages.begin(), received_data->drone.throttle_fr_to_str());
                rl_messages.insert(rl_messages.begin(), received_data->drone.throttle_rl_to_str());
                rr_messages.insert(rr_messages.begin(), received_data->drone.throttle_rr_to_str());
                ch_messages.insert(ch_messages.begin(), received_data->channel.to_str());

                tel_message0.insert(tel_message0.begin(), received_data->drone.escState[0].to_str());
                tel_message1.insert(tel_message1.begin(), received_data->drone.escState[1].to_str());
                tel_message2.insert(tel_message2.begin(), received_data->drone.escState[2].to_str());
                tel_message3.insert(tel_message3.begin(), received_data->drone.escState[3].to_str());

                curr_message.insert(curr_message.begin(), received_data->drone.current_to_str());


                vRingbufferReturnItem(this->web_buf_handle, (void*)received_data);
            }
            else {
                break;
            }
        }   

        /* ypr json */
        std::string ypr_key1{"ypr1"};
        std::string ypr_item1 = construct_json_item(ypr_key1, ypr_messages1);

        std::string ypr_key2{"ypr2"};
        std::string ypr_item2 = construct_json_item(ypr_key2, ypr_messages2);

        /* throttle json */
        std::string fl_key{"fl"};
        std::string fl_item = construct_json_item(fl_key, fl_messages);

        std::string fr_key{"fr"};
        std::string fr_item = construct_json_item(fr_key, fr_messages);

        std::string rl_key{"rl"};
        std::string rl_item = construct_json_item(rl_key, rl_messages);

        std::string rr_key{"rr"};
        std::string rr_item = construct_json_item(rr_key, rr_messages);

        /*  channels json */
        std::string ch_key{"channel"};
        std::string ch_item = construct_json_item(ch_key, ch_messages);

        /* telemetry json */
        std::string tel0_key{"telemetry0"};
        std::string tel0_item = construct_json_item(tel0_key, tel_message0);

        std::string tel1_key{"telemetry1"};
        std::string tel1_item = construct_json_item(tel1_key, tel_message1);

        std::string tel2_key{"telemetry2"};
        std::string tel2_item = construct_json_item(tel2_key, tel_message2);

        std::string tel3_key{"telemetry3"};
        std::string tel3_item = construct_json_item(tel3_key, tel_message3);

        /* current json */
        std::string curr_key{"current"};
        std::string curr_item = construct_json_item(curr_key, curr_message);

        std::vector<std::string> jsonItems;

        jsonItems.emplace_back(ypr_item1);
        jsonItems.emplace_back(ypr_item2);
        jsonItems.emplace_back(fl_item);
        jsonItems.emplace_back(fr_item);
        jsonItems.emplace_back(rl_item);
        jsonItems.emplace_back(rr_item);
        jsonItems.emplace_back(ch_item);
        jsonItems.emplace_back(tel0_item);
        jsonItems.emplace_back(tel1_item);
        jsonItems.emplace_back(tel2_item);
        jsonItems.emplace_back(tel3_item);

        this->sendDataToServer(this->serverIp, this->serverPort, create_json(jsonItems));

    }
}