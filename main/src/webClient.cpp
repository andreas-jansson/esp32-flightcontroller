

#include "driver/i2c.h"
#include "nvs_flash.h"

#include "webClient.h"
#include "common_data.h"



#define log_tag "mpu6050"

#define CONFIG_ESP_WIFI_SSID "Ubiquity 2"
#define CONFIG_ESP_WIFI_PASSWORD "#SuperDuper66!"

static const char *TAG = "scan";



#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "esp_log.h"
#include "esp_system.h"

Client* Client::client = nullptr;


esp_err_t Client::sendDataToServer(const std::string& server_ip, int port, const std::string& message) {
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
        //ESP_LOGE(TAG, "Error occurred during sending: errno %d msg: %s len %u", errno, message.c_str(), message.length());
        close(this->socketFd);
        this->socketFd = -1;
    } 
    

    return ESP_OK;
}

Client::Client(std::string wifiName, std::string wifiPassword,  std::string serverIp, uint16_t serverPort){

    this->wifiName = wifiName;
    this->wifiPassword = wifiPassword;
    this->serverIp = serverIp;
    this->serverPort = serverPort;
    this->socketFd = -1;
}


esp_err_t Client::init(RingbufHandle_t dmp_buf_handle, RingbufHandle_t web_buf_handle){

    esp_err_t status = 0;

    this->dmp_buf_handle = dmp_buf_handle;
    this->web_buf_handle = web_buf_handle;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = { .rssi = -127,
                           .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                            }
        },
    };
        
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

void Client::event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

Client* Client::GetInstance(std::string wifiName, std::string wifiPassword, std::string serverIp, uint16_t serverPort){
  if(client==nullptr){
        client = new Client(wifiName, wifiPassword, serverIp, serverPort);
    }
    return client;
}

Client* Client::GetInstance(){
    return client;
}

void Client::web_task2(void* args){

    size_t item_size = sizeof(TelemetryData);

    while(true){

        std::vector<std::string> messages;
        
        for (int i = 0; i < 5; i++) {
            TelemetryData* received_data = (TelemetryData*)xRingbufferReceive(this->web_buf_handle, &item_size, portMAX_DELAY);
            if (received_data != nullptr) {
                messages.insert(messages.begin(), received_data->ypr.to_str());
                vRingbufferReturnItem(this->web_buf_handle, (void*)received_data);
            }
            else {
                break;
            }
        }   

        std::string jsonMsg;

        jsonMsg = "{ \"ypr\": [";
        for (size_t i = 0; i < messages.size(); i++) {
            jsonMsg += "\"" + messages[i] + "\"";
            if (i < messages.size() - 1) { 
                jsonMsg += ", ";
            }
        }
        jsonMsg += "]}";

        this->sendDataToServer(this->serverIp, this->serverPort, jsonMsg);

    }
}