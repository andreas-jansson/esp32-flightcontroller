

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
    struct sockaddr_in server_addr;
    

    if(this->socketFd == -1 || !this->connected){
        // Create socket
        this->socketFd = socket(AF_INET, SOCK_STREAM, 0);
        if (this->socketFd < 0) {
            //ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            return ESP_FAIL;
        }
        //ESP_LOGI(TAG, "Socket created");
        
        int yes = 1;
        setsockopt(this->socketFd, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int)); 

        // Configure server address
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr);

        // Connect to server
        int err = connect(this->socketFd, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (err != 0) {
            //ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(this->socketFd);
            return ESP_FAIL;
        }
        //ESP_LOGI(TAG, "Successfully connected to server");
    }

    int sent = send(this->socketFd, message.c_str(), message.length(), 0);

    if (sent < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        connected = false;
    } 
    

    return ESP_OK;
}

esp_err_t Client::sendDataToServer(const std::string& server_ip, int port, const std::vector<std::string> messages) {
    struct sockaddr_in server_addr;
    if(this->socketFd == -1){

        this->socketFd = socket(AF_INET, SOCK_STREAM, 0);
        if (this->socketFd < 0) {
            return ESP_FAIL;
        }
        
        int yes = 1;
        setsockopt(this->socketFd, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int)); 

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr);

        // Connect to server
        int err = connect(this->socketFd, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (err != 0) {
            close(this->socketFd);
            return ESP_FAIL;
        }
    }

    std::string sendMsg{};
    for(const auto& m : messages){
        sendMsg += m;
    }

    // Send message
    int sent = send(this->socketFd, sendMsg.c_str(), sendMsg.length(), 0);
    if (sent < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        this->socketFd = -1;
    } 
    else {
        //ESP_LOGI(TAG, "Message sent: %s", message.c_str());
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


void Client::web_task(void* args){

    size_t item_size{12};

    while(true){

        std::vector<std::string> messages;
        
        for (int i = 0; i < 5; i++) {
            // Receive data from the ring buffer
            YawPitchRoll* received_data = (YawPitchRoll*)xRingbufferReceive(this->dmp_buf_handle, &item_size, pdMS_TO_TICKS(5));

            if (received_data != nullptr && item_size == sizeof(YawPitchRoll)) {

                if(received_data->yaw >= 666){
                    vRingbufferReturnItem(this->dmp_buf_handle, (void*)received_data);
                    continue;
                }

                messages.insert(messages.begin(), received_data->to_str());
                vRingbufferReturnItem(this->dmp_buf_handle, (void*)received_data);
            }
            else {
                break;
            }
            vTaskDelay(10/portTICK_PERIOD_MS);
        }   

        if(messages.size() == 1)
            this->sendDataToServer(this->serverIp, this->serverPort, messages[0]);
        else if(messages.size() > 1){
            this->sendDataToServer(this->serverIp, this->serverPort, messages);
        }
    }
}

void Client::web_task2(void* args){

    size_t item_size = sizeof(TelemetryData);

    while(true){

        std::vector<std::string> messages;
        
        for (int i = 0; i < 10; i++) {
            TelemetryData* received_data = (TelemetryData*)xRingbufferReceive(this->web_buf_handle, &item_size, pdMS_TO_TICKS(50));

            if (received_data != nullptr && item_size == sizeof(TelemetryData)) {
                messages.insert(messages.begin(), received_data->ypr.to_str());
                vRingbufferReturnItem(this->web_buf_handle, (void*)received_data);
            }
            else {
                break;
            }
        }   

        if(messages.size() == 1)
            this->sendDataToServer(this->serverIp, this->serverPort, messages[0]);
        else if(messages.size() > 1){
            this->sendDataToServer(this->serverIp, this->serverPort, messages);
        }
    }
}