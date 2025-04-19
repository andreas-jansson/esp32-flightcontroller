#pragma once

#include <string>
#include <vector>

#include "freertos/ringbuf.h"
#include "esp_check.h"
#include "esp_wifi.h"


class WebClient{
    static class WebClient* client;
    int socketFd{};
    bool connected{};

    std::string wifiName{};
    std::string wifiPassword{};
    std::string serverIp{};
    uint16_t serverPort{};
    RingbufHandle_t dmp_buf_handle;
    RingbufHandle_t web_buf_handle;

    static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    WebClient(std::string wifiName, std::string wifiPassword,  std::string serverIp, uint16_t serverPort);

    public:
    WebClient(WebClient &other) = delete;
    void operator=(const WebClient &) = delete;
    static WebClient *GetInstance(std::string wifiName, std::string wifiPassword,  std::string serverIp, uint16_t serverPort);
	static WebClient *GetInstance();

    esp_err_t sendDataToServer(const std::string& server_ip, int port, const std::string& message);
    esp_err_t sendDataToServer(const std::string& server_ip, int port, const std::vector<std::string> messages);

    esp_err_t init(RingbufHandle_t dmp_buf_handle, RingbufHandle_t web_buf_handle);

    void web_task(void* args);
    void web_task2(void* args);

};

