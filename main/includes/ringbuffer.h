#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <thread>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


typedef uint32_t TickType_t;

struct Payload{
    uint16_t id{};
    char str[6]{};
};

class CircularBuffer{

	SemaphoreHandle_t m_dataAvailable{};
    SemaphoreHandle_t m_bufferMutex{};

    std::string m_name;

public:

    volatile bool m_newPeek{false};

    volatile uint32_t m_maxSize{};
    volatile uint32_t m_itemSize{};
    volatile uint32_t m_currItems{};


    volatile uint32_t m_startIdx{};
    volatile uint32_t m_endIdx{};

    volatile uint64_t m_overwrittenCntr{};
    volatile uint64_t m_totalMsgCntr{};

    std::byte* m_data{};


    bool isSlotEmpty(uint32_t idx);

    CircularBuffer(uint32_t nItems, uint32_t itemsSize, std::string name=""){
        m_dataAvailable = xSemaphoreCreateCounting(nItems, 0);
        m_bufferMutex = xSemaphoreCreateMutex();
        m_maxSize = nItems;
        m_itemSize = itemsSize;
        m_name = name;
    }

    


    esp_err_t insert_obj(void* data);
    esp_err_t get_obj(void* data, TickType_t ticksToWait);
    void peek_obj(void* data, TickType_t ticksToWait);
    void print_all();
    void clear_buffer();

    uint64_t getNumOverwritten(){return m_overwrittenCntr;} 

};


typedef CircularBuffer* CircularHandle_t;

CircularHandle_t CircularBufCreate(uint32_t nItems, uint32_t itemsSize, std::string name);

esp_err_t CircularBufEnqueue(CircularHandle_t bufHandle, void* data);

esp_err_t CircularBufDequeue(CircularHandle_t bufHandle, void* data, TickType_t ticksToWait);

void print_all(CircularHandle_t bufHandle);


