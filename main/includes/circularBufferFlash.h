#pragma once


#include <cstdint>
#include <vector>
#include <map>
#include <thread>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_partition.h"

#include "ringbuffer.h"


typedef uint32_t TickType_t;


class CircularBufferFlash: public CircularBuffer{

	SemaphoreHandle_t m_dataAvailable{};
    SemaphoreHandle_t m_bufferMutex{};

    std::string m_name;
    bool m_isFlash{};
    void* m_addr{};
    void* m_currAddr{};
    size_t m_maxFlashSize;
    const esp_partition_t* m_bb{};

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

    bool isCurrentFlashSectorErased{};

    bool isSlotEmpty(uint32_t idx);


    CircularBufferFlash(uint32_t nItems, uint32_t itemsSize, std::string name, std::string partionName){
        m_dataAvailable = xSemaphoreCreateCounting(nItems, 0);
        m_bufferMutex = xSemaphoreCreateMutex();
        m_maxSize = nItems;
        m_itemSize = itemsSize;
        m_name = name;
        m_currAddr = m_addr;

        const esp_partition_t* m_bb = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, partionName.c_str());
        assert(m_bb);

    }



    esp_err_t insert_obj(void* data);
    esp_err_t get_obj(void* data, TickType_t ticksToWait);
    void peek_obj(void* data, TickType_t ticksToWait);
    void print_all();
    void clear_buffer();

    uint64_t getNumOverwritten(){return m_overwrittenCntr;} 

};


typedef CircularBuffer* CircularHandle_t;

CircularHandle_t CircularBufCreateFlash(uint32_t nItems, uint32_t itemsSize, std::string name, std::string partionName);
esp_err_t CircularBufEnqueueFlash(CircularHandle_t bufHandle, void* data);
esp_err_t CircularBufDequeueFlash(CircularHandle_t bufHandle, void* data, TickType_t ticksToWait);
void print_all_flash(CircularHandle_t bufHandle);


