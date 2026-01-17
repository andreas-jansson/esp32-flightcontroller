#include "circularBufferFlash.h"

#include <iostream>
#include <cmath>
#include "cstring"

static std::map<CircularHandle_t , CircularBufferFlash*> s_buffer;


CircularHandle_t CircularBufCreateFlash(uint32_t nItems, uint32_t itemsSize, std::string name, std::string partionName){

    if(itemsSize % 0x1000  || itemsSize == 0)
        return nullptr;

    const esp_partition_t* bbCtx = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, partionName.c_str());
    assert(bbCtx);

    printf("blackbox: addr=0x%08lx size=%lu\n", (unsigned long)bbCtx->address, (unsigned long)bbCtx->size);

    if ((uint64_t)nItems * itemsSize > bbCtx->size) return nullptr;

    auto new_buf = new CircularBufferFlash(nItems, itemsSize, name, partionName);

    s_buffer.emplace(new_buf, new_buf);

    return reinterpret_cast<CircularHandle_t>(new_buf);
}

esp_err_t CircularBufEnqueueFlash(CircularHandle_t bufHandle, void* data){

    if(!s_buffer.count(bufHandle)) return ESP_ERR_INVALID_ARG;

    return s_buffer[bufHandle]->insert_obj(data);
}

esp_err_t CircularBufDequeueFlash(CircularHandle_t bufHandle, void* data, TickType_t ticksToWait){
    
    if(!s_buffer.count(bufHandle)) return ESP_ERR_INVALID_ARG;

    return s_buffer[bufHandle]->get_obj(data, ticksToWait);
}


esp_err_t CircularBufferFlash::insert_obj(void* data)
{
    bool increased{};

    xSemaphoreTake(m_bufferMutex, portMAX_DELAY);

    // If full, make room by advancing start
    if (m_currItems == m_maxSize) {
        ++m_overwrittenCntr;
        m_startIdx = (m_startIdx + 1) % m_maxSize;
    }
    else {
        ++m_currItems;
        increased = true;
    }

    uint32_t offset = m_itemSize * m_endIdx;
    ESP_ERROR_CHECK(esp_partition_erase_range(this->m_bb, offset, m_itemSize));

    ESP_ERROR_CHECK(esp_partition_write(this->m_bb, offset, data, m_itemSize));

    // Advance end pointer
    m_endIdx = (m_endIdx + 1) % m_maxSize;

    ++m_totalMsgCntr;

    //m_bufferMutex.unlock();
    xSemaphoreGive(m_bufferMutex);

    if (increased) 
        xSemaphoreGive(m_dataAvailable);

    return ESP_OK;
}

esp_err_t CircularBufferFlash::get_obj(void* data, TickType_t ticksToWait) {

    if (!data) return ESP_ERR_INVALID_ARG;

    // 1) wait for at least one element to be available
    if (xSemaphoreTake(m_dataAvailable, ticksToWait) == pdFALSE) {
        //if(m_name=="radio")
        //{
        //    printf("m_maxSize[%lu] m_itemSize[%lu] m_currItems[%lu] m_startIdx[%lu] m_endIdx[%lu] m_totalMsgCntr[%llu] m_overwrittenCntr[%llu]\n",
        //    m_maxSize, m_itemSize, m_currItems, m_startIdx, m_endIdx, m_totalMsgCntr, m_overwrittenCntr);
        //}
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreTake(m_bufferMutex, portMAX_DELAY);


    uint32_t offset = m_itemSize * m_startIdx;

    ESP_ERROR_CHECK(esp_partition_read(m_bb, offset, data, m_itemSize));
    
    m_startIdx = (m_startIdx + 1) % m_maxSize;
    --m_currItems;

    xSemaphoreGive(m_bufferMutex);

    return ESP_OK;
}

