#include "ringbuffer.h"

#include <iostream>
#include <cmath>
#include "cstring"



static std::map<CircularHandle_t , CircularBuffer*> s_buffer;


CircularHandle_t CircularBufCreate(uint32_t nItems, uint32_t itemsSize, std::string name){

    auto new_buf = new CircularBuffer(nItems, itemsSize, name);

    new_buf->m_data = static_cast<std::byte *>(calloc(new_buf->m_maxSize, new_buf->m_itemSize));
    if (!new_buf->m_data) {
         delete new_buf; 
         return nullptr;
    }

    s_buffer.emplace(new_buf, new_buf);

    return reinterpret_cast<CircularHandle_t>(new_buf);
}

esp_err_t CircularBufEnqueue(CircularHandle_t bufHandle, void* data){

    //if(!s_buffer.count(bufHandle)) return ESP_ERR_INVALID_ARG;

    return s_buffer[bufHandle]->insert_obj(data);
}



esp_err_t CircularBufDequeue(CircularHandle_t bufHandle, void* data, TickType_t ticksToWait){
    
    //if(!s_buffer.count(bufHandle)) return ESP_ERR_INVALID_ARG;

    return s_buffer[bufHandle]->get_obj(data, ticksToWait);
}


esp_err_t CircularBuffer::get_obj(void* data, TickType_t ticksToWait) {

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

    std::memcpy(data, m_data + (m_startIdx * m_itemSize), m_itemSize);

    m_startIdx = (m_startIdx + 1) % m_maxSize;
    --m_currItems;

    xSemaphoreGive(m_bufferMutex);

    return ESP_OK;
}


void CircularBuffer::peek_obj(void* data, TickType_t ticksToWait) {

   if(!m_newPeek)
   {
       return;
   }

    //m_bufferMutex.lock();
    xSemaphoreTake(m_bufferMutex, portMAX_DELAY);

    if(!m_currItems){
        memcpy(data, this->m_data + (m_startIdx * m_itemSize), m_itemSize);
    }
    
    m_newPeek = false;

    //m_bufferMutex.unlock();
    xSemaphoreGive(m_bufferMutex);
}

esp_err_t CircularBuffer::insert_obj(void* data)
{
    bool increased{};
    //m_bufferMutex.lock();
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

    memcpy(this->m_data + (m_endIdx * m_itemSize), data, m_itemSize);

    // Advance end pointer
    m_endIdx = (m_endIdx + 1) % m_maxSize;

    ++m_totalMsgCntr;

    //m_bufferMutex.unlock();
    xSemaphoreGive(m_bufferMutex);

    if (increased) 
        xSemaphoreGive(m_dataAvailable);

    return ESP_OK;
}


bool CircularBuffer::isSlotEmpty(uint32_t idx){
    if(m_currItems == m_maxSize)
        return true;
    if(!m_currItems)
        return false;

       if (m_startIdx >= m_endIdx) {
           return idx < m_endIdx || idx >= m_startIdx;
       } else {
           return idx < m_endIdx && idx >= m_startIdx;
       }
}


void CircularBuffer::clear_buffer(){

    xSemaphoreTake(m_bufferMutex, portMAX_DELAY);

    while (xSemaphoreTake(m_dataAvailable, 0) == pdTRUE) { /* drain */ }

    m_startIdx          = 0;
    m_endIdx            = 0;
    m_newPeek           = false;
    m_overwrittenCntr   = 0;
    m_currItems         = 0;

    memset(m_data, 0, m_maxSize*m_itemSize);

    xSemaphoreGive(m_bufferMutex);
}
