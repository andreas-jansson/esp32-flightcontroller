#include "ringbuffer.h"

#include <iostream>
#include <cmath>
#include "cstring"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"


static std::map<CircularHandle_t , CircularBuffer*> s_buffer;


CircularHandle_t CircularBufCreate(uint32_t nItems, uint32_t itemsSize, std::string name=""){

    auto new_buf = new CircularBuffer(nItems, itemsSize, name);

    new_buf->m_data = static_cast<std::byte *>(calloc(new_buf->m_maxSize, new_buf->m_itemSize));

    s_buffer.emplace(new_buf,new_buf);


    return reinterpret_cast<CircularHandle_t>(new_buf);
}

esp_err_t CircularBufEnqueue(CircularHandle_t bufHandle, void* data){

    return s_buffer[bufHandle]->insert_obj(data);

}

esp_err_t CircularBufDequeue(CircularHandle_t bufHandle, void* data, TickType_t ticksToWait){
    
    return s_buffer[bufHandle]->get_obj(data, ticksToWait);
}

esp_err_t CircularBuffer::get_obj(void* data, TickType_t ticksToWait) {
    // 1) wait for at least one element to be available
    if (xSemaphoreTake(m_dataAvailable, ticksToWait) == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    if(m_currItems == 0){
        return ESP_ERR_NOT_FOUND;
    }

    // 2) lock the buffer while we remove one element
    std::lock_guard<decltype(m_bufferMutex)> lock(m_bufferMutex);

    std::memcpy(data, m_data + (m_startIdx * m_itemSize), m_itemSize);

    // advance the tail (start) index and decrement the size
    m_startIdx = (m_startIdx + 1) % m_maxSize;
    --m_currItems;

    // 3) if there’s still data left, release the semaphore
    //    so another pending reader wakes up immediately
    if (m_currItems > 0) {
        xSemaphoreGive(m_dataAvailable);
    }

    return ESP_OK;
}


esp_err_t CircularBuffer::get_obj2(void* data, TickType_t ticksToWait) {

    BaseType_t semStatus = xSemaphoreTake(m_dataAvailable, ticksToWait);

    if(semStatus == pdFALSE){
        return ESP_ERR_TIMEOUT;
    }

    m_bufferMutex.lock();
    printf("reading at idx: %lu\n", m_startIdx);

    if(!m_isEmpty){
        memcpy(data, this->m_data + (m_startIdx * m_itemSize), m_itemSize);
        
        m_startIdx =  ( m_startIdx + 1 ) %m_maxSize;
    }

    if(m_startIdx == m_endIdx && !m_isFull){
        m_isEmpty = true;
    }

    if(!m_isEmpty){
        xSemaphoreGive(m_dataAvailable);
    }


    uint64_t currMsgNr = m_totalMsgCntr - std::abs(static_cast<int64_t>(m_startIdx) - static_cast<int64_t>(m_endIdx));
    //printf("reading number: %llu m_totalMsgCntr: %llu\n", currMsgNr, m_totalMsgCntr);


    m_bufferMutex.unlock();

    return ESP_OK;

}


void CircularBuffer::peek_obj(void* data, TickType_t ticksToWait) {

   if(!m_newPeek)
   {
       return;
   }

    m_bufferMutex.lock();

    if(!m_isEmpty){
        memcpy(data, this->m_data + (m_startIdx * m_itemSize), sizeof(m_itemSize));
    }
    
    m_newPeek = false;

    m_bufferMutex.unlock();

}

esp_err_t CircularBuffer::insert_obj(void* data)
{
    m_bufferMutex.lock();

    // If full, make room by advancing start
    if (m_currItems == m_maxSize) {
        ++m_overwrittenCntr;
        m_startIdx = (m_startIdx + 1) % m_maxSize;
    }
    else {
        ++m_currItems;
    }

    memcpy(this->m_data + (m_endIdx * m_itemSize), data, m_itemSize);

    // Advance end pointer
    m_endIdx = (m_endIdx + 1) % m_maxSize;

    ++m_totalMsgCntr;

    xSemaphoreGive(m_dataAvailable);
    m_bufferMutex.unlock();

    return ESP_OK;
}

esp_err_t CircularBuffer::insert_obj2(void* data){

    m_bufferMutex.lock();

    if(m_startIdx == m_endIdx  && m_isEmpty){
        //printf("START m_start: %u m_end: %u offset: %u\n", m_startIdx, m_endIdx, (m_endIdx * m_itemSize));
        memcpy(this->m_data + (m_endIdx * m_itemSize), data, m_itemSize);
        m_isEmpty = false;
        //printf("number: %llu m_totalMsgCntr: %llu \n", m_totalMsgCntr +1, m_totalMsgCntr);

    }
    else if((m_startIdx == m_endIdx  && !m_isEmpty) || m_isFull){
        //printf("FULL m_start: %lu m_end: %lu offset: %lu\n", m_startIdx, m_endIdx, (m_endIdx * m_itemSize));

        memcpy(this->m_data + (m_endIdx * m_itemSize), data, m_itemSize);
        m_overwrittenCntr++;
        m_startIdx = (m_startIdx + 1) % m_maxSize;

        uint64_t overWritten = m_totalMsgCntr - std::abs(static_cast<int64_t>(m_startIdx) - static_cast<int64_t>(m_endIdx));
        //printf("number: %llu overwritten, new nr: %llu\n", overWritten, m_totalMsgCntr + 1);
        printf("inserting at idx: %lu\n", m_endIdx);

    }
    else{

        //printf("NORMAL m_start: %u m_end: %u offset: %u\n", m_startIdx, m_endIdx, (m_endIdx * m_itemSize));
        memcpy(this->m_data + (m_endIdx * m_itemSize), data, m_itemSize);
        //printf("number: %llu\n", m_totalMsgCntr + 1);
        printf("inserting at idx: %lu\n", m_endIdx);

    }

    m_newPeek = true;
    ++m_endIdx;
    m_endIdx %= m_maxSize;

     if(m_startIdx == m_endIdx && !m_isEmpty){
         m_isFull = true;
     }

     m_totalMsgCntr++;
     
     BaseType_t semStatus = xSemaphoreGive(m_dataAvailable);

     m_bufferMutex.unlock();

     return ESP_OK;

}


bool CircularBuffer::isSlotEmpty(uint32_t idx){

    if(m_isEmpty)
        return true;
    if(m_isFull)
        return false;

       if (m_startIdx >= m_endIdx) {
           return idx < m_endIdx || idx >= m_startIdx;
       } else {
           return idx < m_endIdx && idx >= m_startIdx;
       }
}


void CircularBuffer::clear_buffer(){


    m_startIdx = 0;
    m_endIdx = 0;
    m_newPeek = false;
    m_overwrittenCntr = 0;

    memset(m_data, 0, sizeof(m_maxSize*m_itemSize));


}
