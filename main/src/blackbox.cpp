
#include <string.h>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <climits>

#include "blackbox.h"
#include "circularBufferFlash.h"

using namespace blackbox;

Blackbox::Blackbox(CircularHandle_t blackbox_telemetry_handle)
{

    m_nrItems = flashBlockSize / sizeof(logItem); //rounds down since integer division
    printf("%lu / %u = %lu\n", flashBlockSize, sizeof(logItem), m_nrItems);

    if(!m_nrItems){
        printf("error failed to create blackbox. m_nrItems[%lu]\n", m_nrItems);
        return;
    }

    m_telemetryHandle = blackbox_telemetry_handle;

    //this->m_flashHandle = CircularBufCreateFlash(m_nrItems, flashBlockSize, "blackbox", "blackbox");

}

void* Blackbox::log_task(void *args)
{
    esp_err_t status{};
    uint64_t seqNr{};
    uint32_t magic = std::rand() % UINT32_MAX;
    

    printf("blackbox created!  [%p]\n", this->m_flashHandle);
    printf("blackbox magic nr  [%lu]\n", magic);
    printf("blackbox m_nrItems [%lu]\n", m_nrItems);

    m_logBuffer = static_cast<logItem*>(calloc(m_nrItems, sizeof(blackbox::logItem)));

    while (true)
    {
        for(int i=0;i<m_nrItems;i++){
            logItem log{};
            esp_err_t status = CircularBufDequeue(m_telemetryHandle, &log.data, portMAX_DELAY);
            if(status == ESP_OK){
                log.seqNr = seqNr;
                log.magicNr = magic;
                m_logBuffer[i] = log;
                seqNr++;
                printf("seq %llu ch2[%u]\n", seqNr, static_cast<uint16_t>(log.data.channel.ch2));
            }
        }

        //status = CircularBufEnqueueFlash(m_flashHandle, m_logBuffer);
        //ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    }

    return nullptr;
}