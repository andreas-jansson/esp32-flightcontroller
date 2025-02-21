#include <stdio.h>
#include "ESP_CRSF.h"

#define RX_BUF_SIZE 1024    //UART buffer size



#include <machine/endian.h>

/* Swap bytes in 16 bit value.  */
#ifndef __bswap_16
#ifdef __GNUC__
# define __bswap_16(x) \
    (__extension__							      \
     ({ unsigned short int __bsx = (x);					      \
        ((((__bsx) >> 8) & 0xff) | (((__bsx) & 0xff) << 8)); }))
#else
static INLINE unsigned short int
__bswap_16 (unsigned short int __bsx)
{
  return ((((__bsx) >> 8) & 0xff) | (((__bsx) & 0xff) << 8));
}
#endif
#endif // __bswap_16

/* Swap bytes in 32 bit value.  */
#ifndef __bswap_32
#ifdef __GNUC__
# define __bswap_32(x) \
    (__extension__							      \
     ({ unsigned int __bsx = (x);					      \
        ((((__bsx) & 0xff000000) >> 24) | (((__bsx) & 0x00ff0000) >>  8) |    \
	 (((__bsx) & 0x0000ff00) <<  8) | (((__bsx) & 0x000000ff) << 24)); }))
#else
static INLINE unsigned int
__bswap_32 (unsigned int __bsx)
{
  return ((((__bsx) & 0xff000000) >> 24) | (((__bsx) & 0x00ff0000) >>  8) |
	  (((__bsx) & 0x0000ff00) <<  8) | (((__bsx) & 0x000000ff) << 24));
}
#endif
#endif // __bswap_32

#ifndef __bswap_64
#if defined __GNUC__ && __GNUC__ >= 2
/* Swap bytes in 64 bit value.  */
# define __bswap_constant_64(x) \
     ((((x) & 0xff00000000000000ull) >> 56)				      \
      | (((x) & 0x00ff000000000000ull) >> 40)				      \
      | (((x) & 0x0000ff0000000000ull) >> 24)				      \
      | (((x) & 0x000000ff00000000ull) >> 8)				      \
      | (((x) & 0x00000000ff000000ull) << 8)				      \
      | (((x) & 0x0000000000ff0000ull) << 24)				      \
      | (((x) & 0x000000000000ff00ull) << 40)				      \
      | (((x) & 0x00000000000000ffull) << 56))

# define __bswap_64(x) \
     (__extension__							      \
      ({ union { __extension__ unsigned long long int __ll;		      \
		 unsigned int __l[2]; } __w, __r;			      \
         if (__builtin_constant_p (x))					      \
	   __r.__ll = __bswap_constant_64 (x);				      \
	 else								      \
	   {								      \
	     __w.__ll = (x);						      \
	     __r.__l[0] = __bswap_32 (__w.__l[1]);			      \
	     __r.__l[1] = __bswap_32 (__w.__l[0]);			      \
	   }								      \
	 __r.__ll; }))
#endif
#endif // __bswap_64


// CRC8 lookup table (poly 0xd5)
static uint8_t crc8_table[256] = {0};

void generate_CRC(uint8_t poly)
{
    for (int idx=0; idx<256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift=0; shift<8; ++shift)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        crc8_table[idx] = crc & 0xff;
    }
}

// Function to calculate CRC8 checksum
uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--)
    {
        crc = crc8_table[crc ^ *data++];
    }

    return crc;
}


SemaphoreHandle_t xMutex;

static int uart_num = 1;
static QueueHandle_t uart_queue;
crsf_channels_t received_channels = {0};
crsf_battery_t received_battery = {0};


static void rx_task(void *arg)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            if (event.type == UART_DATA ) {
                //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);

                //printf("uart data:\n");
                //for(int i=0;i<event.size;i++){
                //    printf(" 0x%-2x\n", dtmp[i]);
                //}
                //printf("\n");

                //extract length and type
                uint8_t type = dtmp[2];
                uint8_t length = dtmp[1];
                uint8_t dest = dtmp[0];
                //printf("type: 0x%-2x length: 0x%-2x dest: 0x%-2x\n", type, length, dest);
                //read the rest of the frame
                uint8_t payload_length = length - 2;
                uint8_t payload[payload_length];

                //for (int i = 0; i < payload_length; i++) {
                //    payload[i] = dtmp[i+3];
                //}
                memcpy(&received_channels, (crsf_channels_t*)&dtmp[3], 22);

                printf("[0]sync: 0x%x  type: 0x%x frameLen: %u data:", dest, type, length);

                for(int j=3;j<9;j++)
                    printf(" 0x%x", dtmp[j]);
                printf("\n");


                //if (type == CRSF_TYPE_CHANNELS) {
                //    
                //    xSemaphoreTake(xMutex, portMAX_DELAY);
                //    received_channels = *(crsf_channels_t*)payload;
                //    xSemaphoreGive(xMutex);
//
                //}
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void CRSF_init(crsf_config_t *config)
{

    generate_CRC(0xd5);

    uart_num = config->uart_num;

    //begin uart communication with RX
    uart_config_t uart_config = {
        .baud_rate = 420000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        
    };
    uart_param_config(config->uart_num, &uart_config);
    uart_set_pin(uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, RX_BUF_SIZE, 10, &uart_queue, 0));

    //create semaphore
    xMutex = xSemaphoreCreateMutex();
    //create task
    xTaskCreate(rx_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES-1, NULL);
    
}

//receive uart data frame
void CRSF_receive_channels(crsf_channels_t *channels)
{
    xSemaphoreTake(xMutex, portMAX_DELAY);
    *channels = received_channels;
    xSemaphoreGive(xMutex);
}
/**
 * @brief function sends payload to a destination using uart
 * 
 * @param payload pointer to payload of given crsf_type_t
 * @param destination destination for payload, typically CRSF_DEST_FC
 * @param type type of data contained in payload
 * @param payload_length length of the payload type
 */
void CRSF_send_payload(const void* payload, crsf_dest_t destination, crsf_type_t type, uint8_t payload_length)
{
    uint8_t packet[payload_length+4]; //payload + dest + len + type + crc

    packet[0] = destination;
    packet[1] = payload_length+2; // size of payload + type + crc
    packet[2] = type;

    memcpy(&packet[3], payload, payload_length);

    //calculate crc
    unsigned char checksum = crc8(&packet[2], payload_length+1);
    
    packet[payload_length+3] = checksum;

    //send frame
    uart_write_bytes(uart_num, &packet, payload_length+4);
}

void CRSF_send_battery_data(crsf_dest_t dest, crsf_battery_t* payload)
{
    crsf_battery_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_battery_t*)payload;
    payload_proc->voltage = __bswap16(payload_proc->voltage);
    payload_proc->current = __bswap16(payload_proc->current);
    payload_proc->capacity = __bswap16(payload_proc->capacity) << 8;

    CRSF_send_payload(payload_proc, dest, CRSF_TYPE_BATTERY, sizeof(crsf_battery_t));
}

void CRSF_send_gps_data(crsf_dest_t dest, crsf_gps_t* payload)
{
    crsf_gps_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_gps_t*)payload;
    payload_proc->latitude = __bswap32(payload_proc->latitude);
    payload_proc->longitude = __bswap32(payload_proc->longitude);
    payload_proc->groundspeed = __bswap16(payload_proc->groundspeed);
    payload_proc->heading = __bswap16(payload_proc->heading);
    payload_proc->altitude = __bswap16(payload_proc->altitude);

    CRSF_send_payload(payload_proc, dest, CRSF_TYPE_GPS, sizeof(crsf_gps_t));
}
