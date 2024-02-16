#include <stdio.h>
#include "include/GPS_UBX.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "string.h"

#define TAG             "GPS_UBX"
#define UBX_HEADER_1     0xB5
#define UBX_HEADER_2     0x62

#define MESSAGE_ID_NAV_POSLLH   0x02
#define MESSAGE_ID_NAV_SOL      0x06
#define MESSAGE_ID_NAV_VELNED   0x12

uint8_t rxPin = 16;
uint8_t txPin = 17;
uint8_t uartPort = UART_NUM_2;

uint16_t makeUint16(uint8_t low,uint8_t high){
    return (uint16_t)(high * 0xff) + low;
}

uint8_t validateChecksum(uint8_t *ubxPacket,uint16_t lengthUbxPacket){
    uint8_t CK_A=0,CK_B=0,i;

    for(i=0;i<lengthUbxPacket-3;i++){
        CK_A += ubxPacket[i+2];
        CK_B += CK_A;
    }
    if(CK_A == ubxPacket[lengthUbxPacket-1] && CK_B == ubxPacket[lengthUbxPacket]){
        return true;
    }
    else{
        return false;
    }
}

uint8_t *getUbxPacket(uint8_t *data){
    uint16_t lengthPacket = makeUint16(data[4],data[5]) + 8;
    uint8_t *ubxPacket = (uint8_t *)malloc(lengthPacket);
    memcpy(ubxPacket,data,lengthPacket);

    if(validateChecksum(ubxPacket,lengthPacket-1)){
        return ubxPacket;
    }
    else{
        return NULL;
    }
}

uint8_t *getUbxPayload(uint8_t *ubxPacket){
    uint16_t lengthPayload = makeUint16(ubxPacket[4],ubxPacket[5])-1;
    uint8_t *ubxPayload = (uint8_t *)malloc(lengthPayload+1);
    memcpy(ubxPayload,ubxPacket+6,lengthPayload+1);
    return ubxPayload;
}

void receiveTask(void *pvParameters){

    uint8_t data[1000],length = 0;

    while(1){      
        uart_get_buffered_data_len(uartPort, (size_t*)&length);
        
        if(length > 0){    
            length = uart_read_bytes(uartPort,data,length,0);

            // esp_log_buffer_hex(TAG, &data, length);

            if(data[0] == UBX_HEADER_1 && data[1] == UBX_HEADER_2){
                uint8_t *ubxPacket = getUbxPacket(data);
                if (ubxPacket != NULL) {

                    uint8_t messageClass = ubxPacket[2];
                    uint8_t messageId = ubxPacket[3];
                    uint8_t *ubxPayload = getUbxPayload(ubxPacket);
                
                    if(messageClass == 0x01){
                        switch(messageId){
                            case MESSAGE_ID_NAV_POSLLH:
                                // nav_posllh
                                nav_posllh_data_t navPosllh;
                                memcpy(&navPosllh,ubxPayload,sizeof(nav_posllh_data_t));
                                printf("NAV_POSLLH FOUND, latitude: %f, longitude: %f,horizontal Accuracy: %.03fm\n",navPosllh.latitude/10000000.00,navPosllh.longitude/10000000.00,navPosllh.horizontalAccurary/1000.00);
                            break;

                            case MESSAGE_ID_NAV_SOL:
                                //nav-sol
                                nav_sol_data_t navSol;
                                memcpy(&navSol,ubxPayload,sizeof(nav_sol_data_t));
                                // printf("NAV_SOL FOUND, ecefPosX: %ld, ecefPosY: %ld, ecefPosZ: %ld,posAccuracy: %ld\n",navSol.ecefPosX,navSol.ecefPosY,navSol.ecefPosZ,navSol.posAccuracy);
                                printf("NAV_SOL FOUND,fix: %d,ecefPosX: %ld,posDop: %d\n",navSol.gpsFix,navSol.ecefPosX,navSol.posDOP);
                                // printf("Time: weeknumber: %d,tieOfWeek: %ld,timeFractional: %ld\n",navSol.weekNumber,navSol.timeOfWeek,navSol.timeFractional);
                            break;
                            case MESSAGE_ID_NAV_VELNED:
                                /// nav-velned
                                nav_velned_data_t navVelned;
                                memcpy(&navVelned,ubxPayload,sizeof(nav_velned_data_t));
                                printf("NAV-VELNED FOUND,speed2D: %ld,speed3D: %ld\n",navVelned.speed2D,navVelned.speed3D);
                            break;
                            case 0x30:
                                /// nav-
                                printf("0x30 FOUND\n");
                            break;
                            case 0x03:
                                /// nav-
                                printf("0x03 FOUND\n");
                            break;
                            case 0x22:
                                /// nav-
                                printf("0x22 FOUND\n");
                            break;
                        }
                    }

                    free(ubxPacket);
                    free(ubxPayload);
                }
               
                // esp_log_buffer_hex(TAG, &data, length);

                // printf("%d,%d,%d,%d\n",camera1,camera2,camera3,camera4);
            }
            else{
                // ESP_LOGE(TAG,"RESYNC");
                uart_flush(uartPort);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void gpsUbxInit(uint8_t _uartPort,uint32_t _baudrate,uint8_t _rxPin,uint8_t _txPin){

    uartPort = _uartPort;
    rxPin = _rxPin;
    txPin = _txPin;

    /*configuro uart*/
    uart_config_t uart_config={
        .baud_rate = _baudrate,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .rx_flow_ctrl_thresh = 112,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(uartPort,&uart_config);

    uart_set_pin(uartPort,txPin,rxPin,-1,-1);

    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    uart_driver_install(uartPort, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);

    xTaskCreate(&receiveTask,"read gps task", 15000, NULL, 3 , NULL);
}
