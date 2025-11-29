#include <stdio.h>
#include <string.h>
#include <math.h> // Added for sine wave demo
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


#define MSP_RX_PIN      (GPIO_NUM_17)
#define MSP_TX_PIN      (GPIO_NUM_16)
#define MSP_UART_NUM    (UART_NUM_2)
#define MSP_BAUD_RATE   (115200)
#define BUF_SIZE        (1024)

#define LOOP_RATE_MS    50 

static const char *TAG = "MSP_CTRL";

#define MSP_CMD_RC          105 // Read RC
#define MSP_CMD_SET_RAW_RC  200 // Write RC

typedef enum {
    MSP_IDLE, MSP_HEADER_M_STATE, MSP_DIRECTION_STATE, 
    MSP_SIZE_STATE, MSP_CMD_STATE, MSP_PAYLOAD_STATE, MSP_CRC_STATE
} msp_state_t;

typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;
} msp_rc_channels_t;

msp_rc_channels_t rc_data;


void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = MSP_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(MSP_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(MSP_UART_NUM, &uart_config);
    uart_set_pin(MSP_UART_NUM, MSP_TX_PIN, MSP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void msp_send_packet(uint8_t cmd_id, void *payload, uint8_t size) {
    uint8_t header[6];
    uint8_t checksum = 0;

    header[0] = '$'; header[1] = 'M'; header[2] = '<';
    header[3] = size; header[4] = cmd_id;
    
    checksum = size ^ cmd_id;

    uart_write_bytes(MSP_UART_NUM, (const char *)header, 5);

    uint8_t *payload_ptr = (uint8_t *)payload;
    if (size > 0 && payload != NULL) {
        for (int i = 0; i < size; i++) {
            checksum ^= payload_ptr[i];
        }
        uart_write_bytes(MSP_UART_NUM, (const char *)payload, size);
    }
    uart_write_bytes(MSP_UART_NUM, (const char *)&checksum, 1);
}

void msp_request_rc() {
    msp_send_packet(MSP_CMD_RC, NULL, 0);
}

void msp_set_raw_rc(msp_rc_channels_t *channels) {
    uint8_t payload[16]; 
    payload[0] = channels->roll & 0xFF;  payload[1] = channels->roll >> 8;
    payload[2] = channels->pitch & 0xFF; payload[3] = channels->pitch >> 8;
    payload[4] = channels->yaw & 0xFF;   payload[5] = channels->yaw >> 8;
    payload[6] = channels->throttle & 0xFF; payload[7] = channels->throttle >> 8;
    payload[8] = channels->aux1 & 0xFF;  payload[9] = channels->aux1 >> 8;
    payload[10]= channels->aux2 & 0xFF;  payload[11]= channels->aux2 >> 8;
    payload[12]= channels->aux3 & 0xFF;  payload[13]= channels->aux3 >> 8;
    payload[14]= channels->aux4 & 0xFF;  payload[15]= channels->aux4 >> 8;

    msp_send_packet(MSP_CMD_SET_RAW_RC, payload, 16);
}

void process_msp_byte(uint8_t byte) {
    static msp_state_t state = MSP_IDLE;
    static uint8_t message_len = 0;
    static uint8_t message_cmd = 0;
    static uint8_t rx_buffer[64];
    static uint8_t rx_index = 0;
    static uint8_t calculated_crc = 0;

    switch (state) {
        case MSP_IDLE: if (byte == '$') state = MSP_HEADER_M_STATE; break;
        case MSP_HEADER_M_STATE: if (byte == 'M') state = MSP_DIRECTION_STATE; else state = MSP_IDLE; break;
        case MSP_DIRECTION_STATE: if (byte == '>') state = MSP_SIZE_STATE; else state = MSP_IDLE; break;
        case MSP_SIZE_STATE: 
            message_len = byte; 
            state = (message_len > 60) ? MSP_IDLE : MSP_CMD_STATE; 
            calculated_crc = byte; 
            break;
        case MSP_CMD_STATE:
            message_cmd = byte;
            calculated_crc ^= byte;
            rx_index = 0;
            state = (message_len > 0) ? MSP_PAYLOAD_STATE : MSP_CRC_STATE;
            break;
        case MSP_PAYLOAD_STATE:
            rx_buffer[rx_index++] = byte;
            calculated_crc ^= byte;
            if (rx_index == message_len) state = MSP_CRC_STATE;
            break;
        case MSP_CRC_STATE:
            if (calculated_crc == byte && message_cmd == MSP_CMD_RC) {
                ESP_LOGD(TAG, "RC Updated from FC");
            }
            state = MSP_IDLE;
            break;
        default: state = MSP_IDLE; break;
    }
}

void msp_task(void *pvParameters) {
    init_uart();
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    
    rc_data.roll = 1500; 
    rc_data.pitch = 1500; 
    rc_data.yaw = 1500; 
    rc_data.throttle = 1000; // Zero throttle
    rc_data.aux1 = 1000;     // Disarmed
    rc_data.aux2 = 1000;
    rc_data.aux3 = 1000;
    rc_data.aux4 = 1000;

    int loop_counter = 0;

    while (1) {
        int len = uart_read_bytes(MSP_UART_NUM, data, BUF_SIZE, 0); // Non-blocking read
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                process_msp_byte(data[i]);
            }
        }

        loop_counter++;

        rc_data.roll = 1500 + (int)(sin(loop_counter * 0.1) * 200);

        rc_data.throttle = 1100; 

        rc_data.aux1 = 2000; // Example: ARM (if AUX1 is arm switch)
        rc_data.aux2 = 2000; // Example: Activate MSP OVERRIDE Mode

        msp_set_raw_rc(&rc_data);

        if (loop_counter % 20 == 0) { // Print every 1s (approx)
             ESP_LOGI(TAG, "SENDING -> R:%d P:%d Y:%d T:%d A1:%d", 
                rc_data.roll, rc_data.pitch, rc_data.yaw, rc_data.throttle, rc_data.aux1);
        }

        vTaskDelay(LOOP_RATE_MS / portTICK_PERIOD_MS);
    }
    free(data);
}

void app_main(void) {
    xTaskCreate(msp_task, "msp_task", 4096, NULL, 5, NULL);
}