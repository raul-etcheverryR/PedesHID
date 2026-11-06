#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

#if CONFIG_BT_SDP_COMMON_ENABLED
#include "esp_sdp_api.h"
#endif

#include "esp_hidd.h"
#include "esp_hid_gap.h"

static const char *TAG = "HID_DEV_DEMO";

typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_bt_hid_param = {0};

const unsigned char mouseReportMap[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)
    0x09, 0x01, // USAGE (Pointer)
    0xa1, 0x00, // COLLECTION (Physical)

    0x05, 0x09, // USAGE_PAGE (Button)
    0x19, 0x01, // USAGE_MINIMUM (Button 1)
    0x29, 0x03, // USAGE_MAXIMUM (Button 3)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0x01, // LOGICAL_MAXIMUM (1)
    0x95, 0x03, // REPORT_COUNT (3)
    0x75, 0x01, // REPORT_SIZE (1)
    0x81, 0x02, // INPUT (Data,Var,Abs)

    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x05, // REPORT_SIZE (5)
    0x81, 0x03, // INPUT (Cnst,Var,Abs)

    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x30, // USAGE (X)
    0x09, 0x31, // USAGE (Y)
    0x09, 0x38, // USAGE (Wheel)
    0x15, 0x81, // LOGICAL_MINIMUM (-127)
    0x25, 0x7f, // LOGICAL_MAXIMUM (127)
    0x75, 0x08, // REPORT_SIZE (8)
    0x95, 0x03, // REPORT_COUNT (3)
    0x81, 0x06, // INPUT (Data,Var,Rel)

    0xc0, // END_COLLECTION
    0xc0  // END_COLLECTION
};

static esp_hid_raw_report_map_t bt_report_maps[] = {
    {
        .data = mouseReportMap,
        .len = sizeof(mouseReportMap)
    },
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "ESP BT HID1",
    .manufacturer_name  = "Espressif",
    .serial_number      = "1234567890",
    .report_maps        = bt_report_maps,
    .report_maps_len    = 1
};

// send the buttons, change in x, and change in y
void send_mouse(uint8_t buttons, char dx, char dy, char wheel)
{
    static uint8_t buffer[4] = {0};

    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;

    esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, 0, buffer, 4);
}

#define HALL_A_D0 GPIO_NUM_19
#define HALL_B_D0 GPIO_NUM_21

int64_t ULTIMO_EVENTO_HALL_A = 0;
int64_t ULTIMO_EVENTO_HALL_B = 0;

const int64_t DEBOUNCE_US = 8000; // 8 ms

QueueHandle_t FILA_HALL;


typedef struct {
    int hall;
    int estado;
} hall_event_t;

static void IRAM_ATTR hall_isr_handler(void *arg){
    int HALL = (int)arg;
    hall_event_t EVENTO = {
        .hall = HALL,
        .estado = gpio_get_level( HALL == 1 ? HALL_A_D0 : HALL_B_D0)
    };

    xQueueSendFromISR(FILA_HALL, &EVENTO, NULL);

}

void botao_hall_teste(void *arg){
    FILA_HALL = xQueueCreate( 128, sizeof(hall_event_t));
    hall_event_t EVENTO;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_A_D0 | 1ULL << HALL_B_D0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE

    };

    gpio_config(&io_conf);


    gpio_install_isr_service(0);
    gpio_isr_handler_add(
            HALL_A_D0,
            hall_isr_handler,
            (void *)1

    );

    gpio_isr_handler_add(
        HALL_B_D0,
        hall_isr_handler,
        (void *)2

    );

    bool HALL_A_PRESSIONADO = false;
    bool HALL_B_PRESSIONADO = false;

    while (1) {
        xQueueReceive( FILA_HALL, &EVENTO, portMAX_DELAY);
        int64_t AGORA = esp_timer_get_time();
        printf( "HALL=%d ESTADO=%d\n", EVENTO.hall, EVENTO.estado);

        if(EVENTO.hall == 1){
            if(( AGORA - ULTIMO_EVENTO_HALL_A) < DEBOUNCE_US){
                continue;

            }
            ULTIMO_EVENTO_HALL_A = AGORA;

            if(EVENTO.estado == 1 && !HALL_A_PRESSIONADO){
                HALL_A_PRESSIONADO = true; 
                send_mouse( 1, 0, 0, 0);
                printf("A pressionado\n");

            } else if(EVENTO.estado == 0 && HALL_A_PRESSIONADO){
                HALL_A_PRESSIONADO = false;
                send_mouse( 0, 0, 0, 0);
                printf("CLIQUE ESQ\n");

            }

        }

        if(EVENTO.hall == 2){
            if(( AGORA - ULTIMO_EVENTO_HALL_B) < DEBOUNCE_US){
                continue;

            }
            ULTIMO_EVENTO_HALL_B = AGORA;

            if(EVENTO.estado == 1 && !HALL_B_PRESSIONADO){
                HALL_B_PRESSIONADO = true;
                send_mouse( 2, 0, 0, 0);
                printf("B pressionado\n");  

            } else if(EVENTO.estado == 0 && HALL_B_PRESSIONADO){
                HALL_B_PRESSIONADO = false;
                send_mouse( 0, 0, 0, 0);
                printf("CLIQUE DIR\n");

            }

        }

    }

}

#define ANALOG_EIXO_X GPIO_NUM_2
#define ANALOG_EIXO_Y  GPIO_NUM_15

void cursor_analog_teste(void *arg){

    printf("ENTREI NA TASK DO JOYSTICK\n");

    adc_oneshot_unit_handle_t adc_handle_analog;
    adc_oneshot_unit_init_cfg_t init_config_analog = {.unit_id = ADC_UNIT_2, };
    adc_oneshot_new_unit( &init_config_analog, &adc_handle_analog);
    adc_oneshot_chan_cfg_t config_analog = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12, };
    adc_oneshot_config_channel( adc_handle_analog, ADC_CHANNEL_2, &config_analog);
    adc_oneshot_config_channel( adc_handle_analog, ADC_CHANNEL_3, &config_analog);

    int VALOR_X;
    int VALOR_Y;

    int DESLOC_EIXO_X;
    int DESLOC_EIXO_Y;

    printf("X = %d | Y = %d\n", VALOR_X, VALOR_Y);

    while(1){

        adc_oneshot_read(adc_handle_analog, ADC_CHANNEL_2, &VALOR_X);
        adc_oneshot_read(adc_handle_analog, ADC_CHANNEL_3, &VALOR_Y);

        if( VALOR_X <= 1000){
            DESLOC_EIXO_X = -2;

        } else if ( VALOR_X >= 3000){
            DESLOC_EIXO_X =  2;

        } else {
            DESLOC_EIXO_X = 0;

        }
        
        if( VALOR_Y <= 1000){
            DESLOC_EIXO_Y = -2;

        } else if ( VALOR_Y >= 3000){
            DESLOC_EIXO_Y =  2;

        } else {
            DESLOC_EIXO_Y = 0;

        }

        //send_mouse( 0, DESLOC_EIXO_X, DESLOC_EIXO_Y, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void bt_hid_demo_task(void *pvParameters)
{
    static const char* help_string = "########################################################################\n"\
    "BT hid mouse demo usage:\n"\
    "You can input these value to simulate mouse: 'q', 'w', 'e', 'a', 's', 'd', 'h'\n"\
    "q -- click the left key\n"\
    "w -- move up\n"\
    "e -- click the right key\n"\
    "a -- move left\n"\
    "s -- move down\n"\
    "d -- move right\n"\
    "h -- show the help\n"\
    "########################################################################\n";

    printf("%s\n", help_string);

    char c;

    while (1) {
        c = fgetc(stdin);

        switch (c) {
        case 'q':
            send_mouse(1, 0, 0, 0);
            break;

        case 'w':
            send_mouse(0, 0, -10, 0);
            break;

        case 'e':
            send_mouse(2, 0, 0, 0);
            break;

        case 'a':
            send_mouse(0, -10, 0, 0);
            break;

        case 's':
            send_mouse(0, 0, 10, 0);
            break;

        case 'd':
            send_mouse(0, 10, 0, 0);
            break;

        case 'h':
            printf("%s\n", help_string);
            break;

        default:
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void cursor_analog_teste_task_start(void){
    xTaskCreate(cursor_analog_teste, "cursor_analog_teste", 2 * 1024, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

void botao_hall_teste_task_start(void){
    xTaskCreate( botao_hall_teste, "botao_hall_teste", 2 * 1024, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

void bt_hid_task_start_up(void)
{
    xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_hid_param.task_hdl);
    return;
}

void bt_hid_task_shut_down(void)
{
    if (s_bt_hid_param.task_hdl) {
        vTaskDelete(s_bt_hid_param.task_hdl);
        s_bt_hid_param.task_hdl = NULL;
    }
}

static void bt_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BT";

    switch (event) {

    case ESP_HIDD_START_EVENT: {

        if (param->start.status == ESP_OK) {

            ESP_LOGI(TAG, "START OK");
            ESP_LOGI(TAG, "Setting to connectable, discoverable");

            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_GENERAL_DISCOVERABLE
            );

        } else {

            ESP_LOGE(TAG, "START failed!");
        }

        break;
    }

    case ESP_HIDD_CONNECT_EVENT: {

        if (param->connect.status == ESP_OK) {

            ESP_LOGI(TAG, "CONNECT OK");
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");

            esp_bt_gap_set_scan_mode(
                ESP_BT_NON_CONNECTABLE,
                ESP_BT_NON_DISCOVERABLE
            );

            bt_hid_task_start_up();
            cursor_analog_teste_task_start();
            botao_hall_teste_task_start();

        } else {

            ESP_LOGE(TAG, "CONNECT failed!");
        }

        break;
    }

    case ESP_HIDD_PROTOCOL_MODE_EVENT: {

        ESP_LOGI(
            TAG,
            "PROTOCOL MODE[%u]: %s",
            param->protocol_mode.map_index,
            param->protocol_mode.protocol_mode ? "REPORT" : "BOOT"
        );

        break;
    }

    case ESP_HIDD_OUTPUT_EVENT: {

        ESP_LOGI(
            TAG,
            "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:",
            param->output.map_index,
            esp_hid_usage_str(param->output.usage),
            param->output.report_id,
            param->output.length
        );

        ESP_LOG_BUFFER_HEX(
            TAG,
            param->output.data,
            param->output.length
        );

        break;
    }

    case ESP_HIDD_FEATURE_EVENT: {

        ESP_LOGI(
            TAG,
            "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:",
            param->feature.map_index,
            esp_hid_usage_str(param->feature.usage),
            param->feature.report_id,
            param->feature.length
        );

        ESP_LOG_BUFFER_HEX(
            TAG,
            param->feature.data,
            param->feature.length
        );

        break;
    }

    case ESP_HIDD_DISCONNECT_EVENT: {

        if (param->disconnect.status == ESP_OK) {

            ESP_LOGI(TAG, "DISCONNECT OK");

            bt_hid_task_shut_down();

            ESP_LOGI(TAG, "Setting to connectable, discoverable again");

            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_GENERAL_DISCOVERABLE
            );

        } else {

            ESP_LOGE(TAG, "DISCONNECT failed!");
        }

        break;
    }

    case ESP_HIDD_STOP_EVENT: {

        ESP_LOGI(TAG, "STOP");

        break;
    }

    default:
        break;
    }

    return;
}

#if CONFIG_BT_SDP_COMMON_ENABLED

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    switch (event) {

    case ESP_SDP_INIT_EVT:

        ESP_LOGI(TAG, "ESP_SDP_INIT_EVT: status:%d", param->init.status);

        if (param->init.status == ESP_SDP_SUCCESS) {

            esp_bluetooth_sdp_dip_record_t dip_record = {
                .hdr =
                {
                    .type = ESP_SDP_TYPE_DIP_SERVER,
                },
                .vendor = bt_hid_config.vendor_id,
                .vendor_id_source = ESP_SDP_VENDOR_ID_SRC_BT,
                .product = bt_hid_config.product_id,
                .version = bt_hid_config.version,
                .primary_record = true,
            };

            esp_sdp_create_record(
                (esp_bluetooth_sdp_record_t *)&dip_record
            );
        }

        break;

    case ESP_SDP_DEINIT_EVT:

        ESP_LOGI(
            TAG,
            "ESP_SDP_DEINIT_EVT: status:%d",
            param->deinit.status
        );

        break;

    case ESP_SDP_SEARCH_COMP_EVT:

        ESP_LOGI(
            TAG,
            "ESP_SDP_SEARCH_COMP_EVT: status:%d",
            param->search.status
        );

        break;

    case ESP_SDP_CREATE_RECORD_COMP_EVT:

        ESP_LOGI(
            TAG,
            "ESP_SDP_CREATE_RECORD_COMP_EVT: status:%d, handle:0x%x",
            param->create_record.status,
            param->create_record.record_handle
        );

        break;

    case ESP_SDP_REMOVE_RECORD_COMP_EVT:

        ESP_LOGI(
            TAG,
            "ESP_SDP_REMOVE_RECORD_COMP_EVT: status:%d",
            param->remove_record.status
        );

        break;

    default:
        break;
    }
}

#endif /* CONFIG_BT_SDP_COMMON_ENABLED */


void app_main(void)
{
    esp_err_t ret;

#if HID_DEV_MODE == HIDD_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID device or BLE!");
    return;
#endif

    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());

        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);

    ret = esp_hid_gap_init(HID_DEV_MODE);

    ESP_ERROR_CHECK(ret);

#if CONFIG_BT_HID_DEVICE_ENABLED

    ESP_LOGI(TAG, "setting device name");

    esp_bt_gap_set_device_name(bt_hid_config.device_name);

    ESP_LOGI(TAG, "setting cod major, peripheral");

    esp_bt_cod_t cod = {0};

    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_POINTING;

    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "setting bt device");

    ESP_ERROR_CHECK(
        esp_hidd_dev_init(
            &bt_hid_config,
            ESP_HID_TRANSPORT_BT,
            bt_hidd_event_callback,
            &s_bt_hid_param.hid_dev
        )
    );

    

#if CONFIG_BT_SDP_COMMON_ENABLED

    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());

#endif /* CONFIG_BT_SDP_COMMON_ENABLED */

#endif /* CONFIG_BT_HID_DEVICE_ENABLED */
}