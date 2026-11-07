#include <math.h>

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

#include "esp_err.h"

#if CONFIG_BT_SDP_COMMON_ENABLED
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
    .device_name        = "Pedes HID",
    .manufacturer_name  = "Espressif",
    .serial_number      = "1234567890",
    .report_maps        = bt_report_maps,
    .report_maps_len    = 1
};

int BOTOES = 0;
int DESLOC_EIXO_X = 0;
int DESLOC_EIXO_Y = 0;

// Identificador da tarefa de leitura dos sensores IR
static TaskHandle_t ir_task_handle = NULL;

// Envia um relatório HID: botoes, movimento X/Y e rolagem.
// O buffer e local para evitar que tarefas diferentes sobrescrevam
// o mesmo vetor enquanto montam seus relatorios.
esp_err_t send_mouse(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
    if (s_bt_hid_param.hid_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[4] = {
        buttons,
        (uint8_t)dx,
        (uint8_t)dy,
        (uint8_t)wheel
    };

    return esp_hidd_dev_input_set(
        s_bt_hid_param.hid_dev, 0, 0, buffer, sizeof(buffer)
    );
}

// Mapeamento dos pinos do Hall
#define HALL_A_D0 GPIO_NUM_26
#define HALL_B_D0 GPIO_NUM_27

// Mapeamento dos pinos do IR
#define IR_A_OUT GPIO_NUM_34
#define IR_B_OUT GPIO_NUM_35

QueueHandle_t FILA_HALL; // Fila responsável por armazenar eventos da interrupção

typedef struct { // Estrutura que representa o sensor que gerou o evento
    int hall;    // e seu estado
    int estado;
} hall_event_t;

static void IRAM_ATTR hall_isr_handler(void *arg){ // Rotina de serviço de interrupção ISR, executa
                                                   // quando há mudança no nível lógico do pino
    int HALL = (int)arg; // Identifica qual sensor gerou a interrupção
    hall_event_t EVENTO = {
        .hall = HALL,       // Guarda sensor e estado em estrutura para a fila
        .estado = gpio_get_level( HALL == 1 ? HALL_A_D0 : HALL_B_D0) // realiza a leitura do pino de acordo com
                                                                     // argumento HALL recebido
    };

    xQueueSendFromISR(FILA_HALL, &EVENTO, NULL); // Enfileira estrutura na fila
}




// Quantidade de transicoes validas para gerar uma unidade de rolagem.
// 4 = um ciclo completo de quadratura por unidade HID.
#define TRANSICOES_POR_ROLAGEM 4

static void scroll_IR(void *arg)
{
    ESP_LOGI(TAG, "[IR] Iniciando encoder optico...");

    gpio_config_t config = {
        .pin_bit_mask = (1ULL << IR_A_OUT) | (1ULL << IR_B_OUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t erro = gpio_config(&config);

    if (erro != ESP_OK) {
        ESP_LOGE(TAG, "[IR] Erro na configuracao: %s",
                 esp_err_to_name(erro));

        ir_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Estados: 00 -> 01 -> 11 -> 10 -> 00.
    // Cada transicao valida soma +1 ou -1, conforme o sentido.
    static const int transicoes[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };

    int a = gpio_get_level(IR_A_OUT);
    int b = gpio_get_level(IR_B_OUT);
    int estado_anterior = (a << 1) | b;
    int passos = 0;

    ESP_LOGI(TAG, "[ENCODER] Estado inicial: A=%d B=%d", a, b);
    ESP_LOGI(TAG, "[SCROLL] Encoder pronto. Uma unidade a cada %d transicoes.",
             TRANSICOES_POR_ROLAGEM);

    while (1) {
        a = gpio_get_level(IR_A_OUT);
        b = gpio_get_level(IR_B_OUT);

        int estado_atual = (a << 1) | b;

        if (estado_atual != estado_anterior) {
            int indice = (estado_anterior << 2) | estado_atual;
            int delta = transicoes[indice];

            if (delta == 0) {
                // Transicao diagonal: os dois bits mudaram entre leituras.
                // Nao e possivel determinar o sentido com seguranca.
                ESP_LOGW(TAG,
                         "[ENCODER] Transicao invalida: %d%d -> %d%d",
                         (estado_anterior >> 1) & 1,
                         estado_anterior & 1,
                         a, b);
                passos = 0;
            } else {
                passos += delta;

                if (passos >= TRANSICOES_POR_ROLAGEM) {
                    passos -= TRANSICOES_POR_ROLAGEM;

                    // No HID, valor positivo representa rolar para frente/cima.
                    esp_err_t envio = send_mouse((uint8_t)BOTOES, 0, 0, 1);

                    if (envio == ESP_OK) {
                        ESP_LOGI(TAG, "[SCROLL] Rolagem para cima (+1)");
                    } else {
                        ESP_LOGW(TAG, "[SCROLL] Falha ao enviar rolagem: %s",
                                 esp_err_to_name(envio));
                    }
                } else if (passos <= -TRANSICOES_POR_ROLAGEM) {
                    passos += TRANSICOES_POR_ROLAGEM;

                    // No HID, valor negativo representa rolar para tras/baixo.
                    esp_err_t envio = send_mouse((uint8_t)BOTOES, 0, 0, -1);

                    if (envio == ESP_OK) {
                        ESP_LOGI(TAG, "[SCROLL] Rolagem para baixo (-1)");
                    } else {
                        ESP_LOGW(TAG, "[SCROLL] Falha ao enviar rolagem: %s",
                                 esp_err_to_name(envio));
                    }
                }
            }

            estado_anterior = estado_atual;
        }

        // Manter 10 ms: esta temporizacao ja foi testada sem watchdog.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Função de leitura dos botões hall (esq & dir)
#define HALL_DEBOUNCE_MS 20
void botao_hall(void *arg) {
    FILA_HALL = xQueueCreate(64, sizeof(hall_event_t)); // Cria a fila na task
    hall_event_t EVENTO;                                // Cria estrutura de eventos
    int estado_anterior_A = -1;                         // Inicia variaveis de 
    int estado_anterior_B = -1;                         // estado anterior

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_A_D0) | (1ULL << HALL_B_D0), // Mascara para representação de ambos pinos
        .mode = GPIO_MODE_INPUT,   // seleciona pinos como entradas digitais
        .pull_up_en = GPIO_PULLUP_DISABLE, // desabilita pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // desabilita pull-down
        .intr_type = GPIO_INTR_ANYEDGE // Gera interrupção suba e queda do sinal logico
    };

    gpio_config(&io_conf); // aplica configurações anteriores de io_conf
    gpio_install_isr_service(0); // Habilita interrupções

    gpio_isr_handler_add( HALL_A_D0, hall_isr_handler, (void *)1); // Associa hall_isr_handler ao sensor A - argumento 1 na chamada da ISR
    gpio_isr_handler_add( HALL_B_D0, hall_isr_handler, (void *)2); // O mesmo ao sensor B - argumento 2 na chamada da ISR

    while (1) {
        xQueueReceive(FILA_HALL, &EVENTO, portMAX_DELAY);
        gpio_num_t pino = (EVENTO.hall == 1) // Identifica o pino correspondente
            ? HALL_A_D0
            : HALL_B_D0;

        vTaskDelay(pdMS_TO_TICKS(HALL_DEBOUNCE_MS)); // Aguarda para permitir que o sinal estabilize

        int estado_atual = gpio_get_level(pino); // Lê novamente o nível físico do sensor

        if (estado_atual != EVENTO.estado) { // Se o sinal mudou durante a espera, descarta o evento
            continue;
        }
        int *estado_anterior = (EVENTO.hall == 1) // Filtra eventos repetidos
            ? &estado_anterior_A
            : &estado_anterior_B;

        if (EVENTO.estado == *estado_anterior) { // Se o sinal mantém-se o mesmo descarta o processamento
            continue;
        }

        *estado_anterior = EVENTO.estado; // Atualiza estado_anterior

        printf("HALL=%d ESTADO=%d\n",
               EVENTO.hall, EVENTO.estado);

        if (EVENTO.hall == 1) { // Verifica se o evento pretence ao sensor A
            if (EVENTO.estado == 1) {   // Caso estado estja em 1 BOTOES (argumento do relatorio HID) 
                BOTOES = 1;             // é setado para 1
                send_mouse( BOTOES, DESLOC_EIXO_X, DESLOC_EIXO_Y, 0); // Envio do relatório HID com o estado das variaveis compartilhadas/globais (cursor_analog)
                printf("PRESSIONANDO ESQ\n");
            } else if (BOTOES == 1) {   // Caso esteja em 1 é setado para 0
                BOTOES = 0;             // e o ciclo de clique e finalizado
                send_mouse( BOTOES, DESLOC_EIXO_X, DESLOC_EIXO_Y, 0);// Envio do relatório HID
                printf("CLIQUE ESQ\n");
            }
        } else if (EVENTO.hall == 2) {  // Mesmo tratamento para o sensor B
            if (EVENTO.estado == 1) {
                BOTOES = 2;
                send_mouse( BOTOES, DESLOC_EIXO_X, DESLOC_EIXO_Y, 0); // Envio do relatório HID
                printf("PRESSIONANDO DIR\n");
            } else if (BOTOES == 2) {
                BOTOES = 0;
                send_mouse( BOTOES, DESLOC_EIXO_X, DESLOC_EIXO_Y, 0); // Envio do relatório HID
                printf("CLIQUE DIR\n");
            }
        }
    }
}

// Função de leitura do joystick cursor

#define CENTRO_X 2048.0f // Valor central do eixo X do joystick
#define CENTRO_Y 2048.0f // Valor central do eixo Y do joystick
#define DEADZONE 0.3121f // Valor da deadzone radial, 0.3121f corresponde a 32% do valor máximo do eixo (4095)
void cursor_analog(void *arg){
    adc_oneshot_unit_handle_t adc_handle_analog; // declara o identificador para unidade ADC
    adc_oneshot_unit_init_cfg_t init_config_analog = {.unit_id = ADC_UNIT_1, }; // Seleciona a unidade ADC1 do ESP-32
    adc_oneshot_new_unit( &init_config_analog, &adc_handle_analog); // inicia a unidade ADC selecionada e atribui o identificador
                                                                    // adc_handle_analog já declarado
    adc_oneshot_chan_cfg_t config_analog = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12, }; // Defnine configurações
                                                                        // de leitura dos canais: 12 bits (0 - 4095)
    adc_oneshot_config_channel( adc_handle_analog, ADC_CHANNEL_4, &config_analog); // Configura canal ADC4 para leitura analógica
    adc_oneshot_config_channel( adc_handle_analog, ADC_CHANNEL_5, &config_analog); // O mesmo para canal ADC5 - (Pinos GPIO: 32, 33)

    int VALOR_X = 0;
    int VALOR_Y = 0;
    int RESTO_X = 0;
    int RESTO_Y = 0;

    printf("X = %d | Y = %d\n", VALOR_X, VALOR_Y);

    while(1){
        adc_oneshot_read(adc_handle_analog, ADC_CHANNEL_4, &VALOR_X); // Leitura do canal correspondente ao eixo X
        adc_oneshot_read(adc_handle_analog, ADC_CHANNEL_5, &VALOR_Y); // Leitura de Y

        // Centralização e normalização
        float X = (VALOR_X - CENTRO_X) / CENTRO_X;
        float Y = (VALOR_Y - CENTRO_Y) / CENTRO_Y;

        // Módulo do vetor
        float MODULO = sqrtf(X * X + Y * Y);

        // Verificação da deadzone radial
        if (MODULO <= DEADZONE) {
            DESLOC_EIXO_X = 0;
            DESLOC_EIXO_Y = 0;
            RESTO_X = 0;
            RESTO_Y = 0;

        } else {

            DESLOC_EIXO_X += RESTO_X;
            DESLOC_EIXO_Y += RESTO_Y;

            // Sentido negativo
            if (VALOR_Y <= 500){
                DESLOC_EIXO_Y = -3;
                RESTO_Y += VALOR_Y - 500;

            } else if (VALOR_Y < 1000){
                DESLOC_EIXO_Y = -2;
                RESTO_Y += VALOR_Y - 1000;

            } else if(VALOR_Y < 1500){
                DESLOC_EIXO_Y = -1;
                RESTO_Y += VALOR_Y - 1500;

            } else if (VALOR_Y <= 2500){ // Zona morta
                DESLOC_EIXO_Y = 0;
                RESTO_Y = 0;

            } else if (VALOR_Y <= 3000){ // Sentido positivo
                DESLOC_EIXO_Y = 1;
                RESTO_Y += VALOR_Y - 2501;

            } else if (VALOR_Y <= 3500){
                DESLOC_EIXO_Y = 2;
                RESTO_Y += VALOR_Y - 3001;

            } else if (VALOR_Y > 3500){
                DESLOC_EIXO_Y = 3;
                RESTO_Y += VALOR_Y - 3501;

            }

            // Sentido negativo
            if (VALOR_X <= 500){
                DESLOC_EIXO_X = -3;
                RESTO_X += VALOR_X - 500;

            } else if (VALOR_X < 1000){
                DESLOC_EIXO_X = -2;
                RESTO_X += VALOR_X - 1000;

            } else if(VALOR_X < 1500){
                DESLOC_EIXO_X = -1;
                RESTO_X += VALOR_X - 1500;

            } else if (VALOR_X <= 2500){ // Zona morta
                DESLOC_EIXO_X = 0;
                RESTO_X = 0;

            } else if (VALOR_X <= 3000){ // Sentido positivo
                DESLOC_EIXO_X = 1;
                RESTO_X += VALOR_X - 2501;

            } else if (VALOR_X <= 3500){
                DESLOC_EIXO_X = 2;
                RESTO_X += VALOR_X - 3001;

            } else if (VALOR_X > 3500){
                DESLOC_EIXO_X = 3;
                RESTO_X += VALOR_X - 3501;

            }
        }
        
        send_mouse(BOTOES, DESLOC_EIXO_X, DESLOC_EIXO_Y, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS); // Libera CPU por no mínimo 10 ms
    }
}

void cursor_analog_task_start(void){ // Configuração da task cursor_analog_teste
    xTaskCreate(cursor_analog, "cursor_analog", 2 * 1024, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

void botao_hall_task_start(void){ // Configuração da task botao_hall_teste | dps tem que tirar esse "teste" do nome delas kkk
    xTaskCreate( botao_hall, "botao_hall", 2 * 1024, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

void scroll_IR_task_start(void)
{
    printf("[IR] Solicitando criacao da tarefa...\n");

    if (ir_task_handle != NULL) {
        printf("[IR] A tarefa ja foi criada.\n");
        return;
    }

    BaseType_t resultado = xTaskCreate(
        scroll_IR,
        "scroll_IR",
        3072,
        NULL,
        5,
        &ir_task_handle
    );

    if (resultado == pdPASS) {
        printf("[IR] Tarefa criada com sucesso!\n");
    } else {
        ir_task_handle = NULL;
        printf("[IR] ERRO: nao foi possivel criar a tarefa!\n");
    }
}

void bt_hid_task_shut_down(void) // aqui faz parte do exemplo mas acho que é o procedimento quando tu encerra a conexão bluetooth, seta as flags que são lidas em outros trechos do código
{                                // É uma função não uma task
    if (s_bt_hid_param.task_hdl) {
        vTaskDelete(s_bt_hid_param.task_hdl);
        s_bt_hid_param.task_hdl = NULL;
    }
}

static void bt_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) // Mais configurações do BT (bluetooth) e alguns log que tu ve no monitor quando ele
{                                                                                                           // ta pronto pra realizar uma conexão
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BT";

    switch (event) {

    case ESP_HIDD_START_EVENT: {

        if (param->start.status == ESP_OK) { // Nesse caso o esp ta pronto para a conexão

            ESP_LOGI(TAG, "START OK");
            ESP_LOGI(TAG, "Setting to connectable, discoverable"); // Isso a gente vê quando inicializa o monitor indicando ue ja da pra estabelecer a conexão

            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_GENERAL_DISCOVERABLE
            );

        } else {

            ESP_LOGE(TAG, "START failed!");
        }

        break;
    }

    case ESP_HIDD_CONNECT_EVENT: { // Nesse caso a conexão foi estabelecida

        if (param->connect.status == ESP_OK) {

            ESP_LOGI(TAG, "CONNECT OK");
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");

            esp_bt_gap_set_scan_mode(
                ESP_BT_NON_CONNECTABLE,
                ESP_BT_NON_DISCOVERABLE
            );

            ESP_LOGI(TAG, "Bluetooth conectado: iniciando sensores!");

            cursor_analog_task_start(); // se criar uma nova task e não configurar e nem inicializar ela em algum case, ela nunca vai rodar
            botao_hall_task_start();    // aqui nos temos as 2 que estamos usando, antigamente estava aqui também a task do exemplo deles
            scroll_IR_task_start();

        } else {

            ESP_LOGE(TAG, "CONNECT failed!");
        }

        break;
    }

    case ESP_HIDD_PROTOCOL_MODE_EVENT: { // Outros cases... Vale investigar exatamente o que são cada um

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

void app_main(void) // aqui é a Main do código
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

    ESP_ERROR_CHECK( // Aqui a Main inicializa bt_hid_event_callback
        esp_hidd_dev_init( // que é a função verifica o estado da conexão e inicializa as tasks, comentarios e função a partir da linha: 299
            &bt_hid_config,
            ESP_HID_TRANSPORT_BT,
            bt_hidd_event_callback, // Aqui essa função e inicializada
            &s_bt_hid_param.hid_dev
        )
    );

    

#if CONFIG_BT_SDP_COMMON_ENABLED

    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());

#endif /* CONFIG_BT_SDP_COMMON_ENABLED */

#endif /* CONFIG_BT_HID_DEVICE_ENABLED */

}