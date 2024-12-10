#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "math.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BUF_SIZE (128)       // buffer size
#define TXD_PIN 1            // UART TX pin
#define RXD_PIN 3            // UART RX pin
#define UART_NUM UART_NUM_0  // UART port number
#define BAUD_RATE 115200     // Baud rate
#define M_PI 3.14159265358979323846

#define I2C_MASTER_SCL_IO GPIO_NUM_22  // GPIO pin
#define I2C_MASTER_SDA_IO GPIO_NUM_21  // GPIO pin
#define I2C_MASTER_FREQ_HZ 10000
#define BME_ESP_SLAVE_ADDR 0x76
#define WRITE_BIT 0x0
#define READ_BIT 0x1
#define ACK_CHECK_EN 0x0
#define EXAMPLE_I2C_ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

// WiFi and TCP configurations
#define WIFI_SSID "tic3_wija"         // SSID Wifi
#define WIFI_PASS ""     // Contraseña Wifi
#define SERVER_IP "10.42.0.1"  // Dirección IP de la Raspberry Pi
#define SERVER_PORT 12345         // Puerto donde está escuchando el servidor en la Raspberry Pi

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// TCP socket
static int sock = -1;

esp_err_t ret = ESP_OK;
esp_err_t ret2 = ESP_OK;

uint16_t val0[6];

float task_delay_ms = 1000;

volatile bool monitoreo_activo = false;
volatile int modo_operacion = 1;         // 1: Modo Datos en Bruto, 2: Modo Datos Procesados
volatile int frecuencia_muestreo = 1000; // en milisegundos
volatile int ventana_tiempo = 7000;      // en milisegundos

static const char *TAG = "ESP32_TCP_CLIENT";

// Declaración de funciones
float leer_temperatura(void);
float leer_humedad(void);

// Declaración de los prototipos de las tareas
void tcp_client_receive_task(void *pvParameters);
void send_data_task(void *pvParameters);

esp_err_t sensor_init(void) {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

esp_err_t bme_i2c_read(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme_i2c_write(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_wr, size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------ BME 688 ------------- //
uint8_t calc_gas_wait(uint16_t dur) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1176
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur >> 2;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

uint8_t calc_res_heat(uint16_t temp) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1145
    uint8_t heatr_res;
    uint8_t amb_temp = 25;

    uint8_t reg_par_g1 = 0xED;
    uint8_t par_g1;
    bme_i2c_read(I2C_NUM_0, &reg_par_g1, &par_g1, 1);

    uint8_t reg_par_g2_lsb = 0xEB;
    uint8_t par_g2_lsb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_lsb, &par_g2_lsb, 1);
    uint8_t reg_par_g2_msb = 0xEC;
    uint8_t par_g2_msb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_msb, &par_g2_msb, 1);
    uint16_t par_g2 = (int16_t)(CONCAT_BYTES(par_g2_msb, par_g2_lsb));

    uint8_t reg_par_g3 = 0xEE;
    uint8_t par_g3;
    bme_i2c_read(I2C_NUM_0, &reg_par_g3, &par_g3, 1);

    uint8_t reg_res_heat_range = 0x02;
    uint8_t res_heat_range;
    uint8_t mask_res_heat_range = (0x3 << 4);
    uint8_t tmp_res_heat_range;

    uint8_t reg_res_heat_val = 0x00;
    uint8_t res_heat_val;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) {
        temp = 400;
    }

    bme_i2c_read(I2C_NUM_0, &reg_res_heat_range, &tmp_res_heat_range, 1);
    bme_i2c_read(I2C_NUM_0, &reg_res_heat_val, &res_heat_val, 1);
    res_heat_range = (mask_res_heat_range & tmp_res_heat_range) >> 4;

    var1 = (((int32_t)amb_temp * par_g3) / 1000) * 256;
    var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (res_heat_range + 4));
    var5 = (131 * res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

int bme_get_chipid(void) {
    uint8_t reg_id = 0xd0;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_id, &tmp, 1);
    printf("Valor de CHIPID: %2X \n\n", tmp);

    if (tmp == 0x61) {
        printf("Chip BME688 reconocido.\n\n");
        return 0;
    } else {
        printf("Chip BME688 no reconocido. \nCHIP ID: %2x\n\n", tmp);  // %2X
    }

    return 1;
}

int bme_softreset(void) {
    uint8_t reg_softreset = 0xE0, val_softreset = 0xB6;

    ret = bme_i2c_write(I2C_NUM_0, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("\nError en softreset: %s \n", esp_err_to_name(ret));
        return 1;
    } else {
        printf("\nSoftreset: OK\n\n");
    }
    return 0;
}

void bme_forced_mode(void) {
    /*
    Fuente: Datasheet[19]
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=19

    Para configurar el BME688 en forced mode los pasos son:

    1. Set humidity oversampling to 1x     |-| 0b001 to osrs_h<2:0>
    2. Set temperature oversampling to 2x  |-| 0b010 to osrs_t<2:0>
    3. Set pressure oversampling to 16x    |-| 0b101 to osrs_p<2:0>

    4. Set gas duration to 100 ms          |-| 0x59 to gas_wait_0
    5. Set heater step size to 0           |-| 0x00 to res_heat_0
    6. Set number of conversion to 0       |-| 0b0000 to nb_conv<3:0> and enable gas measurements
    7. Set run_gas to 1                    |-| 0b1 to run_gas<5>

    8. Set operation mode                  |-| 0b01  to mode<1:0>

    */

    // Datasheet[33]
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0

    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // Configuramos el sensor de gas

    // 4. Seteamos gas_wait_0 a 100ms
    uint8_t gas_duration = calc_gas_wait(100);

    // 5. Seteamos res_heat_0
    uint8_t heater_step = calc_res_heat(300);

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000000;
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    bme_i2c_write(I2C_NUM_0, &gas_wait_0, &gas_duration, 1);
    bme_i2c_write(I2C_NUM_0, &res_heat_0, &heater_step, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 01, pasando primero por sleep
    uint8_t mode = 0b00000001;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) {
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03);
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03;
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_temp_celsius(uint32_t temp_adc) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion
    uint8_t addr_par_t1_lsb = 0xE9, addr_par_t1_msb = 0xEA;
    uint8_t addr_par_t2_lsb = 0x8A, addr_par_t2_msb = 0x8B;
    uint8_t addr_par_t3_lsb = 0x8C;
    uint16_t par_t1;
    uint16_t par_t2;
    uint16_t par_t3;

    uint8_t par[5];
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t3_lsb, par + 4, 1);

    par_t1 = (par[1] << 8) | par[0];
    par_t2 = (par[3] << 8) | par[2];
    par_t3 = par[4];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int t_fine;
    int calc_temp;

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = (int32_t)(var2 + var3);
    calc_temp = (((t_fine * 5) + 128) >> 8);
    return calc_temp;
}

float leer_temperatura(void) {
    uint8_t tmp;
    uint8_t forced_temp_addr[] = {0x22, 0x23, 0x24};

    uint32_t temp_adc = 0;

    // Seteamos el sensor en modo forzado para realizar la medición
    bme_forced_mode();

    // Lectura de la temperatura
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[0], &tmp, 1);
    temp_adc = temp_adc | (tmp << 12);
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[1], &tmp, 1);
    temp_adc = temp_adc | (tmp << 4);
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[2], &tmp, 1);
    temp_adc = temp_adc | (tmp & 0x0F);

    // Cálculo de la temperatura compensada
    uint32_t temp_comp = bme_temp_celsius(temp_adc);
    float temperatura = (float)temp_comp / 100;

    return temperatura;
}

float leer_humedad(void) {
    uint8_t tmp;

    // Direcciones de los registros de datos de humedad
    uint8_t forced_hum_addr[] = {0x25, 0x26};

    // Direcciones de los parámetros de calibración para humedad
    uint8_t par_h1_msb_addr = 0xE3, par_h1_lsb_addr = 0xE2;
    uint8_t par_h2_msb_addr = 0xE1, par_h2_lsb_addr = 0xE2;
    uint8_t par_h3_addr = 0xE4;
    uint8_t par_h4_addr = 0xE5;
    uint8_t par_h5_addr = 0xE6;
    uint8_t par_h6_addr = 0xE7;
    uint8_t par_h7_addr = 0xE8;

    // Variables para almacenar los parámetros de calibración
    uint16_t par_h1, par_h2;
    int8_t par_h3, par_h4, par_h5, par_h6, par_h7;

    // Leer parámetros de calibración
    uint8_t par[9];
    bme_i2c_read(I2C_NUM_0, &par_h1_lsb_addr, &par[0], 1);
    bme_i2c_read(I2C_NUM_0, &par_h1_msb_addr, &par[1], 1);
    bme_i2c_read(I2C_NUM_0, &par_h2_lsb_addr, &par[2], 1);
    bme_i2c_read(I2C_NUM_0, &par_h2_msb_addr, &par[3], 1);
    bme_i2c_read(I2C_NUM_0, &par_h3_addr, &par[4], 1);
    bme_i2c_read(I2C_NUM_0, &par_h4_addr, &par[5], 1);
    bme_i2c_read(I2C_NUM_0, &par_h5_addr, &par[6], 1);
    bme_i2c_read(I2C_NUM_0, &par_h6_addr, &par[7], 1);
    bme_i2c_read(I2C_NUM_0, &par_h7_addr, &par[8], 1);

    // Combinar datos para obtener los parámetros de calibración
    par_h1 = (par[1] << 4) | (par[0] & 0x0F);
    par_h2 = (par[3] << 4) | ((par[2] & 0xF0) >> 4);
    par_h3 = (int8_t)par[4];
    par_h4 = (int8_t)par[5];
    par_h5 = (int8_t)par[6];
    par_h6 = (int8_t)par[7];
    par_h7 = (int8_t)par[8];

    uint16_t hum_adc = 0;

    // Ponemos el sensor en modo forzado para realizar la medición
    bme_forced_mode();

    // Lectura de la humedad
    bme_i2c_read(I2C_NUM_0, &forced_hum_addr[0], &tmp, 1); // MSB
    hum_adc = (tmp << 8);
    bme_i2c_read(I2C_NUM_0, &forced_hum_addr[1], &tmp, 1); // LSB
    hum_adc |= tmp;

    // Cálculo de la humedad compensada (versión entera)
    float temperatura = leer_temperatura();
    int32_t temp_scaled = (int32_t)(temperatura * 100);
    int32_t var1 = ((int32_t)hum_adc - (((int32_t)par_h1 << 4))) - ((temp_scaled * (int32_t)par_h3) / ((int32_t)100));
    var1 = (var1 * 10) >> 1;
    int32_t var2 = ((int32_t)par_h2 * (((temp_scaled * (int32_t)par_h4) / (int32_t)100) +
                                       (((temp_scaled * ((temp_scaled * (int32_t)par_h5) / (int32_t)100)) >> 6) / (int32_t)100))) >> 10;
    int32_t var3 = ((int32_t)temp_scaled * (int32_t)par_h6) >> 10;
    int32_t var4 = (((var3 * var3) >> 10) * (int32_t)par_h7) >> 10;
    int32_t hum_comp = (var1 * var2) >> 12;
    hum_comp += var4;

    // Ajustar la humedad al rango 0% - 100%
    if (hum_comp > 100000) {
        hum_comp = 100000;
    } else if (hum_comp < 0) {
        hum_comp = 0;
    }

    hum_comp += (int32_t)(0.40 * 1024 * 100); // 40% en la escala usada

    // Ajustar la humedad al rango 0% - 100% después del ajuste
    if (hum_comp > 100000) {
        hum_comp = 100000;
    } else if (hum_comp < 0) {
        hum_comp = 0;
    }

    float humedad = (float)hum_comp / 1024;

    return humedad;
}

// Funciones WiFi y TCP
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Intentando reconectar al AP %s", WIFI_SSID);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Obtenida IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &event_handler,
                                        NULL,
                                        &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &event_handler,
                                        NULL,
                                        &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_sta terminado.");

    // Esperar hasta que se conecte a la red WiFi
    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);
}

void tcp_client_task(void *pvParameters) {
    char host_ip[] = SERVER_IP;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SERVER_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "No se pudo crear el socket: errno %d", errno);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Socket creado, conectando al servidor...");

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Error de conexión: errno %d", errno);
            close(sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Conectado al servidor");

        xTaskCreate(tcp_client_receive_task, "tcp_client_receive_task", 4096, NULL, 5, NULL);
        xTaskCreate(send_data_task, "send_data_task", 4096, NULL, 5, NULL);

        break;
    }

    vTaskDelete(NULL);
}

void tcp_client_receive_task(void *pvParameters) {
    char rx_buffer[128];
    char recv_buffer[256];  // Buffer para acumular datos
    int recv_len = 0;       // Longitud de datos en recv_buffer

    while (1) {
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Fallo en recv: errno %d", errno);
            break;
        } else if (len == 0) {
            ESP_LOGW(TAG, "Conexión cerrada");
            break;
        } else {
            // Añadir datos recibidos a recv_buffer
            if (recv_len + len >= sizeof(recv_buffer) - 1) {
                // Desbordamiento de buffer, resetear buffer
                ESP_LOGE(TAG, "Desbordamiento de buffer, reseteando buffer");
                recv_len = 0;
            }
            memcpy(recv_buffer + recv_len, rx_buffer, len);
            recv_len += len;
            recv_buffer[recv_len] = '\0';  // Añadir terminador nulo

            // Procesar líneas completas
            char *line_start = recv_buffer;
            char *newline_pos;
            while ((newline_pos = strchr(line_start, '\n')) != NULL) {
                *newline_pos = '\0';  // Reemplazar el carácter de nueva línea por terminador nulo
                char *command = line_start;

                // Eliminar retorno de carro si está presente
                int command_len = strlen(command);
                if (command_len > 0 && command[command_len - 1] == '\r') {
                    command[command_len - 1] = '\0';
                }

                ESP_LOGI(TAG, "Comando recibido: %s", command);

                // Procesar comando
                if (strcmp(command, "DATA") == 0) {
                    monitoreo_activo = true;
                    ESP_LOGI(TAG, "Monitoreo iniciado");
                } else if (strcmp(command, "STOP") == 0) {
                    monitoreo_activo = false;
                    ESP_LOGI(TAG, "Monitoreo detenido");
                } else if (strncmp(command, "SET_FREQ ", 9) == 0) {
                    int freq = atoi(command + 9);
                    if (freq > 0) {
                        frecuencia_muestreo = freq;
                        ESP_LOGI(TAG, "Frecuencia de muestreo establecida en %d ms", frecuencia_muestreo);
                    }
                } else if (strcmp(command, "MODE1") == 0) {
                    modo_operacion = 1;
                    ESP_LOGI(TAG, "Modo 1 (Datos en Bruto) activado");
                } else if (strcmp(command, "MODE2") == 0) {
                    modo_operacion = 2;
                    ESP_LOGI(TAG, "Modo 2 (Datos Procesados) activado");
                } else if (strncmp(command, "SET_WINDOW ", 11) == 0) {
                    int window = atoi(command + 11);
                    if (window > 0) {
                        ventana_tiempo = window;
                        ESP_LOGI(TAG, "Ventana de tiempo establecida en %d ms", ventana_tiempo);
                    }
                } else {
                    ESP_LOGW(TAG, "Comando desconocido: %s", command);
                }

                // Mover al inicio de la siguiente línea
                line_start = newline_pos + 1;
            }

            // Mover los datos restantes al inicio de recv_buffer
            int remaining_len = recv_len - (line_start - recv_buffer);
            if (remaining_len > 0) {
                memmove(recv_buffer, line_start, remaining_len);
            }
            recv_len = remaining_len;
            recv_buffer[recv_len] = '\0';  // Añadir terminador nulo
        }
    }

    ESP_LOGE(TAG, "Se cerró la conexión, reiniciando tarea de cliente TCP");
    close(sock);
    vTaskDelete(NULL);
}

void send_data_task(void *pvParameters) {
    while (1) {
        if (monitoreo_activo) {
            if (modo_operacion == 1) {
                // Leer datos del sensor
                float temperatura = leer_temperatura();
                float humedad = leer_humedad();
                // Enviar datos
                char data_buffer[128];
                int len = snprintf(data_buffer, sizeof(data_buffer), "DATA %.2f %.2f\n", temperatura, humedad);
                ESP_LOGI(TAG, "Enviando datos: %s", data_buffer);  // Mensaje de log
                int err = send(sock, data_buffer, len, 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error al enviar datos: errno %d", errno);
                }
                vTaskDelay(frecuencia_muestreo / portTICK_PERIOD_MS);
            } else if (modo_operacion == 2) {
                // Acumular datos durante ventana_tiempo
                int num_lecturas = ventana_tiempo / frecuencia_muestreo;
                float temp_total = 0, temp_max = -FLT_MAX, temp_min = FLT_MAX;
                float hum_total = 0, hum_max = -FLT_MAX, hum_min = FLT_MAX;

                for (int i = 0; i < num_lecturas; i++) {
                    float temperatura = leer_temperatura();
                    float humedad = leer_humedad();

                    temp_total += temperatura;
                    if (temperatura > temp_max) temp_max = temperatura;
                    if (temperatura < temp_min) temp_min = temperatura;

                    hum_total += humedad;
                    if (humedad > hum_max) hum_max = humedad;
                    if (humedad < hum_min) hum_min = humedad;

                    vTaskDelay(frecuencia_muestreo / portTICK_PERIOD_MS);
                }

                float temp_promedio = temp_total / num_lecturas;
                float hum_promedio = hum_total / num_lecturas;

                // Enviar datos procesados
                char data_buffer[256];
                int len = snprintf(data_buffer, sizeof(data_buffer),
                                   "STATS %.2f %.2f %.2f %.2f %.2f %.2f\n",
                                   temp_promedio, temp_max, temp_min,
                                   hum_promedio, hum_max, hum_min);
                ESP_LOGI(TAG, "Enviando datos: %s", data_buffer);  // Mensaje de log
                int err = send(sock, data_buffer, len, 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error al enviar datos: errno %d", errno);
                }
            }
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void) {
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Borrar la partición NVS y volver a intentar
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(sensor_init());
    bme_get_chipid();
    bme_softreset();

    wifi_init_sta();
    xTaskCreate(tcp_client_task, "tcp_client_task", 4096, NULL, 5, NULL);
}