#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"

#include <cc1101.h>

static const char *TAG = "RX";

// Mets 0 si tes relais sont actifs à l'état bas (très courant sur les boards relais)
#define RELAY_ON_LEVEL   1
#define RELAY_OFF_LEVEL  (1 - RELAY_ON_LEVEL)

static const gpio_num_t relay_pins[8] = {
    GPIO_NUM_13, // 1
    GPIO_NUM_12, // 2
    GPIO_NUM_14, // 3
    GPIO_NUM_27, // 4
    GPIO_NUM_26, // 5
    GPIO_NUM_25, // 6
    GPIO_NUM_33, // 7
    GPIO_NUM_32  // 8
};

static uint8_t relay_state[8] = {0}; // 0=off, 1=on


#define NVS_NAMESPACE "relay_states"
#define NVS_KEY "states"

static esp_err_t save_relay_states(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, NVS_KEY, relay_state, sizeof(relay_state));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving relay states: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Relay states saved to NVS");
    }

    nvs_close(nvs_handle);
    return err;
}

static esp_err_t restore_relay_states(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace not found or error opening: %s", esp_err_to_name(err));
        return err;
    }

    size_t required_size = sizeof(relay_state);
    err = nvs_get_blob(nvs_handle, NVS_KEY, relay_state, &required_size);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No saved relay states found, using defaults");
        } else {
            ESP_LOGE(TAG, "Error reading relay states: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
        return err;
    }

    ESP_LOGI(TAG, "Relay states restored from NVS");
    nvs_close(nvs_handle);
    return ESP_OK;
}

int rssi(char raw) {
    uint8_t rssi_dec;
    uint8_t rssi_offset = 74;
    rssi_dec = (uint8_t)raw;
    if (rssi_dec >= 128) return ((int)(rssi_dec - 256) / 2) - rssi_offset;
    return (rssi_dec / 2) - rssi_offset;
}

int lqi(char raw) {
    return 0x3F - raw;
}

static void relay_init(void)
{
    gpio_config_t io = {0};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;

    uint64_t mask = 0;
    for (int i = 0; i < 8; i++) {
        mask |= (1ULL << relay_pins[i]);
    }
    io.pin_bit_mask = mask;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io);

    // Restaurer les états depuis NVS
    restore_relay_states();

    // Appliquer les états restaurés (ou OFF par défaut si pas de sauvegarde)
    for (int i = 0; i < 8; i++) {
        gpio_set_level(relay_pins[i], relay_state[i] ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
        ESP_LOGI(TAG, "Relay %d initialized to state %u", i + 1, relay_state[i]);
    }
}

static void send_ack(uint8_t relay_idx_1based, uint8_t state)
{
    CCPACKET ack;
    ack.data[0] = 'A';
    ack.data[1] = (uint8_t)('0' + relay_idx_1based);
    ack.data[2] = ':';
    ack.data[3] = (uint8_t)(state ? '1' : '0');
    ack.length = 4;
    sendData(ack);
}


static void rx_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Start RX task (waiting for set state commands)");
    CCPACKET pkt;

    while (1) {
        if (packet_available()) {
            if (receiveData(&pkt) > 0) {
                if (!pkt.crc_ok) {
                    ESP_LOGE(TAG, "crc not ok");
                    continue;
                }

                ESP_LOGI(TAG, "RX packet len=%d rssi=%ddBm lqi=%d",
                         pkt.length, rssi(pkt.rssi), lqi(pkt.lqi));
                

                // Expect "S<digit><0|1>" - Set state command
                if (pkt.length >= 3 && pkt.data[0] == 'S' && 
                    pkt.data[1] >= '1' && pkt.data[1] <= '8' &&
                    (pkt.data[2] == '0' || pkt.data[2] == '1')) {
                    uint8_t relay = (uint8_t)(pkt.data[1] - '0'); // 1..8
                    uint8_t target_state = (uint8_t)(pkt.data[2] - '0'); // 0 ou 1
                    int idx = (int)relay - 1;

                    // Mise à jour directe de l'état (pas de toggle)
                    relay_state[idx] = target_state;
                    gpio_set_level(relay_pins[idx], relay_state[idx] ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);

                    ESP_LOGI(TAG, "Set relay %u -> %u", relay, relay_state[idx]);

                    // Sauvegarder l'état dans NVS
                    save_relay_states();

                    // Envoi de l'ACK avec l'état confirmé
                    send_ack(relay, relay_state[idx]);
                }
                // Expect "Q<digit>" - Query state command
                else if (pkt.length >= 2 && pkt.data[0] == 'Q' && 
                         pkt.data[1] >= '1' && pkt.data[1] <= '8') {
                    uint8_t relay = (uint8_t)(pkt.data[1] - '0'); // 1..8
                    int idx = (int)relay - 1;

                    // Réponse avec l'état actuel du relais
                    ESP_LOGI(TAG, "Query relay %u -> current state %u", relay, relay_state[idx]);
                    send_ack(relay, relay_state[idx]);
                } else {
                    ESP_LOGW(TAG, "Unknown command: len=%d, data[0]=0x%02x('%c')", 
                             pkt.length, 
                             pkt.data[0],
                             (pkt.data[0] >= 32 && pkt.data[0] < 127) ? pkt.data[0] : '?');
                }
            }
        }
        vTaskDelay(1);
    }

    vTaskDelete(NULL);
}


void app_main(void)
{
    // Initialiser NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    relay_init();


    uint8_t freq;
#if CONFIG_CC1101_FREQ_315
    freq = CFREQ_315;
#elif CONFIG_CC1101_FREQ_433
    freq = CFREQ_433;
#elif CONFIG_CC1101_FREQ_868
    freq = CFREQ_868;
#elif CONFIG_CC1101_FREQ_915
    freq = CFREQ_915;
#endif

    uint8_t mode;
#if CONFIG_CC1101_SPEED_4800
    mode = CSPEED_4800;
#elif CONFIG_CC1101_SPEED_9600
    mode = CSPEED_9600;
#elif CONFIG_CC1101_SPEED_19200
    mode = CSPEED_19200;
#elif CONFIG_CC1101_SPEED_38400
    mode = CSPEED_38400;
#endif

    ret = init(freq, mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CC1101 not installed");
        while (1) vTaskDelay(1);
    }

    uint8_t syncWord[2] = {199, 10};
    setSyncWordArray(syncWord);
    setChannel(CONFIG_CC1101_CHANNEL);
    disableAddressCheck();

#if CONFIG_CC1101_POWER_MIN
    setTxPowerAmp(POWER_MIN);
#elif CONFIG_CC1101_POWER_0db
    setTxPowerAmp(POWER_0db);
#elif CONFIG_CC1101_POWER_MAX
    setTxPowerAmp(POWER_MAX);
#endif

    xTaskCreate(&rx_task, "RX", 4096, NULL, 5, NULL);
    
}