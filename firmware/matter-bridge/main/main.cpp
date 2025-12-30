#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h" 

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "driver/gpio.h"

extern "C" {
#include <cc1101.h>
}

#include <esp_matter.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>

#include <app/clusters/on-off-server/on-off-server.h>
#include <app/server/Server.h>

static const char *TAG = "TX_MATTER";

typedef struct {
    uint8_t relay_idx_1based;
    bool state;
} rf_command_t;

static QueueHandle_t rf_command_queue = NULL;
static bool g_state[8] = {false};

// --- Utilitaires RF ---

static int rssi(char raw) {
    uint8_t rssi_dec = (uint8_t)raw;
    uint8_t rssi_offset = 74;
    if (rssi_dec >= 128) return ((int)(rssi_dec - 256) / 2) - rssi_offset;
    return (rssi_dec / 2) - rssi_offset;
}

static bool wait_for_ack(uint8_t relay_idx_1based, CCPACKET *out_pkt)
{
    // On passe explicitement en mode réception
    setRxState();
    
    // Stratégie : On vérifie 20 fois avec une pause de 10ms.
    // Total attente max = 200ms.
    // L'utilisation de vTaskDelay permet aux tâches WiFi/Matter de s'exécuter.
    const int max_retries = 20;

    for (int i = 0; i < max_retries; i++) {
        
        // On regarde si le CC1101 a reçu quelque chose dans son buffer
        if (packet_available()) {
            if (receiveData(out_pkt) > 0) {
                if (out_pkt->crc_ok) {
                    // Vérification du format du paquet
                    if (out_pkt->length >= 4 && out_pkt->data[0] == 'A') {
                        uint8_t got = (uint8_t)(out_pkt->data[1] - '0');
                        if (got == relay_idx_1based) {
                            return true; // Succès !
                        }
                    }
                }
            }
        }
        
        // CRUCIAL : On rend la main à l'OS. Le Watchdog est content.
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    
    return false; // Timeout
}

static void perform_rf_toggle(uint8_t relay_idx_1based, bool state)
{
    CCPACKET pkt_tx;
    CCPACKET pkt_rx;

    ESP_LOGI(TAG, "RF TASK: Sending toggle for Relay %d -> %d", relay_idx_1based, state);

    pkt_tx.data[0] = 'T';
    pkt_tx.data[1] = (uint8_t)('0' + relay_idx_1based);
    pkt_tx.length  = 2;

    // Envoi de la donnée.
    // Note : Si sendData plante ici, c'est un problème matériel (GDO0 mal branché)
    sendData(pkt_tx);

    // Attente de la réponse de manière non bloquante pour l'OS
    bool ok = wait_for_ack(relay_idx_1based, &pkt_rx);
    
    if (ok) {
        bool ack_state = (pkt_rx.data[3] == '1');
        ESP_LOGI(TAG, "RF TASK: ACK OK. Relay %u confirmed %d (RSSI:%d)", 
                 relay_idx_1based, (int)ack_state, rssi(pkt_rx.rssi));
    } else {
        ESP_LOGW(TAG, "RF TASK: NO ACK for relay %u (Timeout)", relay_idx_1based);
    }
}

// --- Tâche Worker RF ---
static void rf_worker_task(void *pvParameters)
{
    rf_command_t cmd;
    
    ESP_LOGI(TAG, "RF Worker Task Started");

    while (1) {
        // Attente bloquante d'un message dans la queue (consomme 0% CPU)
        if (xQueueReceive(rf_command_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            
            // Exécution de la commande RF
            perform_rf_toggle(cmd.relay_idx_1based, cmd.state);
            
            // Petit délai supplémentaire pour la stabilité
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
    }
}

// --- Callbacks Matter ---

static esp_err_t app_attribute_update_cb(esp_matter::attribute::callback_type_t type,
                                        uint16_t endpoint_id, uint32_t cluster_id,
                                        uint32_t attribute_id, esp_matter_attr_val_t *val,
                                        void *priv_data)
{
    if (type != esp_matter::attribute::PRE_UPDATE) {
        return ESP_OK;
    }

    if (cluster_id == chip::app::Clusters::OnOff::Id &&
        attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id) {

        int idx = (int)(intptr_t)priv_data; 
        if (idx >= 0 && idx < 8) {
            bool desired = val->val.b;
            
            // On envoie la commande seulement si l'état change
            // (Matter envoie parfois des mises à jour redondantes)
            if (g_state[idx] != desired) {
                rf_command_t cmd;
                cmd.relay_idx_1based = (uint8_t)(idx + 1);
                cmd.state = desired;

                // Envoi à la queue (timeout 0 pour ne jamais bloquer Matter)
                if (xQueueSend(rf_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGE(TAG, "RF Queue full! Dropping command");
                } else {
                    ESP_LOGI(TAG, "Queued RF command for Relay %d", cmd.relay_idx_1based);
                }
                g_state[idx] = desired;
            }
        }
    }
    return ESP_OK;
}

static void app_event_cb(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address Changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning Complete");
        break;
    default:
        break;
    }
}

// --- Bouton Factory Reset ---
static void factory_reset_button_task(void *pvParameters)
{
    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "Monitoring BOOT button for Factory Reset (Hold 5s)");

    while (1) {
        if (gpio_get_level(GPIO_NUM_0) == 0) {
            int hold_sec = 0;
            while (gpio_get_level(GPIO_NUM_0) == 0 && hold_sec < 50) {
                vTaskDelay(pdMS_TO_TICKS(100));
                hold_sec++;
            }
            if (hold_sec >= 50) {
                ESP_LOGE(TAG, "FACTORY RESET TRIGGERED!");
                esp_matter::factory_reset(); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t init_cc1101_from_kconfig(void)
{
    // Configuration identique
    uint8_t freq = CFREQ_433;
    uint8_t mode = CSPEED_38400;

#if defined(CONFIG_CC1101_FREQ_315)
    freq = CFREQ_315;
#elif defined(CONFIG_CC1101_FREQ_868)
    freq = CFREQ_868;
#endif

#if defined(CONFIG_CC1101_SPEED_4800)
    mode = CSPEED_4800;
#elif defined(CONFIG_CC1101_SPEED_9600)
    mode = CSPEED_9600;
#elif defined(CONFIG_CC1101_SPEED_19200)
    mode = CSPEED_19200;
#endif

    esp_err_t ret = init(freq, mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CC1101 init failed");
        return ESP_FAIL;
    }

    uint8_t syncWord[2] = {199, 10};
    setSyncWordArray(syncWord);
    int channel = 0;
#ifdef CONFIG_CC1101_CHANNEL
    channel = CONFIG_CC1101_CHANNEL;
#endif
    setChannel(channel);
    disableAddressCheck();
    setTxPowerAmp(POWER_0db);

    return ESP_OK;
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    rf_command_queue = xQueueCreate(10, sizeof(rf_command_t));

    if (init_cc1101_from_kconfig() == ESP_OK) {
        // Priorité 5 : Plus haute que Matter, mais grâce aux vTaskDelay,
        // on ne tue pas le système.
        xTaskCreate(rf_worker_task, "rf_worker", 4096, NULL, 5, NULL);
    }

    esp_matter::node::config_t node_config;
    esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, nullptr);
    
    for (int i = 0; i < 8; i++) {
        esp_matter::endpoint::on_off_light::config_t ep_cfg;
        ep_cfg.on_off.on_off = false;
        esp_matter::endpoint::on_off_light::create(node, &ep_cfg, esp_matter::ENDPOINT_FLAG_NONE, (void *)(intptr_t)i);
    }

    esp_matter::start(app_event_cb);
    xTaskCreate(factory_reset_button_task, "factory_reset", 3072, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "System Started");
}