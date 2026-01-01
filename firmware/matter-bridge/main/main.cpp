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
#include "rf_security.h"
}

#include <esp_matter.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_attribute.h>

#include <app/clusters/on-off-server/on-off-server.h>
#include <app/server/Server.h>

// --- AJOUT CRITIQUE POUR LE THREAD SAFETY ---
#include <platform/CHIPDeviceLayer.h>
// --------------------------------------------

static const char *TAG = "TX_MATTER";

typedef struct {
    uint8_t relay_idx_1based;
    bool state;
} rf_command_t;

static QueueHandle_t rf_command_queue = NULL;
static bool g_state[8] = {false};
static uint16_t g_endpoint_ids[8] = {0};
static uint8_t g_comm_failures[8] = {0}; // Compteur d'échecs consécutifs
static bool g_relay_offline[8] = {false}; // État de communication (true = offline)
static bool g_rf_security_initialized = false;
#define MAX_COMM_FAILURES 3 // Nombre d'échecs avant de marquer comme offline

// --- Utilitaires RF ---

static int rssi(char raw) {
    uint8_t rssi_dec = (uint8_t)raw;
    uint8_t rssi_offset = 74;
    if (rssi_dec >= 128) return ((int)(rssi_dec - 256) / 2) - rssi_offset;
    return (rssi_dec / 2) - rssi_offset;
}

static bool wait_for_ack(uint8_t relay_idx_1based, CCPACKET *out_pkt)
{
    setRxState();
    const int max_retries = 15; // Réduit de 20 à 15 pour réduire le timeout max (150ms au lieu de 200ms)

    for (int i = 0; i < max_retries; i++) {
        // Réinitialiser le watchdog à CHAQUE itération pour éviter les timeouts
        esp_task_wdt_reset();
        taskYIELD(); // Forcer le changement de contexte à chaque itération
        
        if (packet_available()) {
            if (receiveData(out_pkt) > 0) {
                if (out_pkt->crc_ok) {
                    // Vérifier la taille minimale : IV (12) + Tag (16) + min data (4)
                    if (out_pkt->length < 12 + 16 + 4) {
                        continue;
                    }
                    
                    // Extraire IV, Tag et Ciphertext
                    uint8_t *iv = out_pkt->data;
                    uint8_t *tag = out_pkt->data + 12;
                    uint8_t *ciphertext = out_pkt->data + 12 + 16;
                    size_t ciphertext_len = out_pkt->length - 12 - 16;
                    
                    // Déchiffrer dans un buffer temporaire
                    uint8_t plaintext[64];
                    size_t plaintext_len;
                    
                    esp_err_t ret = rf_security_decrypt(ciphertext, ciphertext_len,
                                                       iv, tag, plaintext, &plaintext_len);
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "RF TASK: Decryption failed or invalid auth!");
                        continue;
                    }
                    
                    // Copier les données déchiffrées dans out_pkt pour compatibilité
                    // (les fonctions appelantes s'attendent à trouver les données en clair)
                    if (plaintext_len <= sizeof(out_pkt->data)) {
                        memcpy(out_pkt->data, plaintext, plaintext_len);
                        out_pkt->length = plaintext_len;
                    }
                    
                    // Vérifier le contenu déchiffré
                    if (plaintext_len >= 4 && plaintext[0] == 'A') {
                        uint8_t got = (uint8_t)(plaintext[1] - '0');
                        if (got == relay_idx_1based) {
                            esp_task_wdt_reset(); // Réinitialiser avant de retourner
                            return true;
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    esp_task_wdt_reset(); // Réinitialiser avant de retourner false
    return false;
}

static void perform_rf_set_state(uint8_t relay_idx_1based, bool state)
{
    int idx = (int)relay_idx_1based - 1;
    
    // Vérifier si le relais est marqué comme offline
    if (g_relay_offline[idx]) {
        ESP_LOGE(TAG, "RF TASK: Relay %u is OFFLINE - command blocked!", relay_idx_1based);
        return;
    }
    
    CCPACKET pkt_tx;
    CCPACKET pkt_rx;

    ESP_LOGI(TAG, "RF TASK: Sending set state for Relay %d -> %d", relay_idx_1based, state);

    // Préparer le paquet en clair
    uint8_t plaintext[3];
    plaintext[0] = 'S';
    plaintext[1] = (uint8_t)('0' + relay_idx_1based);
    plaintext[2] = (uint8_t)(state ? '1' : '0');

    // Chiffrer le paquet avec AES-128-GCM
    uint8_t ciphertext[16];  // Taille max pour 3 bytes de plaintext
    uint8_t iv[12];
    uint8_t tag[16];
    size_t ciphertext_len;

    esp_task_wdt_reset();
    esp_err_t ret = rf_security_encrypt(plaintext, 3, ciphertext, &ciphertext_len, iv, tag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RF TASK: Encryption failed!");
        return;
    }

    // Construire le paquet final : IV (12) + Tag (16) + Ciphertext (3)
    // Format: [IV:12 bytes][Tag:16 bytes][Ciphertext:variable]
    memcpy(pkt_tx.data, iv, 12);
    memcpy(pkt_tx.data + 12, tag, 16);
    memcpy(pkt_tx.data + 12 + 16, ciphertext, ciphertext_len);
    pkt_tx.length = 12 + 16 + ciphertext_len;  // Total: 31 bytes

    // Réinitialiser le watchdog avant l'opération RF
    esp_task_wdt_reset();
    sendData(pkt_tx);
    // Réinitialiser immédiatement après sendData (peut bloquer)
    esp_task_wdt_reset();
    taskYIELD(); // Forcer le changement de contexte après sendData

    bool ok = wait_for_ack(relay_idx_1based, &pkt_rx);
    
    // Réinitialiser après wait_for_ack
    esp_task_wdt_reset();
    
    if (ok) {
        // Le paquet ACK est déjà déchiffré dans wait_for_ack
        // pkt_rx.data contient maintenant les données en clair
        bool ack_state = (pkt_rx.data[3] == '1');
        ESP_LOGI(TAG, "RF TASK: ACK OK. Relay %u confirmed state %d", 
                 relay_idx_1based, (int)ack_state);
        // Communication réussie : réinitialiser le compteur d'échecs
        g_comm_failures[idx] = 0;
        g_relay_offline[idx] = false;
        
        // Vérification que l'état confirmé correspond à l'état demandé
        if (ack_state != state) {
            ESP_LOGW(TAG, "RF TASK: State mismatch! Requested %d but got %d", 
                     (int)state, (int)ack_state);
        }
    } else {
        ESP_LOGW(TAG, "RF TASK: NO ACK for relay %u (Timeout)", relay_idx_1based);
        g_comm_failures[idx]++;
        
        // Marquer comme offline après plusieurs échecs
        if (g_comm_failures[idx] >= MAX_COMM_FAILURES) {
            g_relay_offline[idx] = true;
            ESP_LOGE(TAG, "RF TASK: Relay %u marked as OFFLINE after %d failures!", 
                     relay_idx_1based, g_comm_failures[idx]);
        }
    }
}

// Fonction pour corriger directement l'état d'un relais (utilisée par sync_status_task)
// Retourne true si la commande a réussi, false sinon
static bool sync_correct_relay_state(uint8_t relay_idx_1based, bool target_state)
{
    int idx = (int)relay_idx_1based - 1;
    CCPACKET pkt_tx;
    CCPACKET pkt_rx;

    ESP_LOGI(TAG, "SYNC: Correcting relay %d to state %d", relay_idx_1based, (int)target_state);

    // Préparer le paquet en clair
    uint8_t plaintext[3];
    plaintext[0] = 'S';
    plaintext[1] = (uint8_t)('0' + relay_idx_1based);
    plaintext[2] = (uint8_t)(target_state ? '1' : '0');

    // Chiffrer
    uint8_t ciphertext[16];
    uint8_t iv[12];
    uint8_t tag[16];
    size_t ciphertext_len;

    esp_task_wdt_reset();
    esp_err_t ret = rf_security_encrypt(plaintext, 3, ciphertext, &ciphertext_len, iv, tag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SYNC: Encryption failed!");
        return false;
    }

    // Construire le paquet
    memcpy(pkt_tx.data, iv, 12);
    memcpy(pkt_tx.data + 12, tag, 16);
    memcpy(pkt_tx.data + 12 + 16, ciphertext, ciphertext_len);
    pkt_tx.length = 12 + 16 + ciphertext_len;

    // Réinitialiser le watchdog avant l'opération RF
    esp_task_wdt_reset();
    sendData(pkt_tx);
    // Réinitialiser immédiatement après sendData (peut bloquer)
    esp_task_wdt_reset();
    taskYIELD(); // Forcer le changement de contexte après sendData

    bool ok = wait_for_ack(relay_idx_1based, &pkt_rx);
    
    // Réinitialiser après wait_for_ack
    esp_task_wdt_reset();
    
    if (ok) {
        bool ack_state = (pkt_rx.data[3] == '1');
        ESP_LOGI(TAG, "SYNC: Correction successful! Relay %u confirmed state %d", 
                 relay_idx_1based, (int)ack_state);
        // Communication réussie : réinitialiser le compteur d'échecs
        g_comm_failures[idx] = 0;
        g_relay_offline[idx] = false;
        g_state[idx] = ack_state;
        
        // Vérifier que l'état confirmé correspond à l'état demandé
        if (ack_state != target_state) {
            ESP_LOGW(TAG, "SYNC: Warning! Requested state %d but relay confirmed %d", 
                     (int)target_state, (int)ack_state);
        }
        
        return true;
    } else {
        ESP_LOGW(TAG, "SYNC: Correction FAILED for relay %u (Timeout)", relay_idx_1based);
        g_comm_failures[idx]++;
        
        // Marquer comme offline après plusieurs échecs
        if (g_comm_failures[idx] >= MAX_COMM_FAILURES) {
            if (!g_relay_offline[idx]) {
                g_relay_offline[idx] = true;
                ESP_LOGE(TAG, "SYNC: Relay %u marked as OFFLINE after %d failures!", 
                         relay_idx_1based, g_comm_failures[idx]);
            }
        }
        return false;
    }
}

static bool query_rf_relay_state(uint8_t relay_idx_1based, bool *out_state)
{
    int idx = (int)relay_idx_1based - 1;
    CCPACKET pkt_tx;
    CCPACKET pkt_rx;

    ESP_LOGI(TAG, "SYNC: Querying state for Relay %d", relay_idx_1based);

    // Préparer le paquet en clair
    uint8_t plaintext[2];
    plaintext[0] = 'Q';
    plaintext[1] = (uint8_t)('0' + relay_idx_1based);

    // Chiffrer
    uint8_t ciphertext[16];
    uint8_t iv[12];
    uint8_t tag[16];
    size_t ciphertext_len;

    esp_task_wdt_reset();
    esp_err_t ret = rf_security_encrypt(plaintext, 2, ciphertext, &ciphertext_len, iv, tag);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SYNC: Encryption failed!");
        return false;
    }

    // Construire le paquet
    memcpy(pkt_tx.data, iv, 12);
    memcpy(pkt_tx.data + 12, tag, 16);
    memcpy(pkt_tx.data + 12 + 16, ciphertext, ciphertext_len);
    pkt_tx.length = 12 + 16 + ciphertext_len;  // Total: 30 bytes

    // Réinitialiser le watchdog avant l'opération RF
    esp_task_wdt_reset();
    sendData(pkt_tx);
    // Réinitialiser immédiatement après sendData (peut bloquer)
    esp_task_wdt_reset();
    taskYIELD(); // Forcer le changement de contexte après sendData

    bool ok = wait_for_ack(relay_idx_1based, &pkt_rx);
    
    // Réinitialiser après wait_for_ack
    esp_task_wdt_reset();
    
    if (ok) {
        // Le paquet est déjà déchiffré dans wait_for_ack
        bool relay_state = (pkt_rx.data[3] == '1');
        *out_state = relay_state;
        ESP_LOGI(TAG, "SYNC: Relay %u state = %d", 
                 relay_idx_1based, (int)relay_state);
        // Communication réussie : réinitialiser le compteur d'échecs
        g_comm_failures[idx] = 0;
        g_relay_offline[idx] = false;
        return true;
    } else {
        ESP_LOGW(TAG, "SYNC: NO ACK for relay %u query (Timeout)", relay_idx_1based);
        g_comm_failures[idx]++;
        
        // Marquer comme offline après plusieurs échecs
        if (g_comm_failures[idx] >= MAX_COMM_FAILURES) {
            if (!g_relay_offline[idx]) {
                g_relay_offline[idx] = true;
                ESP_LOGE(TAG, "SYNC: Relay %u marked as OFFLINE after %d consecutive failures!", 
                         relay_idx_1based, g_comm_failures[idx]);
            }
        }
        return false;
    }
}

// --- Mise à jour de l'état Matter avec remontée vers Home Assistant ---
// Cette fonction suit les bonnes pratiques Matter :
// 1. Vérifie si l'état a vraiment changé avant de mettre à jour (évite les boucles)
// 2. Met à jour g_state AVANT d'appeler attribute::update
// 3. Utilise le verrouillage Matter Stack pour thread safety
// 4. Home Assistant reçoit automatiquement la notification via subscription Matter
static void update_matter_state(uint8_t relay_idx_0based, bool state)
{
    if (relay_idx_0based >= 8) return;

    uint16_t endpoint_id = g_endpoint_ids[relay_idx_0based];
    if (endpoint_id == 0) {
        ESP_LOGW(TAG, "SYNC: Endpoint ID not set for relay %d", relay_idx_0based + 1);
        return;
    }

    // BONNE PRATIQUE : Vérifier si l'état a vraiment changé avant de mettre à jour
    // Cela évite d'envoyer des notifications inutiles et des boucles infinies
    if (g_state[relay_idx_0based] == state) {
        // État déjà à jour, pas besoin de mettre à jour Matter
        return;
    }

    // Mise à jour de la copie locale AVANT d'appeler attribute::update
    // Cela garantit que le callback PRE_UPDATE ne déclenchera pas de commande RF
    // car g_state[idx] == desired sera vrai
    g_state[relay_idx_0based] = state;

    // Préparer la valeur de l'attribut
    esp_matter_attr_val_t val;
    memset(&val, 0, sizeof(esp_matter_attr_val_t));
    val.type = 0x10; // ZCL Boolean (type standard pour les attributs booléens Matter)
    val.val.b = state;
    
    // --- VERROUILLAGE DU STACK MATTER (Thread Safety) ---
    // CRITIQUE : Toute interaction avec Matter doit se faire depuis le thread Matter
    // Cela garantit que Home Assistant reçoit la notification correctement
    // PlatformMgr().LockChipStack() est équivalent à ScopedChipStackLock mais manuel
    chip::DeviceLayer::PlatformMgr().LockChipStack();
    
    // Mise à jour de l'attribut OnOff dans Matter
    // Cela déclenche automatiquement une notification (report) vers Home Assistant
    esp_err_t err = esp_matter::attribute::update(
        endpoint_id,
        chip::app::Clusters::OnOff::Id,
        chip::app::Clusters::OnOff::Attributes::OnOff::Id,
        &val
    );
    
    // Libération du verrou : Matter se réveille et envoie la notification
    chip::DeviceLayer::PlatformMgr().UnlockChipStack();
    // ------------------------------------

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SYNC: Updated Matter state for relay %d (endpoint %d) -> %d (Home Assistant will be notified)", 
                 relay_idx_0based + 1, endpoint_id, (int)state);
    } else {
        ESP_LOGE(TAG, "SYNC: Failed to update Matter state: %s", esp_err_to_name(err));
        // En cas d'erreur, on pourrait restaurer g_state, mais généralement on laisse tel quel
    }
}

// --- Tâche Worker RF ---
static void rf_worker_task(void *pvParameters)
{
    rf_command_t cmd;
    ESP_LOGI(TAG, "RF Worker Task Started");

    // S'abonner au watchdog pour permettre l'appel de perform_rf_set_state qui contient des resets
    esp_task_wdt_add(NULL);

    while (1) {
        // Reset du watchdog à chaque tour de boucle
        esp_task_wdt_reset();

        // On attend une commande. Note: xQueueReceive avec portMAX_DELAY peut bloquer indéfiniment.
        // Pour ne pas déclencher le watchdog pendant l'attente, on utilise un timeout
        // et on boucle pour reset le watchdog.
        if (xQueueReceive(rf_command_queue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Commande reçue, on l'exécute (les resets internes à perform_rf_set_state fonctionneront)
            perform_rf_set_state(cmd.relay_idx_1based, cmd.state);
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
    // On ne s'intéresse qu'à la phase PRE_UPDATE pour intercepter
    if (type != esp_matter::attribute::PRE_UPDATE) {
        return ESP_OK;
    }

    if (cluster_id == chip::app::Clusters::OnOff::Id &&
        attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id) {

        int idx = (int)(intptr_t)priv_data; 
        if (idx >= 0 && idx < 8) {
            bool desired = val->val.b;
            
            // Vérifier si le relais est offline - bloquer la commande
            if (g_relay_offline[idx]) {
                ESP_LOGE(TAG, "COMM ERROR: Relay %d is OFFLINE - command rejected! Communication failed.", idx + 1);
                // Rejeter la commande en restaurant l'ancienne valeur
                val->val.b = g_state[idx];
                return ESP_ERR_INVALID_STATE;
            }
            
            // Cette condition est CRUCIALE pour éviter la boucle infinie
            // Quand update_matter_state() est appelé, g_state est déjà mis à jour,
            // donc on n'entre PAS ici.
            if (g_state[idx] != desired) {
                rf_command_t cmd;
                cmd.relay_idx_1based = (uint8_t)(idx + 1);
                cmd.state = desired;

                if (xQueueSend(rf_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGE(TAG, "RF Queue full! Dropping command");
                } else {
                    ESP_LOGI(TAG, "Queued RF command for Relay %d", cmd.relay_idx_1based);
                }
                
                // On met à jour l'état local immédiatement
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

// --- Tâche de synchronisation périodique ---
// --- Tâche de synchronisation périodique ---
static void sync_status_task(void *pvParameters)
{
    int sync_interval_sec = 30;
    ESP_LOGI(TAG, "Sync Status Task Started (interval: %d seconds)", sync_interval_sec);

    // 1. D'ABORD on attend que le système se stabilise (Wi-Fi, Matter, etc.)
    // On ne s'enregistre PAS encore au watchdog ici, car on va dormir 5s.
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 2. MAINTENANT on s'abonne au watchdog, juste avant d'entrer dans la boucle de travail
    esp_task_wdt_add(NULL);

    while (1) {
        // Réinitialiser le watchdog au début de chaque cycle
        esp_task_wdt_reset();
        
        for (int i = 0; i < 8; i++) {
            uint8_t relay_idx_1based = (uint8_t)(i + 1);
            bool actual_state;

            // Réinitialiser le watchdog avant chaque opération RF
            esp_task_wdt_reset();
            
            // On vérifie l'état réel via RF
            if (query_rf_relay_state(relay_idx_1based, &actual_state)) {
                
                // Gestion du retour en ligne
                if (g_relay_offline[i]) {
                    ESP_LOGI(TAG, "SYNC: Relay %d is back ONLINE!", relay_idx_1based);
                    g_relay_offline[i] = false;
                }
                
                // Si différence avec Matter
                if (g_state[i] != actual_state) {
                    ESP_LOGW(TAG, "SYNC: Mismatch detected! Relay %d: Matter=%d, Relay=%d. Attempting correction...", 
                             relay_idx_1based, (int)g_state[i], (int)actual_state);
                    
                    esp_task_wdt_reset();
                    
                    // Tentative de correction
                    bool correction_success = sync_correct_relay_state(relay_idx_1based, g_state[i]);
                    
                    esp_task_wdt_reset();
                    
                    if (correction_success) {
                        ESP_LOGI(TAG, "SYNC: Relay %d successfully corrected", relay_idx_1based);
                    } else {
                        ESP_LOGE(TAG, "SYNC: Correction failed for relay %d. Updating Matter UI instead.", relay_idx_1based);
                        update_matter_state(i, actual_state);
                    }
                }
            }
            
            // Petit délai entre les relais pour laisser respirer le CPU et le RF
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Attente longue : On doit gérer le watchdog ici aussi si l'intervalle > 5s
        // Le vTaskDelay bloque la tâche, donc on ne peut pas reset pendant ce temps.
        // Solution : Soit on se désabonne temporairement, soit on boucle.
        // Voici la méthode propre "boucle" pour garder la surveillance :
        
        int elapsed = 0;
        while (elapsed < sync_interval_sec * 1000) {
            vTaskDelay(pdMS_TO_TICKS(1000)); // On dort par tranches de 1s
            esp_task_wdt_reset();           // On nourrit le chien
            elapsed += 1000;
        }
    }
}

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
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    rf_command_queue = xQueueCreate(10, sizeof(rf_command_t));

    bool cc1101_initialized = (init_cc1101_from_kconfig() == ESP_OK);
    if (cc1101_initialized) {
        // Initialiser la sécurité RF
        if (rf_security_init() != ESP_OK) {
            ESP_LOGE(TAG, "RF Security initialization failed! Communication will fail.");
        } else {
            g_rf_security_initialized = true;
            ESP_LOGI(TAG, "RF Security (AES-128-GCM) initialized");
        }
        
        xTaskCreate(rf_worker_task, "rf_worker", 4096, NULL, 5, NULL);
    }

    esp_matter::node::config_t node_config;
    esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, nullptr);
    if (node == nullptr) {
        ESP_LOGE(TAG, "Failed to create Matter node!");
        return;
    }
    
    for (int i = 0; i < 8; i++) {
        esp_matter::endpoint::on_off_light::config_t ep_cfg;
        ep_cfg.on_off.on_off = false;
        esp_matter::endpoint_t *endpoint = esp_matter::endpoint::on_off_light::create(node, &ep_cfg, esp_matter::ENDPOINT_FLAG_NONE, (void *)(intptr_t)i);
        if (endpoint != nullptr) {
            g_endpoint_ids[i] = esp_matter::endpoint::get_id(endpoint);
            ESP_LOGI(TAG, "Created endpoint %d for relay %d", g_endpoint_ids[i], i + 1);
        }
    }

    // Vérifier le retour de esp_matter::start()
    ret = esp_matter::start(app_event_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %s (0x%x)", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG, "This might be due to corrupted NVS. Try factory reset or erase NVS partition.");
        // Le système peut continuer en mode RF-only mais Matter ne fonctionnera pas
        ESP_LOGW(TAG, "Continuing in RF-only mode (Matter disabled)");
    } else {
        ESP_LOGI(TAG, "Matter started successfully");
    }
    
    xTaskCreate(factory_reset_button_task, "factory_reset", 3072, NULL, 2, NULL);
    
    if (cc1101_initialized) {
        xTaskCreate(sync_status_task, "sync_status", 4096, NULL, 3, NULL);
    }
    
    ESP_LOGI(TAG, "System Started");
}