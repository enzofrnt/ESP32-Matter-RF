#include "rf_security.h"
#include "esp_log.h"
#include "mbedtls/gcm.h"
#include "esp_random.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "RF_SECURITY";

static uint8_t g_aes_key[16];
static mbedtls_gcm_context g_gcm_ctx;
static bool g_initialized = false;

/**
 * @brief Convertit une chaîne hexadécimale en tableau de bytes
 */
static esp_err_t hex_string_to_bytes(const char *hex_str, uint8_t *bytes, size_t bytes_len)
{
    size_t hex_len = strlen(hex_str);
    if (hex_len != bytes_len * 2) {
        ESP_LOGE(TAG, "Invalid hex string length: expected %zu chars, got %zu",
                 bytes_len * 2, hex_len);
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < bytes_len; i++) {
        char hex_byte[3] = {hex_str[i*2], hex_str[i*2+1], '\0'};
        char *endptr;
        unsigned long val = strtoul(hex_byte, &endptr, 16);
        if (*endptr != '\0' || val > 255) {
            ESP_LOGE(TAG, "Invalid hex character at position %zu", i*2);
            return ESP_ERR_INVALID_ARG;
        }
        bytes[i] = (uint8_t)val;
    }
    return ESP_OK;
}

esp_err_t rf_security_init(void)
{
    if (g_initialized) {
        ESP_LOGW(TAG, "RF Security already initialized");
        return ESP_OK;
    }

    // Convertir la clé hex en bytes
    esp_err_t ret = hex_string_to_bytes(CONFIG_RF_AES_KEY, g_aes_key, 16);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse AES key from CONFIG_RF_AES_KEY");
        return ret;
    }

    // Initialiser le contexte GCM
    mbedtls_gcm_init(&g_gcm_ctx);
    
    // Configurer la clé AES-128
    ret = mbedtls_gcm_setkey(&g_gcm_ctx, MBEDTLS_CIPHER_ID_AES,
                            g_aes_key, 128);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set AES key: -0x%04x", -ret);
        mbedtls_gcm_free(&g_gcm_ctx);
        return ESP_FAIL;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "AES-128-GCM initialized successfully");
    
    // Log les 4 premiers bytes de la clé pour vérification (debug uniquement)
    ESP_LOGD(TAG, "AES Key (first 4 bytes): %02X %02X %02X %02X...",
             g_aes_key[0], g_aes_key[1], g_aes_key[2], g_aes_key[3]);
    
    return ESP_OK;
}

esp_err_t rf_security_encrypt(const uint8_t *plaintext, size_t plaintext_len,
                              uint8_t *ciphertext, size_t *ciphertext_len,
                              uint8_t *iv, uint8_t *tag)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "RF Security not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    if (plaintext == NULL || ciphertext == NULL || iv == NULL || tag == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for encryption");
        return ESP_ERR_INVALID_ARG;
    }

    // Générer IV aléatoire (12 bytes pour GCM)
    esp_fill_random(iv, 12);

    // Chiffrer avec GCM (pas d'AAD utilisé ici)
    int ret = mbedtls_gcm_crypt_and_tag(&g_gcm_ctx, MBEDTLS_GCM_ENCRYPT,
                                        plaintext_len,
                                        iv, 12,                    // IV de 12 bytes
                                        NULL, 0,                   // AAD (pas utilisé)
                                        plaintext,                 // Input
                                        ciphertext,                // Output
                                        16,                        // Tag length (16 bytes)
                                        tag);                      // Tag

    if (ret != 0) {
        ESP_LOGE(TAG, "AES-GCM encryption failed: -0x%04x", -ret);
        return ESP_FAIL;
    }

    *ciphertext_len = plaintext_len;
    return ESP_OK;
}

esp_err_t rf_security_decrypt(const uint8_t *ciphertext, size_t ciphertext_len,
                              const uint8_t *iv, const uint8_t *tag,
                              uint8_t *plaintext, size_t *plaintext_len)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "RF Security not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    if (ciphertext == NULL || plaintext == NULL || iv == NULL || tag == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for decryption");
        return ESP_ERR_INVALID_ARG;
    }

    // Déchiffrer et vérifier l'authentification avec GCM
    int ret = mbedtls_gcm_auth_decrypt(&g_gcm_ctx,
                                      ciphertext_len,
                                      iv, 12,                     // IV de 12 bytes
                                      NULL, 0,                    // AAD (pas utilisé)
                                      tag, 16,                    // Tag de 16 bytes
                                      ciphertext,                 // Input
                                      plaintext);                 // Output

    if (ret != 0) {
        ESP_LOGE(TAG, "AES-GCM decryption/authentication failed: -0x%04x", -ret);
        return ESP_FAIL;
    }

    *plaintext_len = ciphertext_len;
    return ESP_OK;
}

esp_err_t rf_security_generate_iv(uint8_t *iv, size_t iv_len)
{
    if (iv == NULL || iv_len < 12) {
        ESP_LOGE(TAG, "Invalid parameters for IV generation");
        return ESP_ERR_INVALID_ARG;
    }

    esp_fill_random(iv, iv_len);
    return ESP_OK;
}

