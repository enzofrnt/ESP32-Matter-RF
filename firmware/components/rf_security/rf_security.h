#ifndef RF_SECURITY_H
#define RF_SECURITY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/**
 * @brief Initialise le module de sécurité RF avec AES-128-GCM
 * 
 * @return ESP_OK si succès, ESP_FAIL sinon
 */
esp_err_t rf_security_init(void);

/**
 * @brief Chiffre un paquet avec AES-128-GCM
 * 
 * @param plaintext Données en clair
 * @param plaintext_len Longueur des données en clair
 * @param ciphertext Buffer pour les données chiffrées (doit être >= plaintext_len)
 * @param ciphertext_len Longueur des données chiffrées (sortie)
 * @param iv Buffer pour l'IV (12 bytes) - sera généré aléatoirement
 * @param tag Buffer pour le tag d'authentification (16 bytes) - sortie
 * @return ESP_OK si succès
 */
esp_err_t rf_security_encrypt(const uint8_t *plaintext, size_t plaintext_len,
                              uint8_t *ciphertext, size_t *ciphertext_len,
                              uint8_t *iv, uint8_t *tag);

/**
 * @brief Déchiffre un paquet avec AES-128-GCM
 * 
 * @param ciphertext Données chiffrées
 * @param ciphertext_len Longueur des données chiffrées
 * @param iv IV utilisé pour le chiffrement (12 bytes)
 * @param tag Tag d'authentification (16 bytes)
 * @param plaintext Buffer pour les données en clair (doit être >= ciphertext_len)
 * @param plaintext_len Longueur des données en clair (sortie)
 * @return ESP_OK si succès et authentification valide, ESP_FAIL sinon
 */
esp_err_t rf_security_decrypt(const uint8_t *ciphertext, size_t ciphertext_len,
                              const uint8_t *iv, const uint8_t *tag,
                              uint8_t *plaintext, size_t *plaintext_len);

/**
 * @brief Génère un IV aléatoire (12 bytes pour GCM)
 * 
 * @param iv Buffer pour l'IV (12 bytes minimum)
 * @param iv_len Longueur du buffer (doit être >= 12)
 * @return ESP_OK si succès
 */
esp_err_t rf_security_generate_iv(uint8_t *iv, size_t iv_len);

#endif // RF_SECURITY_H


