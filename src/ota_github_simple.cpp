#include "ota_github_simple.h"
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>   // Arduino's HTTP(S) updater

#ifndef OTA_LATEST_ASSET_URL
#define OTA_LATEST_ASSET_URL "https://github.com/53Aries/TLTB_OTA/releases/latest/download/firmware.bin"
#endif

// If you want strict TLS validation later, replace setInsecure() with setCACertBundle().
// For now (simplest), we skip cert verification since this isn’t security-critical.
esp_err_t runGithubOta() {
  WiFiClientSecure client;
  client.setInsecure();  // simplest: no CA verification (works out of the box)

  // If you decide to validate TLS later:
  // extern const unsigned char ca_bundle_start[] asm("_binary_x509_crt_bundle_start");
  // client.setCACertBundle(ca_bundle_start);

  HTTPUpdate updater;
  updater.rebootOnUpdate(true);                  // auto-reboot after flashing
  updater.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // needed for /latest/ URL
  updater.setLedPin(LED_BUILTIN, LOW);          // optional: blink during update

  Serial.println(F("[OTA] Checking GitHub latest…"));
  t_httpUpdate_return r = updater.update(client, OTA_LATEST_ASSET_URL);

  switch (r) {
    case HTTP_UPDATE_OK:
      // device will reboot; return OK anyway
      Serial.println(F("[OTA] Update OK (rebooting)…"));
      return ESP_OK;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F("[OTA] No update available."));
      return ESP_ERR_NOT_FOUND;
    case HTTP_UPDATE_FAILED:
    default:
      Serial.printf("[OTA] Update failed, err=%d (%s)\n",
                    updater.getLastError(), updater.getLastErrorString().c_str());
      return ESP_FAIL;
  }
}
