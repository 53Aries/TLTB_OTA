#pragma once
#include <Arduino.h>
#include <esp_err.h>

// Call this when you're connected to Wi-Fi.
// Returns ESP_OK on success (it will reboot after flashing), else error code.
esp_err_t runGithubOta();
