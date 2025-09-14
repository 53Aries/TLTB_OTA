#pragma once
#include <Arduino.h>
#include <Wire.h>

// ------- Minimal INA226 register access (keeps HW ALERT with old libs) -------
static const uint8_t INA_ADDR_DEFAULT = 0x40;

// Registers (per TI datasheet)
enum {
  INA_REG_CONFIG      = 0x00,
  INA_REG_SHUNT_V     = 0x01,  // 2.5 uV/LSB (signed)
  INA_REG_BUS_V       = 0x02,  // 1.25 mV/LSB (unsigned)
  INA_REG_POWER       = 0x03,
  INA_REG_CURRENT     = 0x04,
  INA_REG_CALIB       = 0x05,
  INA_REG_MASK_ENABLE = 0x06,  // alert bits + latch
  INA_REG_ALERT_LIMIT = 0x07
};

inline void inaWrite16(uint8_t i2c_addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

inline uint16_t inaRead16(uint8_t i2c_addr, uint8_t reg) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)i2c_addr, 2);
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (hi << 8) | lo;
}

// ---- Calibrate for Current_LSB = 1 mA with Rshunt = 2.5 mΩ -> CAL = 2048
inline void inaCalibrate_1mA_2p5mR(uint8_t i2c_addr = INA_ADDR_DEFAULT) {
  inaWrite16(i2c_addr, INA_REG_CALIB, 2048);
}

// ---- Program SOL (shunt over-limit) + Latch (LEN), set AlertLimit (A)
inline void inaArmAlert_Latched(float limit_A,
                                float r_shunt_ohm = 0.0025f,
                                uint8_t i2c_addr  = INA_ADDR_DEFAULT) {
  // Alert limit counts = (Vshunt / 2.5uV)
  float v_shunt = limit_A * r_shunt_ohm;         // volts
  uint16_t counts = (uint16_t)roundf(v_shunt / 0.0000025f);
  inaWrite16(i2c_addr, INA_REG_ALERT_LIMIT, counts);

  // Enable SOL (bit15) + LEN (bit8)
  const uint16_t SOL = 1u << 15;
  const uint16_t LEN = 1u << 8;
  inaWrite16(i2c_addr, INA_REG_MASK_ENABLE, SOL | LEN);
}
