#include <Wire.h>
#include <Arduino.h>

#include "pca9555.h"

constexpr uint8_t inRegBase = 0b0;
constexpr uint8_t outRegBase = 0b10;
constexpr uint8_t outReg0 = 2; // 0b10
constexpr uint8_t outReg1 = 3; // 0b11
constexpr uint8_t configRegBase = 0b110;
constexpr uint8_t configReg0 = 6; // 0b110
constexpr uint8_t configReg1 = 7; // 0b111
constexpr uint8_t msgLength = 1;

PCA9555::PCA9555(uint8_t A0, uint8_t A1, uint8_t A2) {
    address = 0b0100000 | A0 | (A1 << 1) | (A2 << 2);
}

void PCA9555::setPinType(PinNumber pin, PinType pinType) {
    uint8_t configReg = configRegBase | ((pin & 0xF0) >> 4);
    std::optional<uint8_t> regValueOptional = readRegister(configReg);
    if (regValueOptional.has_value()) {
       uint8_t regValue = regValueOptional.value();
        if (pinType == PinType::OUTPUT_PIN) {
            regValue &= ~(1 << (pin & 0x0F));
        } else {
            regValue |= 1 << (pin & 0x0F);
        }
        writeRegister(configReg, regValue);
    }
}

void PCA9555::setPin(PinNumber pin, uint8_t value) {
    uint8_t pinReg = outRegBase | ((pin & 0xF0) >> 4);

    std::optional<uint8_t> regValueOptional = readRegister(pinReg);
    if (regValueOptional.has_value()) {
        uint8_t regValue = regValueOptional.value();
        Serial.println(regValue);
        if (value == LOW) {
            regValue &= ~(1 << (pin & 0x0F));
        } else {
            regValue |= 1 << (pin & 0x0F);
        }
        writeRegister(pinReg, regValue);
    }
    // TODO: At this moment if anything wrong with transmission, do nothing.
}

void PCA9555::togglePin(PinNumber pin) {
    uint8_t pinReg = outRegBase | ((pin & 0xF0) >> 4);

    std::optional<uint8_t> regValueOptional = readRegister(pinReg);
    if (regValueOptional.has_value()) {
        uint8_t regValue = regValueOptional.value();
        Serial.println(regValue);
        regValue ^= 1 << (pin & 0x0F);
        writeRegister(pinReg, regValue);
    }
}

uint8_t PCA9555::readPinValue(PinNumber pin) {
    uint8_t pinReg = inRegBase | ((pin & 0xF0) >> 4);
    std::optional<uint8_t> regValueOptional = readRegister(pinReg);
    if (regValueOptional.has_value()) {
        uint8_t regValue = regValueOptional.value();
        return (regValue >> (pin & 0x0F)) & 1;
    }

    // TODO: add better error handling when no value is read?
    return 0xFF;
}

void PCA9555::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

std::optional<uint8_t> PCA9555::readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    uint8_t status = Wire.endTransmission();

    if (status == 0) {
        Wire.requestFrom(address, msgLength);
        return Wire.read();
    }

    return std::nullopt;
}
