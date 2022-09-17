#pragma once

#include <stdint.h>

typedef enum {
    PIN_0_0 = 0x00,
    PIN_0_1 = 0x01,
    PIN_0_2 = 0x02,
    PIN_0_3 = 0x03,
    PIN_0_4 = 0x04,
    PIN_0_5 = 0x05,
    PIN_0_6 = 0x06,
    PIN_0_7 = 0x07,

    PIN_1_0 = 0x10,
    PIN_1_1 = 0x11,
    PIN_1_2 = 0x12,
    PIN_1_3 = 0x13,
    PIN_1_4 = 0x14,
    PIN_1_5 = 0x15,
    PIN_1_6 = 0x16,
    PIN_1_7 = 0x17,
} PinNumber;

typedef enum {
    INPUT_PIN = 1,
    OUTPUT_PIN = 0
} PinType;

class PCA9555 {
public:
    /**
     * @brief Construct a new PCA9555 object.
     *
     * @param A0 Address bit A0 determined by hardware connection.
     * @param A1 Address bit A1 determined by hardware connection.
     * @param A2 Address bit A2 determined by hardware connection.
     */
    PCA9555(uint8_t A0, uint8_t A1, uint8_t A2);

    /**
     * @brief Set pin type (input/output) of a single pin.
     *
     * @param pin The pin to be configured.
     * @param pinType Pin type to be configured.
     */
    void setPinType(PinNumber pin, PinType pinType);

    /**
     * @brief Set pin value for single pin.
     *
     * @param pin The pin to be set.
     * @param value logic value (high or low).
     */
    void setPin(PinNumber pin, uint8_t value);

    /**
     * @brief Toggle logical value of a specific pin.
     *
     * @param pin The pin to be configured.
     */
    void togglePin(PinNumber pin);

    uint8_t readPinValue(PinNumber pin);
private:
    uint8_t address;
    void writeRegister(uint8_t reg, uint8_t value);
    std::optional<uint8_t> readRegister(uint8_t reg);
};