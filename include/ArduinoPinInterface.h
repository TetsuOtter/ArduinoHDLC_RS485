#ifndef ARDUINO_PIN_INTERFACE_H
#define ARDUINO_PIN_INTERFACE_H

#include "IPinInterface.h"

#ifndef NATIVE_TEST
#include <Arduino.h>
#endif

/**
 * @brief Arduino環境でのピン操作実装
 * 
 * IPinInterfaceの実装クラス。
 * 実際のArduino関数を呼び出してピン操作を行う。
 */
class ArduinoPinInterface : public IPinInterface {
public:
    /**
     * @brief ピンモードの設定
     * @param pin ピン番号
     * @param mode ピンモード (INPUT, OUTPUT等)
     */
    void pinMode(uint8_t pin, uint8_t mode) override {
        ::pinMode(pin, mode);
    }

    /**
     * @brief デジタルピンへの書き込み
     * @param pin ピン番号
     * @param value 書き込む値 (HIGH/LOW)
     */
    void digitalWrite(uint8_t pin, uint8_t value) override {
        ::digitalWrite(pin, value);
    }

    /**
     * @brief デジタルピンからの読み取り
     * @param pin ピン番号
     * @return 読み取った値 (HIGH/LOW)
     */
    uint8_t digitalRead(uint8_t pin) override {
        return ::digitalRead(pin);
    }

    /**
     * @brief 割り込みの設定
     * @param interruptNum 割り込み番号
     * @param callback コールバック関数
     * @param mode 割り込みモード (CHANGE, RISING等)
     */
    void attachInterrupt(uint8_t interruptNum, void (*callback)(), uint8_t mode) override {
        ::attachInterrupt(interruptNum, callback, mode);
    }

    /**
     * @brief 割り込みの解除
     * @param interruptNum 割り込み番号
     */
    void detachInterrupt(uint8_t interruptNum) override {
        ::detachInterrupt(interruptNum);
    }

    /**
     * @brief マイクロ秒単位の遅延
     * @param microseconds 遅延時間（マイクロ秒）
     */
    void delayMicroseconds(uint32_t microseconds) override {
        ::delayMicroseconds(microseconds);
    }
};

#endif // ARDUINO_PIN_INTERFACE_H
