#ifndef PIN_INTERFACE_H
#define PIN_INTERFACE_H

#include <stdint.h>
#ifdef NATIVE_TEST
#include <cstddef>  // size_t用
#endif

/**
 * @brief ピン操作のインターフェース
 * 
 * RS485Driverクラスから物理ピン操作を分離するためのインターフェース。
 * テスト時はモック実装を注入することで単体テストを可能にする。
 */
class IPinInterface {
public:
    virtual ~IPinInterface() = default;

    /**
     * @brief ピンモードの設定
     * @param pin ピン番号
     * @param mode ピンモード (INPUT, OUTPUT等)
     */
    virtual void pinMode(uint8_t pin, uint8_t mode) = 0;

    /**
     * @brief デジタルピンへの書き込み
     * @param pin ピン番号
     * @param value 書き込む値 (HIGH/LOW)
     */
    virtual void digitalWrite(uint8_t pin, uint8_t value) = 0;

    /**
     * @brief デジタルピンからの読み取り
     * @param pin ピン番号
     * @return 読み取った値 (HIGH/LOW)
     */
    virtual uint8_t digitalRead(uint8_t pin) = 0;

    /**
     * @brief 割り込みの設定
     * @param interruptNum 割り込み番号
     * @param callback コールバック関数
     * @param mode 割り込みモード (CHANGE, RISING等)
     */
    virtual void attachInterrupt(uint8_t interruptNum, void (*callback)(), uint8_t mode) = 0;

    /**
     * @brief 割り込みの解除
     * @param interruptNum 割り込み番号
     */
    virtual void detachInterrupt(uint8_t interruptNum) = 0;

    /**
     * @brief マイクロ秒単位の遅延
     * @param microseconds 遅延時間（マイクロ秒）
     */
    virtual void delayMicroseconds(uint32_t microseconds) = 0;
};

#endif // PIN_INTERFACE_H
