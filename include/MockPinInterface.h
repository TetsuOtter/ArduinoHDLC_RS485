#ifndef MOCK_PIN_INTERFACE_H
#define MOCK_PIN_INTERFACE_H

#include "IPinInterface.h"
#include <vector>
#include <functional>

// Arduino定数の定義（テスト環境用）
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef INPUT
#define INPUT 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef CHANGE
#define CHANGE 1
#endif

/**
 * @brief テスト用のモックピンインターフェース
 * 
 * 物理ピンを使わずにピン操作をシミュレートする。
 * テスト時の状態確認やコールバック呼び出しをサポート。
 */
class MockPinInterface : public IPinInterface {
public:
    // ピンの状態を記録する構造体
    struct PinState {
        uint8_t mode = 0;     // ピンモード
        uint8_t value = 0;    // ピンの値
    };

    // ログエントリの構造体
    struct LogEntry {
        enum Type { PIN_MODE, DIGITAL_WRITE, DIGITAL_READ, ATTACH_INTERRUPT, DETACH_INTERRUPT, DELAY_MICROS };
        Type type;
        uint8_t pin;
        uint8_t value;
        uint32_t timestamp;
    };

private:
    std::vector<PinState> m_pinStates;
    std::vector<LogEntry> m_log;
    std::function<void()> m_interruptCallback;
    uint8_t m_currentInterruptNum;
    uint32_t m_timeCounter;

public:
    MockPinInterface() : m_pinStates(256), m_currentInterruptNum(255), m_timeCounter(0) {}

    /**
     * @brief ピンモードの設定
     */
    void pinMode(uint8_t pin, uint8_t mode) override {
        if (pin < m_pinStates.size()) {
            m_pinStates[pin].mode = mode;
        }
        m_log.push_back({LogEntry::PIN_MODE, pin, mode, m_timeCounter++});
    }

    /**
     * @brief デジタルピンへの書き込み
     */
    void digitalWrite(uint8_t pin, uint8_t value) override {
        if (pin < m_pinStates.size()) {
            m_pinStates[pin].value = value;
        }
        m_log.push_back({LogEntry::DIGITAL_WRITE, pin, value, m_timeCounter++});
    }

    /**
     * @brief デジタルピンからの読み取り
     */
    uint8_t digitalRead(uint8_t pin) override {
        uint8_t value = 0;
        if (pin < m_pinStates.size()) {
            value = m_pinStates[pin].value;
        }
        m_log.push_back({LogEntry::DIGITAL_READ, pin, value, m_timeCounter++});
        return value;
    }

    /**
     * @brief 割り込みの設定
     */
    void attachInterrupt(uint8_t interruptNum, void (*callback)(), uint8_t mode) override {
        m_currentInterruptNum = interruptNum;
        m_interruptCallback = callback;
        m_log.push_back({LogEntry::ATTACH_INTERRUPT, interruptNum, mode, m_timeCounter++});
    }

    /**
     * @brief 割り込みの解除
     */
    void detachInterrupt(uint8_t interruptNum) override {
        if (m_currentInterruptNum == interruptNum) {
            m_interruptCallback = nullptr;
            m_currentInterruptNum = 255;
        }
        m_log.push_back({LogEntry::DETACH_INTERRUPT, interruptNum, 0, m_timeCounter++});
    }

    /**
     * @brief マイクロ秒単位の遅延
     */
    void delayMicroseconds(uint32_t microseconds) override {
        (void)microseconds; // 未使用パラメータ警告を回避
        m_log.push_back({LogEntry::DELAY_MICROS, 0, 0, m_timeCounter++});
        // 実際の遅延は行わない（テスト高速化のため）
    }

    // テスト用のユーティリティメソッド

    /**
     * @brief ピンの値を設定（外部からの信号をシミュレート）
     */
    void setPinValue(uint8_t pin, uint8_t value) {
        if (pin < m_pinStates.size()) {
            m_pinStates[pin].value = value;
        }
    }

    /**
     * @brief ピンの値を取得
     */
    uint8_t getPinValue(uint8_t pin) const {
        if (pin < m_pinStates.size()) {
            return m_pinStates[pin].value;
        }
        return 0;
    }

    /**
     * @brief ピンモードを取得
     */
    uint8_t getPinMode(uint8_t pin) const {
        if (pin < m_pinStates.size()) {
            return m_pinStates[pin].mode;
        }
        return 0;
    }

    /**
     * @brief 割り込みコールバックを手動で呼び出し
     */
    void triggerInterrupt() {
        if (m_interruptCallback) {
            m_interruptCallback();
        }
    }

    /**
     * @brief ログをクリア
     */
    void clearLog() {
        m_log.clear();
        m_timeCounter = 0;
    }

    /**
     * @brief ログエントリ数を取得
     */
    size_t getLogSize() const {
        return m_log.size();
    }

    /**
     * @brief ログエントリを取得
     */
    const LogEntry& getLogEntry(size_t index) const {
        return m_log[index];
    }

    /**
     * @brief 特定のピンへの書き込み回数をカウント
     */
    int countDigitalWrites(uint8_t pin, uint8_t value = 255) const {
        int count = 0;
        for (const auto& entry : m_log) {
            if (entry.type == LogEntry::DIGITAL_WRITE && entry.pin == pin) {
                if (value == 255 || entry.value == value) {
                    count++;
                }
            }
        }
        return count;
    }

    /**
     * @brief 遅延呼び出し回数をカウント
     */
    int countDelays() const {
        int count = 0;
        for (const auto& entry : m_log) {
            if (entry.type == LogEntry::DELAY_MICROS) {
                count++;
            }
        }
        return count;
    }
};

#endif // MOCK_PIN_INTERFACE_H
