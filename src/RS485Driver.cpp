#include "RS485Driver.h"

#ifdef NATIVE_TEST
// テスト環境用のArduino定数定義
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT
#define INPUT 0
#endif
#ifndef CHANGE
#define CHANGE 1
#endif
#endif

// 静的メンバは不要になったので削除

RS485Driver::RS485Driver(IPinInterface &pinInterface, uint8_t txPin, uint8_t rxPin, uint8_t dePin, uint8_t rePin, uint32_t baudRate)
    : m_pinInterface(pinInterface), m_txPin(txPin), m_rxPin(rxPin), m_dePin(dePin), m_rePin(rePin), m_baudRate(baudRate), m_isTransmitting(false), m_initialized(false)
{
}

bool RS485Driver::begin()
{
    if (m_initialized)
    {
        return true;
    }

    // ピンの初期化
    m_pinInterface.pinMode(m_txPin, OUTPUT);
    m_pinInterface.pinMode(m_rxPin, INPUT);
    m_pinInterface.pinMode(m_dePin, OUTPUT);
    m_pinInterface.pinMode(m_rePin, OUTPUT);

    // 初期状態は受信モード
    enableReceive();

    m_initialized = true;
    return true;
}

bool RS485Driver::transmit(const uint8_t *data, size_t bitLength)
{
    if (!m_initialized || !data || bitLength == 0)
    {
        return false;
    }

    // 送信モードに切り替え
    enableTransmit();

    // 少し待機してラインが安定するのを待つ
    m_pinInterface.delayMicroseconds(100);

    // データをビット単位で送信
    for (size_t i = 0; i < bitLength; i++)
    {
        size_t byteIndex = i / 8;
        size_t bitIndex = i % 8;
        uint8_t bit = (data[byteIndex] >> (7 - bitIndex)) & 1;
        transmitBit(bit);
    }

    // 送信完了後、受信モードに戻す
    m_pinInterface.delayMicroseconds(100);
    enableReceive();

    return true;
}

size_t RS485Driver::read(uint8_t *buffer, size_t maxBits, uint32_t timeoutMs)
{
    if (!m_initialized || !buffer || maxBits == 0)
    {
        return 0;
    }

    // 受信モードに設定
    enableReceive();

    uint32_t startTime = m_pinInterface.millis();
    size_t bitsReceived = 0;
    uint32_t bitDelay = 1000000UL / m_baudRate; // マイクロ秒単位のビット間隔

    // 最初の状態変化を待つ（フレーム開始の検出）
    uint8_t currentPinState = m_pinInterface.digitalRead(m_rxPin);
    uint8_t lastPinState = currentPinState;

    while (bitsReceived < maxBits)
    {
        // タイムアウトチェック
        if (timeoutMs > 0 && (m_pinInterface.millis() - startTime) > timeoutMs)
        {
            break;
        }

        currentPinState = m_pinInterface.digitalRead(m_rxPin);

        // 状態変化があった場合、ビットタイミングで同期
        if (currentPinState != lastPinState)
        {
            // ビット中央でサンプリングするために半ビット時間待機
            m_pinInterface.delayMicroseconds(bitDelay / 2);

            // ビットデータを読み取り
            uint8_t bit = m_pinInterface.digitalRead(m_rxPin) ? 1 : 0;

            // バッファに格納
            size_t byteIndex = bitsReceived / 8;
            size_t bitIndex = 7 - (bitsReceived % 8); // MSBファースト

            if (bit)
            {
                buffer[byteIndex] |= (1 << bitIndex);
            }
            else
            {
                buffer[byteIndex] &= ~(1 << bitIndex);
            }

            bitsReceived++;
            lastPinState = currentPinState;

            // 次のビットタイミングまで待機
            m_pinInterface.delayMicroseconds(bitDelay / 2);
        }

        // 短時間の待機でCPU負荷を軽減
        m_pinInterface.delayMicroseconds(bitDelay / 10);
    }

    return bitsReceived;
}

void RS485Driver::enableTransmit()
{
    m_pinInterface.digitalWrite(m_dePin, HIGH); // ドライバイネーブル
    m_pinInterface.digitalWrite(m_rePin, HIGH); // レシーバディスエーブル
    m_isTransmitting = true;
}

void RS485Driver::enableReceive()
{
    m_pinInterface.digitalWrite(m_dePin, LOW); // ドライバディスエーブル
    m_pinInterface.digitalWrite(m_rePin, LOW); // レシーバイネーブル
    m_isTransmitting = false;
}

bool RS485Driver::isTransmitting() const
{
    return m_isTransmitting;
}

void RS485Driver::transmitBit(uint8_t bit)
{
    m_pinInterface.digitalWrite(m_txPin, bit ? HIGH : LOW);
    bitDelay();
}

void RS485Driver::bitDelay()
{
    // ボーレートに基づいた遅延
    // 1 / baudRate * 1000000 (マイクロ秒)
    uint32_t delayMicros = 1000000UL / m_baudRate;
    m_pinInterface.delayMicroseconds(delayMicros);
}
