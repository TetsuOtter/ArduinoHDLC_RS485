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

RS485Driver::RS485Driver(
    IPinInterface &pinInterface,
    uint8_t txPin,
    uint8_t rxPin,
    uint8_t dePin,
    uint8_t rePin,
    uint32_t baudRate)
    : m_pinInterface(pinInterface),
      m_txPin(txPin),
      m_rxPin(rxPin),
      m_dePin(dePin),
      m_rePin(rePin),
      m_baudRate(baudRate),
      m_isTransmitting(false),
      m_initialized(false)
{
}

bool RS485Driver::begin()
{
    if (this->m_initialized)
    {
        return true;
    }

    // ピンの初期化
    this->m_pinInterface.pinMode(this->m_txPin, OUTPUT);
    this->m_pinInterface.pinMode(this->m_rxPin, INPUT);
    this->m_pinInterface.pinMode(this->m_dePin, OUTPUT);
    this->m_pinInterface.pinMode(this->m_rePin, OUTPUT);

    // 初期状態は受信モード
    this->enableReceive();

    this->m_initialized = true;
    return true;
}

bool RS485Driver::transmit(const uint8_t *data, size_t bitLength)
{
    if (!this->m_initialized || !data || bitLength == 0)
    {
        return false;
    }

    // 送信モードに切り替え
    this->enableTransmit();

    // 少し待機してラインが安定するのを待つ
    this->m_pinInterface.delayMicroseconds(100);

    // データをビット単位で送信
    for (size_t i = 0; i < bitLength; i++)
    {
        size_t byteIndex = i / 8;
        size_t bitIndex = i % 8;
        uint8_t bit = (data[byteIndex] >> (7 - bitIndex)) & 1;
        this->_transmitBit(bit);
    }

    // 送信完了後、受信モードに戻す
    this->m_pinInterface.delayMicroseconds(100);
    this->enableReceive();

    return true;
}

size_t RS485Driver::read(uint8_t *buffer, size_t maxBits, uint32_t timeoutMs)
{
    if (!this->m_initialized || !buffer || maxBits == 0)
    {
        return 0;
    }

    // 受信モードに設定
    this->enableReceive();

    uint32_t startTime = this->m_pinInterface.millis();
    size_t bitsReceived = 0;

    // バッファを初期化
    size_t maxBytes = (maxBits + 7) / 8;
    for (size_t i = 0; i < maxBytes; i++)
    {
        buffer[i] = 0;
    }

    while (bitsReceived < maxBits)
    {
        // タイムアウトチェック
        if (timeoutMs > 0 && (this->m_pinInterface.millis() - startTime) > timeoutMs)
        {
            break;
        }

        // 1ビット読み取り
        uint8_t bit = this->readBit();

        // バッファに格納
        size_t byteIndex = bitsReceived / 8;
        size_t bitIndex = 7 - (bitsReceived % 8); // MSBファースト

        if (bit)
        {
            buffer[byteIndex] |= (1 << bitIndex);
        }

        bitsReceived++;

        // 次のビットタイミングまで待機
        this->waitBitTime();
    }

    return bitsReceived;
}

uint8_t RS485Driver::readBit()
{
    if (!this->m_initialized)
    {
        return 0;
    }

    // 受信モードに設定（必要に応じて）
    if (this->m_isTransmitting)
    {
        this->enableReceive();
    }

    return this->m_pinInterface.digitalRead(this->m_rxPin) ? 1 : 0;
}

void RS485Driver::waitBitTime()
{
    uint32_t delayMicros = 1000000UL / this->m_baudRate;
    this->m_pinInterface.delayMicroseconds(delayMicros);
}

void RS485Driver::waitHalfBitTime()
{
    uint32_t delayMicros = (1000000UL / this->m_baudRate) / 2;
    this->m_pinInterface.delayMicroseconds(delayMicros);
}

void RS485Driver::waitBitTime(uint32_t elapsedMicros)
{
    uint32_t bitTimeMicros = 1000000UL / this->m_baudRate;
    if (elapsedMicros < bitTimeMicros)
    {
        uint32_t remainingMicros = bitTimeMicros - elapsedMicros;
        this->m_pinInterface.delayMicroseconds(remainingMicros);
    }
}

IPinInterface &RS485Driver::getPinInterface()
{
    return this->m_pinInterface;
}

uint32_t RS485Driver::getBaudRate() const
{
    return this->m_baudRate;
}

void RS485Driver::enableTransmit()
{
    this->m_pinInterface.digitalWrite(this->m_dePin, HIGH); // ドライバイネーブル
    this->m_pinInterface.digitalWrite(this->m_rePin, HIGH); // レシーバディスエーブル
    this->m_isTransmitting = true;
}

void RS485Driver::enableReceive()
{
    this->m_pinInterface.digitalWrite(this->m_dePin, LOW); // ドライバディスエーブル
    this->m_pinInterface.digitalWrite(this->m_rePin, LOW); // レシーバイネーブル
    this->m_isTransmitting = false;
}

bool RS485Driver::isTransmitting() const
{
    return this->m_isTransmitting;
}

void RS485Driver::_transmitBit(uint8_t bit)
{
    this->m_pinInterface.digitalWrite(this->m_txPin, bit ? HIGH : LOW);
    this->waitBitTime();
}
