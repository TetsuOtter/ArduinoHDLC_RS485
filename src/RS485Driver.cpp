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
      m_initialized(false),
      m_bitTimeMicros(1000000UL / baudRate),
      m_halfBitTimeMicros((1000000UL / baudRate) / 2)
{
}

bool RS485Driver::begin()
{
#ifndef NATIVE_TEST
    Serial.print("RS485: Init pins TX=");
    Serial.print(this->m_txPin);
    Serial.print(" RX=");
    Serial.print(this->m_rxPin);
    Serial.print(" DE=");
    Serial.print(this->m_dePin);
    Serial.print(" RE=");
    Serial.println(this->m_rePin);
#endif

    if (this->m_initialized)
    {
#ifndef NATIVE_TEST
        Serial.println("RS485: Already init");
#endif
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
#ifndef NATIVE_TEST
    Serial.println("RS485: Init OK");
#endif
    return true;
}

bool RS485Driver::transmit(const uint8_t *data, size_t bitLength)
{
    if (!this->m_initialized || !data || bitLength == 0)
    {
#ifndef NATIVE_TEST
        Serial.println("RS485: TX invalid params");
#endif
        return false;
    }

#ifndef NATIVE_TEST
    Serial.print("RS485: TX ");
    Serial.print(bitLength);
    Serial.println(" bits");
    // データも出力
    Serial.print("Data: ");
    for (size_t i = 0; i < (bitLength + 7) / 8; i++)
    {
        Serial.print(data[i], HEX);
        if (i < (bitLength + 7) / 8 - 1)
        {
            Serial.print(" ");
        }
    }
    Serial.println();
#endif

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
    this->enableReceive();

#ifndef NATIVE_TEST
    Serial.println("RS485: TX done");
#endif
    return true;
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
    this->m_pinInterface.delayMicroseconds(this->m_bitTimeMicros);
}

void RS485Driver::waitHalfBitTime()
{
    this->m_pinInterface.delayMicroseconds(this->m_halfBitTimeMicros);
}

void RS485Driver::waitBitTime(uint32_t elapsedMicros)
{
    if (elapsedMicros < this->m_bitTimeMicros)
    {
        uint32_t remainingMicros = this->m_bitTimeMicros - elapsedMicros;
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
#ifndef NATIVE_TEST
    Serial.println("RS485: Enable TX mode");
#endif
    this->m_pinInterface.digitalWrite(this->m_dePin, HIGH); // ドライバイネーブル
    this->m_pinInterface.digitalWrite(this->m_rePin, HIGH); // レシーバディスエーブル
    this->m_isTransmitting = true;
}

void RS485Driver::enableReceive()
{
#ifndef NATIVE_TEST
    Serial.println("RS485: Enable RX mode");
#endif
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
