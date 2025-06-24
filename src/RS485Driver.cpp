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

// 静的メンバの初期化
RS485Driver* RS485Driver::s_instance = nullptr;

RS485Driver::RS485Driver(IPinInterface& pinInterface, uint8_t txPin, uint8_t rxPin, uint8_t dePin, uint8_t rePin, uint32_t baudRate)
    : m_pinInterface(pinInterface)
    , m_txPin(txPin)
    , m_rxPin(rxPin)
    , m_dePin(dePin)
    , m_rePin(rePin)
    , m_baudRate(baudRate)
    , m_isTransmitting(false)
    , m_initialized(false)
    , m_receiveCallback(nullptr)
{
    s_instance = this;
}

bool RS485Driver::begin() {
    if (m_initialized) {
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

bool RS485Driver::transmit(const uint8_t* data, size_t bitLength) {
    if (!m_initialized || !data || bitLength == 0) {
        return false;
    }

    // 送信モードに切り替え
    enableTransmit();
    
    // 少し待機してラインが安定するのを待つ
    m_pinInterface.delayMicroseconds(100);

    // データをビット単位で送信
    for (size_t i = 0; i < bitLength; i++) {
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

void RS485Driver::setReceiveCallback(BitReceivedCallback callback) {
    m_receiveCallback = callback;
}

void RS485Driver::startReceive() {
    if (!m_initialized) {
        return;
    }

    // 受信モードに切り替え
    enableReceive();
    
    // 受信ピンに割り込みを設定 (Arduino Unoの場合、pin2=int0, pin3=int1)
    uint8_t interruptNum = (m_rxPin == 2) ? 0 : (m_rxPin == 3) ? 1 : 255;
    if (interruptNum != 255) {
        m_pinInterface.attachInterrupt(interruptNum, receiveInterruptHandler, CHANGE);
    }
}

void RS485Driver::stopReceive() {
    // 割り込みを無効化
    uint8_t interruptNum = (m_rxPin == 2) ? 0 : (m_rxPin == 3) ? 1 : 255;
    if (interruptNum != 255) {
        m_pinInterface.detachInterrupt(interruptNum);
    }
}

void RS485Driver::enableTransmit() {
    m_pinInterface.digitalWrite(m_dePin, HIGH);  // ドライバイネーブル
    m_pinInterface.digitalWrite(m_rePin, HIGH);  // レシーバディスエーブル
    m_isTransmitting = true;
}

void RS485Driver::enableReceive() {
    m_pinInterface.digitalWrite(m_dePin, LOW);   // ドライバディスエーブル
    m_pinInterface.digitalWrite(m_rePin, LOW);   // レシーバイネーブル
    m_isTransmitting = false;
}

bool RS485Driver::isTransmitting() const {
    return m_isTransmitting;
}

void RS485Driver::transmitBit(uint8_t bit) {
    m_pinInterface.digitalWrite(m_txPin, bit ? HIGH : LOW);
    bitDelay();
}

void RS485Driver::bitDelay() {
    // ボーレートに基づいた遅延
    // 1 / baudRate * 1000000 (マイクロ秒)
    uint32_t delayMicros = 1000000UL / m_baudRate;
    m_pinInterface.delayMicroseconds(delayMicros);
}

void RS485Driver::receiveInterruptHandler() {
    if (s_instance) {
        s_instance->handleReceive();
    }
}

void RS485Driver::handleReceive() {
    if (m_isTransmitting || !m_receiveCallback) {
        return;
    }

    // 受信ピンの状態を読み取り
    uint8_t bit = m_pinInterface.digitalRead(m_rxPin) ? 1 : 0;
    
    // コールバック関数を呼び出し
    m_receiveCallback(bit);
}
