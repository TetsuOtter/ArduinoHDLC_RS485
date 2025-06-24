#include <Arduino.h>
#include "RS485Driver.h"
#include "HDLC.h"

// この例では、RS485通信の基本的な使用方法を示します

// ピン定義
#define RS485_TX_PIN 2
#define RS485_RX_PIN 3  
#define RS485_DE_PIN 4
#define RS485_RE_PIN 5
#define BAUD_RATE 9600

// グローバルオブジェクト
RS485Driver driver(RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN, RS485_RE_PIN, BAUD_RATE);
HDLC hdlc(driver);

void setup() {
    Serial.begin(9600);
    
    // 初期化
    if (!hdlc.begin()) {
        Serial.println("Initialization failed!");
        return;
    }
    
    // 例1: 基本的なデータ送信
    uint8_t data1[] = {0x01, 0x02, 0x03};
    hdlc.transmitFrame(data1, sizeof(data1));
    
    // 例2: 16進数文字列での送信
    hdlc.transmitHexString("AA BB CC DD");
    
    // 例3: 受信コールバックの設定
    hdlc.setReceiveCallback([](const uint8_t* data, size_t length, bool isValid) {
        Serial.print("Received: ");
        for (size_t i = 0; i < length; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println(isValid ? "(Valid)" : "(Invalid)");
    });
    
    hdlc.startReceive();
}

void loop() {
    // メインループでは特に何もしない
    // 受信はコールバックで処理される
    delay(1000);
}
