#include <Arduino.h>

#ifndef UNIT_TEST

#include "RS485Driver.h"
#include "HDLC.h"
#include "ArduinoPinInterface.h"

// RS485通信ピンの定義 (Arduino Uno用)
#define RS485_TX_PIN 2
#define RS485_RX_PIN 3
#define RS485_DE_PIN 4
#define RS485_RE_PIN 5
#define RS485_BAUD_RATE 9600

// グローバルオブジェクト
ArduinoPinInterface pinInterface;
RS485Driver rs485Driver(pinInterface, RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN, RS485_RE_PIN, RS485_BAUD_RATE);
HDLC hdlc(rs485Driver);

// 受信データ処理用
String serialBuffer = "";
bool commandReady = false;

/**
 * @brief HDLC受信フレームのコールバック関数
 * @param data 受信したデータ
 * @param length データ長
 * @param isValid CRCチェックの結果
 */
void onFrameReceived(const uint8_t* data, size_t length, bool isValid) {
    Serial.print("Received HDLC frame: ");
    
    if (isValid) {
        Serial.print("VALID - ");
        
        // 16進数文字列として表示
        for (size_t i = 0; i < length; i++) {
            if (data[i] < 0x10) {
                Serial.print("0");
            }
            Serial.print(data[i], HEX);
            if (i < length - 1) {
                Serial.print(" ");
            }
        }
        Serial.println();
    } else {
        Serial.println("INVALID CRC");
    }
}

/**
 * @brief Serial入力の処理
 */
void processSerialInput() {
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                commandReady = true;
            }
        } else {
            serialBuffer += c;
        }
    }
}

/**
 * @brief 受信したコマンドの処理
 */
void processCommand() {
    if (!commandReady) {
        return;
    }
    
    commandReady = false;
    
    // 空白を除去
    serialBuffer.trim();
    
    if (serialBuffer.length() == 0) {
        serialBuffer = "";
        return;
    }
    
    Serial.print("Transmitting: ");
    Serial.println(serialBuffer);
    
    // 16進数文字列をHDLCで送信
    if (hdlc.transmitHexString(serialBuffer)) {
        Serial.println("Transmission successful");
    } else {
        Serial.println("Transmission failed");
    }
    
    serialBuffer = "";
}

/**
 * @brief システムステータスの表示
 */
void printStatus() {
    Serial.println("=== Arduino HDLC RS485 Communication ===");
    Serial.println("Usage: Send hex string via Serial (e.g., '01 02 FF')");
    Serial.println("System initialized and ready.");
    Serial.print("RS485 Baud Rate: ");
    Serial.println(RS485_BAUD_RATE);
    Serial.println("Waiting for commands...");
}

void setup() {
    // シリアル通信の初期化
    Serial.begin(9600);
    while (!Serial) {
        ; // シリアルポートが接続されるまで待機
    }
    
    delay(1000); // 安定化のため少し待機
    
    // HDLC初期化
    if (!hdlc.begin()) {
        Serial.println("ERROR: Failed to initialize HDLC");
        while (1) {
            delay(1000);
        }
    }
    
    // 受信コールバックの設定
    hdlc.setReceiveCallback(onFrameReceived);
    
    // 受信開始
    hdlc.startReceive();
    
    // ステータス表示
    printStatus();
}

void loop() {
    // Serial入力の処理
    processSerialInput();
    
    // コマンド処理
    processCommand();
    
    // 受信データの確認 (コールバックが設定されていない場合)
    String receivedHex = hdlc.readFrameAsHexString();
    if (receivedHex.length() > 0) {
        Serial.print("Received (from queue): ");
        Serial.println(receivedHex);
    }
    
    // 少し待機
    delay(10);
}

#endif // UNIT_TEST