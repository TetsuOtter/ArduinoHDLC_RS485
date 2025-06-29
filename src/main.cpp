#include <Arduino.h>

#ifndef UNIT_TEST

#include "RS485Driver.h"
#include "HDLC.h"
#include "ArduinoPinInterface.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO)
#define RS485_RX_PIN 5
#define RS485_RE_PIN 4
#define RS485_DE_PIN 3
#define RS485_TX_PIN 2
#else
#define RS485_TX_PIN D2
#define RS485_RX_PIN D5
#define RS485_DE_PIN D3
#define RS485_RE_PIN D4
#endif

#define RS485_BAUD_RATE 4800

// グローバルオブジェクト
ArduinoPinInterface pinInterface;
RS485Driver rs485Driver(pinInterface, RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN, RS485_RE_PIN, RS485_BAUD_RATE);
HDLC hdlc(rs485Driver);

// 受信データ処理用
size_t serialBufferLength = 0;
char serialBuffer[128] = {0}; // 明示的にゼロ初期化
bool commandReady = false;

/**
 * @brief HDLC受信フレームのコールバック関数
 * @param data 受信したデータ
 * @param length データ長
 * @param isValid CRCチェックの結果
 */
void onFrameReceived(const uint8_t *data, size_t length, bool isValid)
{
    Serial.print("Received HDLC frame: ");

    if (isValid)
    {
        Serial.print("VALID - ");

        // 16進数文字列として表示
        for (size_t i = 0; i < length; i++)
        {
            if (data[i] < 0x10)
            {
                Serial.print("0");
            }
            Serial.print(data[i], HEX);
            if (i < length - 1)
            {
                Serial.print(" ");
            }
        }
        Serial.println();
    }
    else
    {
        Serial.println("INVALID CRC");
    }
}

/**
 * @brief Serial入力の処理
 */
void processSerialInput()
{
    while (Serial.available())
    {
        char c = Serial.read();
        Serial.print(c);
        Serial.flush();

        if (c == '\n' || c == '\r')
        {
            if (serialBufferLength > 0)
            {
                commandReady = true;
                serialBuffer[serialBufferLength] = '\0';
            }
        }
        else
        {
            if (serialBufferLength < sizeof(serialBuffer) - 1)
            {
                serialBuffer[serialBufferLength++] = c;
            }
        }
    }
}

/**
 * @brief 受信したコマンドの処理
 */
void processCommand()
{
    if (!commandReady)
    {
        return;
    }

    commandReady = false;

    if (serialBufferLength == 0)
    {
        Serial.println("No command to process.");
        return;
    }

    Serial.print("Transmitting: ");
    Serial.println(serialBuffer);
    Serial.print("Buffer length: ");
    Serial.println(serialBufferLength);

    // char配列をHDLCで送信
    if (hdlc.transmitHexString(serialBuffer))
    {
        Serial.println("Transmission successful");
    }
    else
    {
        Serial.println("Transmission failed");
    }

    // バッファをクリア
    serialBufferLength = 0;
}

/**
 * @brief システムステータスの表示
 */
void printStatus()
{
    Serial.println("=== Arduino HDLC RS485 Communication ===");
    Serial.println("Usage: Send hex string via Serial (e.g., '01 02 FF')");
    Serial.println("System initialized and ready.");
    Serial.print("RS485 Baud Rate: ");
    Serial.println(RS485_BAUD_RATE);
    Serial.println("Waiting for commands...");
}

void setup()
{
    // シリアル通信の初期化
    Serial.begin(115200);
#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    bool ledState = false;
#endif
    while (!Serial)
    {
#ifdef LED_BUILTIN
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        ledState = !ledState;
#endif
        delay(500);
    }
#ifdef LED_BUILTIN
    for (int i = 0; i < 20; i++)
    {
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        ledState = !ledState;
    }
#endif

    Serial.println("Initializing Arduino HDLC RS485 Communication...");

    delay(1000); // 安定化のため少し待機

    // HDLC初期化
    if (!hdlc.begin())
    {
        Serial.println("ERROR: Failed to initialize HDLC");
        while (1)
        {
            delay(1000);
        }
    }

    // ステータス表示
    printStatus();
}

void loop()
{
    // Serial入力の処理
    processSerialInput();

    // コマンド処理
    processCommand();

    // ポーリングベースでの受信チェック（短いタイムアウトで）
    // NRZ方式に対応したビット制御ベースの受信を使用
    if (hdlc.receiveFrameWithBitControl(50))
    { // 50msタイムアウト
        // 受信完了後、キューからデータを読み出し
        uint8_t buffer[256];
        size_t receivedLength = hdlc.readFrame(buffer, sizeof(buffer));
        if (receivedLength > 0)
        {
            // コールバック関数を呼び出し（互換性のため）
            onFrameReceived(buffer, receivedLength, true); // CRC検証は内部で実施済み
        }
    }

    // 少し待機
    delay(10);
}

#endif // UNIT_TEST