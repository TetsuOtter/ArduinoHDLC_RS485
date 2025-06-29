#include <Arduino.h>

#ifndef UNIT_TEST

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
HDLC hdlc(pinInterface, RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN, RS485_RE_PIN, RS485_BAUD_RATE);

// 受信データ処理用
size_t binaryBufferLength = 0;
uint8_t binaryBuffer[64] = {0}; // バイナリデータ用バッファ（64バイト = 128文字の16進文字列に対応）
char hexChar = 0;               // 16進文字のペア処理用
bool hasHexChar = false;
bool commandReady = false;

/**
 * @brief 16進数文字を数値に変換するユーティリティ関数
 * @param hexChar 16進数文字 ('0'-'9', 'A'-'F', 'a'-'f')
 * @return 対応する数値 (0-15)、無効な文字の場合は255
 */
static uint8_t hexCharToValue(char hexChar)
{
    if (hexChar >= '0' && hexChar <= '9')
    {
        return hexChar - '0';
    }
    else if (hexChar >= 'A' && hexChar <= 'F')
    {
        return hexChar - 'A' + 10;
    }
    else if (hexChar >= 'a' && hexChar <= 'f')
    {
        return hexChar - 'a' + 10;
    }
    else
    {
        return 255; // 無効な文字
    }
}

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
 * @brief Serial入力の処理（16進文字列を直接バイナリに変換）
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
            if (binaryBufferLength > 0)
            {
                commandReady = true;
            }
        }
        else if (c == ' ')
        {
            // スペースは無視
            continue;
        }
        else
        {
            // 16進文字の処理
            uint8_t hexValue = hexCharToValue(c);
            if (hexValue != 255) // 有効な16進文字
            {
                if (!hasHexChar)
                {
                    // 上位4ビット
                    hexChar = hexValue << 4;
                    hasHexChar = true;
                }
                else
                {
                    // 下位4ビットと結合してバイナリバッファに格納
                    hexChar |= hexValue;
                    if (binaryBufferLength < sizeof(binaryBuffer))
                    {
                        binaryBuffer[binaryBufferLength++] = hexChar;
                    }
                    hasHexChar = false;
                }
            }
        }
    }
}

/**
 * @brief 受信したコマンドの処理（SNRM→UA待機→Iコマンド送信フロー）
 */
void processCommand()
{
    if (!commandReady)
    {
        return;
    }

    commandReady = false;

    if (binaryBufferLength == 0)
    {
        Serial.println("No command to process.");
        return;
    }

    Serial.print("Sending I-frame with data: ");
    for (size_t i = 0; i < binaryBufferLength; i++)
    {
        if (binaryBuffer[i] < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(binaryBuffer[i], HEX);
        if (i < binaryBufferLength - 1)
        {
            Serial.print(" ");
        }
    }
    Serial.println();

    // 1. SNRMコマンド送信とUA応答待機
    Serial.println("Step 1: Sending SNRM and waiting for UA...");
    if (!hdlc.sendSNRMAndWaitUA())
    {
        Serial.println("ERROR: SNRM/UA handshake failed");
        binaryBufferLength = 0;
        return;
    }
    Serial.println("SNRM/UA handshake successful");

    // 2. Iコマンドでデータ送信
    Serial.println("Step 2: Sending I-frame...");
    if (hdlc.sendICommand(binaryBuffer, binaryBufferLength))
    {
        Serial.println("I-frame transmission successful");
    }
    else
    {
        Serial.println("I-frame transmission failed");
    }

    // バッファをクリア
    binaryBufferLength = 0;
}

/**
 * @brief システムステータスの表示
 */
void printStatus()
{
    Serial.println("=== Arduino HDLC RS485 Communication (Integrated) ===");
    Serial.println("Usage: Send hex string via Serial (e.g., '01 02 FF')");
    Serial.println("System automatically sends SNRM before each I-frame");
    Serial.println("System initialized and ready.");
    Serial.print("RS485 Baud Rate: ");
    Serial.println(RS485_BAUD_RATE);
    Serial.println("Waiting for I-frame data...");
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

    // Leonardo用の改良された初期化
#if defined(ARDUINO_AVR_LEONARDO)
    // Leonardoの場合：タイムアウト付きで待機
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime) < 3000) // 3秒でタイムアウト
    {
#ifdef LED_BUILTIN
        if ((millis() - startTime) % 250 == 0) // 250ms間隔で点滅
        {
            digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
            ledState = !ledState;
        }
#endif
        delay(50); // CPU負荷軽減
    }

    // Leonardoのシリアルバッファをクリア
    delay(500); // 追加の安定化時間
    while (Serial.available() > 0)
    {
        Serial.read(); // バッファクリア
    }
    Serial.flush(); // 送信バッファクリア

#else
    // 他のArduino（UNO等）の場合：従来通り
    while (!Serial)
    {
#ifdef LED_BUILTIN
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        ledState = !ledState;
#endif
        delay(500);
    }
#endif

#ifdef LED_BUILTIN
    // 初期化完了の合図（短い点滅）
    for (int i = 0; i < 6; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
#endif

    Serial.println("Initializing Arduino HDLC RS485 Communication...");
    Serial.flush(); // 出力完了を確実にする

    delay(1000); // 安定化のため少し待機

    // HDLC初期化
    if (!hdlc.begin())
    {
        Serial.println("ERROR: Failed to initialize HDLC");
        Serial.flush();
        while (1)
        {
#ifdef LED_BUILTIN
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
#endif
        }
    }

    // ステータス表示
    printStatus();
    Serial.flush(); // 出力完了を確実にする

    // Leonardo用：最終的なシリアルバッファクリア
#if defined(ARDUINO_AVR_LEONARDO)
    delay(200);
    while (Serial.available() > 0)
    {
        Serial.read();
    }
    Serial.println("Leonardo: Ready for input");
    Serial.flush();
#endif
}

void loop()
{
    // Serial入力の処理
    processSerialInput();

    // コマンド処理
    processCommand();

    // 少し待機
    delay(10);
}

#endif // UNIT_TEST