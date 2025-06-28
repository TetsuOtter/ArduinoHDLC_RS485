# Arduino HDLC RS485 ライブラリドキュメント

## 概要

このライブラリは、Arduino 環境で RS485 通信と HDLC プロトコルを実装するためのものです。LTC485CN8 ドライバを使用した RS485 通信の制御と、HDLC フレーミング、CRC 計算、データ検証機能を提供します。

## クラス構成

### RS485Driver クラス

RS485 通信の基本的な制御を行うクラスです。

#### 主要機能

- RS485 ドライバの初期化と制御
- ビット単位でのデータ送信（ブロッキング）
- 割り込みベースでの受信
- 送信/受信モードの自動切り替え

#### 使用例

```cpp
RS485Driver driver(2, 3, 4, 5, 9600); // TX, RX, DE, RE, BaudRate
driver.begin();

uint8_t data[] = {0xA5}; // 1バイト = 8ビット
driver.transmit(data, 8);
```

### HDLC クラス

HDLC プロトコルに基づくデータの送受信を行うクラスです。

#### 主要機能

- HDLC フレーミング（フラグ、スタッフィング）
- CRC-16 計算とチェック
- 16 進数文字列とバイト配列の相互変換
- 受信データのキューイング
- コールバックベースの受信処理

#### 使用例

```cpp
RS485Driver driver(2, 3, 4, 5, 9600);
HDLC hdlc(driver);
hdlc.begin();

// バイト配列での送信
uint8_t data[] = {0x01, 0x02, 0x03};
hdlc.transmitFrame(data, sizeof(data));

// 16進数文字列での送信
hdlc.transmitHexString("AA BB CC DD");

// 受信コールバックの設定
hdlc.setReceiveCallback([](const uint8_t* data, size_t length, bool isValid) {
    // 受信処理
});
```

## ピン配置

### LTC485CN8 接続例（Arduino Uno）

| LTC485CN8 ピン | Arduino Uno | 説明               |
| -------------- | ----------- | ------------------ |
| DI (1)         | D2          | 送信データ入力     |
| RO (4)         | D3          | 受信データ出力     |
| DE (2)         | D4          | ドライバイネーブル |
| RE (3)         | D5          | レシーバイネーブル |
| A (6)          | RS485+      | 非反転出力         |
| B (7)          | RS485-      | 反転出力           |
| VCC (8)        | 5V          | 電源               |
| GND (5)        | GND         | グランド           |

## HDLC フレーム構造

```
+------+--------+------+------+
| Flag | Data   | CRC  | Flag |
| 7E   | ...    | XXXX | 7E   |
+------+--------+------+------+
```

- **Flag**: 0x7E (フレーム境界)
- **Data**: ペイロードデータ（スタッフィング適用）
- **CRC**: CRC-16-IBM（2 バイト、スタッフィング適用）
- **Flag**: 0x7E (フレーム終了)

### バイトスタッフィング

| 元データ | スタッフィング後 |
| -------- | ---------------- |
| 0x7E     | 0x7D 0x5E        |
| 0x7D     | 0x7D 0x5D        |

## API リファレンス

### RS485Driver

#### コンストラクタ

```cpp
RS485Driver(uint8_t txPin, uint8_t rxPin, uint8_t dePin, uint8_t rePin, uint32_t baudRate)
```

#### メソッド

- `bool begin()` - 初期化
- `bool transmit(const uint8_t* data, size_t bitLength)` - データ送信
- `void setReceiveCallback(BitReceivedCallback callback)` - 受信コールバック設定
- `void startReceive()` - 受信開始
- `void stopReceive()` - 受信停止
- `void enableTransmit()` - 送信モード
- `void enableReceive()` - 受信モード
- `bool isTransmitting()` - 送信モード確認

### HDLC

#### コンストラクタ

```cpp
HDLC(RS485Driver& driver)
```

#### メソッド

- `bool begin()` - 初期化
- `bool transmitFrame(const uint8_t* data, size_t length)` - フレーム送信
- `bool transmitHexString(const String& hexString)` - 16 進数文字列送信
- `void setReceiveCallback(FrameReceivedCallback callback)` - 受信コールバック設定
- `bool receiveFrameWithBitControl(uint32_t timeoutMs = 5000)` - フレーム受信（低レベルビット制御）
- `size_t readFrame(uint8_t* buffer, size_t bufferSize)` - フレーム読み出し
- `String readFrameAsHexString()` - 16 進数文字列として読み出し
- `static uint16_t calculateCRC16(const uint8_t* data, size_t length)` - CRC 計算

## テスト

単体テストが含まれています。PlatformIO でテストを実行するには：

```bash
pio test
```

## 使用例

### 基本的な送信

```cpp
#include "RS485Driver.h"
#include "HDLC.h"

RS485Driver driver(2, 3, 4, 5, 9600);
HDLC hdlc(driver);

void setup() {
    hdlc.begin();
    hdlc.transmitHexString("01 02 03 FF");
}
```

### 受信処理

```cpp
void onFrameReceived(const uint8_t* data, size_t length, bool isValid) {
    if (isValid) {
        // 有効なフレームを受信
        for (size_t i = 0; i < length; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void setup() {
    hdlc.begin();
    hdlc.setReceiveCallback(onFrameReceived);
}

void loop() {
    // ポーリングベースで受信を試行
    hdlc.receiveFrameWithBitControl(100);
    delay(10);
}
```

## 制限事項

- 最大フレームサイズ: 256 バイト
- 受信キューサイズ: 1 フレーム（簡易実装）
- マスターノードのみ実装
- 同時送受信は未対応

## 注意事項

- RS485 バスには 120Ω の終端抵抗が必要です
- 長距離通信時は信号品質に注意してください
- 割り込みピンは Arduino Uno の場合、D2 または D3 を使用してください
