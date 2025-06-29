#include "HDLC.h"
#include <stdlib.h> // malloc, free用

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
#endif

HDLC::HDLC(IPinInterface &pinInterface, uint8_t txPin, uint8_t rxPin,
           uint8_t dePin, uint8_t rePin, uint32_t baudRate)
    : m_pinInterface(pinInterface),
      m_txPin(txPin),
      m_rxPin(rxPin),
      m_dePin(dePin),
      m_rePin(rePin),
      m_baudRate(baudRate),
      m_bitTimeMicros(1000000UL / baudRate),
      m_halfBitTimeMicros((1000000UL / baudRate) / 2),
      m_isTransmitting(false),
      m_initialized(false),
      m_targetAddress(0),
      m_sendSequence(0),
      m_receiveSequence(0),
      m_receiveIndex(0),
      m_currentByte(0),
      m_bitCount(0),
      m_consecutiveOnes(0)
{
    this->m_frameQueue.hasData = false;
    this->m_frameQueue.valid = false;
    this->m_frameQueue.length = 0;

    // 待機時間を事前計算
    this->m_shortDelayMicros = (1000000UL / baudRate) / 8; // 1/8ビット時間
}

bool HDLC::begin()
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
    this->_enableReceive();

    this->m_initialized = true;

#ifndef NATIVE_TEST
    // アドレス入力（不正な場合は繰り返し）
    bool validAddress = false;
    while (!validAddress)
    {
        Serial.println("Enter target address (0-255):");
        while (!Serial.available())
        {
            delay(100);
        }

        String addressStr = Serial.readStringUntil('\n');
        addressStr.trim();

        // 空文字列チェック
        if (addressStr.length() == 0)
        {
            Serial.println("Error: Empty input. Please enter a number between 0-255.");
            continue;
        }

        // 数値変換と範囲チェック
        int address = addressStr.toInt();

        // toInt()は不正な文字列の場合0を返すため、"0"と不正文字列を区別する
        if (address == 0 && addressStr != "0")
        {
            Serial.println("Error: Invalid input. Please enter a valid number between 0-255.");
            continue;
        }

        if (address >= 0 && address <= 255)
        {
            this->m_targetAddress = (uint8_t)address;
            Serial.print("Target address set to: ");
            Serial.println(this->m_targetAddress);
            validAddress = true;
        }
        else
        {
            Serial.println("Error: Address out of range. Please enter a number between 0-255.");
        }
    }

    Serial.println("Ready for I-frame data input");
#else
    this->m_targetAddress = 1; // テスト用デフォルト
#endif

    return true;
}

bool HDLC::sendSNRMAndWaitUA()
{
    if (!this->m_initialized)
    {
        return false;
    }

    // SNRMフレームの作成
    uint8_t snrmFrame[MAX_FRAME_SIZE];
    size_t frameLength = this->_createHDLCFrame(this->m_targetAddress, CMD_SNRM, nullptr, 0, snrmFrame, MAX_FRAME_SIZE);

    if (frameLength == 0)
    {
        return false;
    }

    // SNRMフレームを送信
    if (!this->_transmitFrame(snrmFrame, frameLength))
    {
        return false;
    }

    // UA応答を待機（5秒タイムアウト）
    if (!this->receiveFrameWithBitControl(500))
    {
        return false;
    }

    // 受信フレームの検証
    uint8_t buffer[MAX_FRAME_SIZE];
    size_t receivedLength = this->readFrame(buffer, MAX_FRAME_SIZE);

    if (receivedLength >= 3) // アドレス + コントロール + 最低限のCRC
    {
        // UAフレームかチェック（アドレスとコントロールフィールド）
        if (buffer[0] == this->m_targetAddress && buffer[1] == CMD_UA)
        {
            return true;
        }
    }

    return false;
}

bool HDLC::sendICommand(const uint8_t *data, size_t length)
{
    // デフォルトタイムアウト5秒で内部実装を呼び出し
    return this->sendICommand(data, length, 5000);
}

bool HDLC::sendICommand(const uint8_t *data, size_t length, uint32_t timeoutMs)
{
    if (!this->m_initialized || !data || length == 0)
    {
        return false;
    }

    // Iコマンドフレームの作成（送信シーケンス番号を含む）
    uint8_t control = CMD_I | (this->m_sendSequence << 1); // 送信シーケンス番号をビット1-3に設定
    uint8_t iFrame[MAX_FRAME_SIZE];
    size_t frameLength = this->_createHDLCFrame(this->m_targetAddress, control, data, length, iFrame, MAX_FRAME_SIZE);

    if (frameLength == 0)
    {
        return false;
    }

    // Iフレームを送信
    if (!this->_transmitFrame(iFrame, frameLength))
    {
        return false;
    }

#ifndef NATIVE_TEST
    Serial.print("I-frame sent, waiting for response (timeout: ");
    Serial.print(timeoutMs);
    Serial.println("ms)");
#endif

    // レスポンス待機（RRまたはREJフレーム）
    if (!this->receiveFrameWithBitControl(timeoutMs))
    {
#ifndef NATIVE_TEST
        Serial.println("Response timeout");
#endif
        return false; // タイムアウト
    }

    // レスポンスの検証
    uint8_t responseBuffer[MAX_FRAME_SIZE];
    size_t responseLength = this->readFrame(responseBuffer, MAX_FRAME_SIZE);

#ifndef NATIVE_TEST
    Serial.print("Response length: ");
    Serial.println(responseLength);
    Serial.println("Response frame: ");
    for (size_t i = 0; i < responseLength; i++)
    {
        Serial.print(responseBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif

    if (responseLength >= 2) // アドレス + コントロール 最低限
    {
        uint8_t responseAddress = responseBuffer[0];
        uint8_t responseControl = responseBuffer[1];

#ifndef NATIVE_TEST
        Serial.print("Response received - Address: 0x");
        Serial.print(responseAddress, HEX);
        Serial.print(", Control: 0x");
        Serial.println(responseControl, HEX);
#endif

        // アドレスが一致するかチェック
        if (responseAddress != this->m_targetAddress)
        {
#ifndef NATIVE_TEST
            Serial.println("Address mismatch in response");
#endif
            // アドレス不一致は一旦無視
            // return false; // アドレス不一致
        }

        // RRフレーム（正常応答）かチェック
        if (this->_isRRFrame(responseControl))
        {
            // 受信シーケンス番号を確認
            uint8_t receivedSeq = this->_extractSequenceNumber(responseControl);
#ifndef NATIVE_TEST
            Serial.print("RR frame received, sequence: ");
            Serial.print(receivedSeq);
            Serial.print(", expected: ");
            Serial.println(this->m_sendSequence);
#endif

            if (receivedSeq == this->m_sendSequence)
            {
                // 送信シーケンス番号を次に進める（0-7で循環）
                this->m_sendSequence = (this->m_sendSequence + 1) & 0x07;
#ifndef NATIVE_TEST
                Serial.println("I-frame acknowledged successfully");
#endif
                return true; // 正常応答
            }
        }
        // REJフレーム（再送要求）かチェック
        else if (this->_isREJFrame(responseControl))
        {
#ifndef NATIVE_TEST
            Serial.println("REJ frame received (retransmission required)");
#endif
            // REJフレームの場合は再送が必要だが、ここでは失敗として扱う
            return false; // 再送要求（エラー扱い）
        }
#ifndef NATIVE_TEST
        else
        {
            Serial.println("Unknown response frame type");
        }
#endif
    }
    else
    {
#ifndef NATIVE_TEST
        Serial.println("Invalid response length");
#endif
    }

    return false; // 不正なレスポンス
}

void HDLC::setAddress(uint8_t address)
{
    this->m_targetAddress = address;
}

bool HDLC::receiveFrameWithBitControl(uint32_t timeoutMs)
{
    if (!this->m_initialized)
    {
        return false;
    }

    // 受信状態を初期化
    this->_initializeReceiveState();
    this->_enableReceive();

    uint32_t startTime = this->m_pinInterface.millis();
    uint32_t bitStartTime;
    ReceiveContext context;
    this->_initializeReceiveContext(context);

    while ((this->m_pinInterface.millis() - startTime) < timeoutMs)
    {
        bitStartTime = this->m_pinInterface.micros();
        uint8_t bit = this->_readBit();

#ifndef NATIVE_TEST
        Serial.print("Received bit: ");
        Serial.println(bit);
#endif

        // フラグシーケンス検出処理
        this->_processReceivedBit(bit, context);

        // フレーム処理が完了した場合
        if (context.frameComplete)
        {
            return true;
        }

        // 経過時間を考慮したビット待機
        uint32_t elapsedMicros = this->m_pinInterface.micros() - bitStartTime;
        this->_waitBitTime(elapsedMicros);
    }

    return false;
}

void HDLC::_initializeReceiveState()
{
    this->m_receiveIndex = 0;
    this->m_currentByte = 0;
    this->m_bitCount = 0;
    this->m_consecutiveOnes = 0;
    this->m_frameQueue.hasData = false;
}

void HDLC::_initializeReceiveContext(ReceiveContext &context)
{
    context.flagBuffer = 0;
    context.flagBitCount = 0;
    context.inFrame = false;
    context.rawBitIndex = 0;
    context.frameComplete = false;
    // rawDataをゼロ初期化
    memset(context.rawData, 0, sizeof(context.rawData));
}

void HDLC::_processReceivedBit(uint8_t bit, ReceiveContext &context)
{
    // フラグシーケンス検出
    this->_updateFlagDetection(bit, context);

    if (this->_isFlagSequence(context))
    {
        this->_handleFlagSequence(context);
    }
    else if (context.inFrame)
    {
        this->_storeBitInFrame(bit, context);
    }
}

void HDLC::_updateFlagDetection(uint8_t bit, ReceiveContext &context)
{
    context.flagBuffer = (context.flagBuffer << 1) | bit;
    context.flagBitCount++;
    if (context.flagBitCount > 8)
    {
        context.flagBitCount = 8;
    }
}

bool HDLC::_isFlagSequence(const ReceiveContext &context)
{
    const uint8_t FLAG_SEQUENCE = 0x7E; // 01111110
    return (context.flagBuffer == FLAG_SEQUENCE && context.flagBitCount >= 8);
}

void HDLC::_handleFlagSequence(ReceiveContext &context)
{
    if (!context.inFrame)
    {
        // フレーム開始
        this->_startFrame(context);
    }
    else
    {
        // フレーム終了
        this->_endFrame(context);
    }
}

void HDLC::_startFrame(ReceiveContext &context)
{
    context.inFrame = true;
    context.rawBitIndex = 0;
    this->m_consecutiveOnes = 0;
}

void HDLC::_endFrame(ReceiveContext &context)
{
    if (context.rawBitIndex > 0)
    {
        if (this->_processCompleteFrame(context.rawData, context.rawBitIndex))
        {
            context.frameComplete = true;
            return;
        }
    }
    context.inFrame = false;
    context.rawBitIndex = 0;
}

void HDLC::_storeBitInFrame(uint8_t bit, ReceiveContext &context)
{
    if (context.rawBitIndex < sizeof(context.rawData) * 8)
    {
        size_t byteIndex = context.rawBitIndex / 8;
        size_t bitPos = context.rawBitIndex % 8;

        if (bit)
        {
            context.rawData[byteIndex] |= (1 << (7 - bitPos));
        }
        else
        {
            context.rawData[byteIndex] &= ~(1 << (7 - bitPos));
        }
        context.rawBitIndex++;
    }
}

bool HDLC::_processCompleteFrame(const uint8_t *rawData, size_t rawBitCount)
{
    if (!rawData || rawBitCount == 0)
    {
        return false;
    }

    // ビットデスタッフィングを直接実行
    uint8_t consecutiveOnes = 0;
    uint8_t currentByte = 0;
    int8_t bitPosition = 7;
    size_t outputByteIndex = 0;

    // 入力バイト配列をビット単位で処理（rawBitCountまで）
    for (size_t bitIdx = 0; bitIdx < rawBitCount; bitIdx++)
    {
        // ビットインデックスからバイトインデックスとビット位置を計算
        size_t byteIdx = bitIdx / 8;
        int bitPos = 7 - (bitIdx % 8);
        uint8_t bit = (rawData[byteIdx] >> bitPos) & 1;

        if (bit == 1)
        {
            consecutiveOnes++;
        }
        else
        {
            if (consecutiveOnes == 5)
            {
                // スタッフィングされた0ビットなので無視
                consecutiveOnes = 0;
                continue;
            }
            consecutiveOnes = 0;
        }

        // 通常のデータビット
        if (bit == 1)
        {
            currentByte |= (1 << bitPosition);
        }

        bitPosition--;

        if (bitPosition < 0)
        {
            // 1バイト完成
            if (outputByteIndex >= MAX_FRAME_SIZE)
            {
                return false; // バッファオーバーフロー
            }
            this->m_receiveBuffer[outputByteIndex++] = currentByte;
            currentByte = 0;
            bitPosition = 7;
        }
    }

    // 最低限のフレーム長チェック（アドレス+コントロール+CRC）
    if (outputByteIndex < 3)
    {
        return false;
    }

    // CRC検証
    if (!this->_validateFrameCRC(outputByteIndex))
    {
        return false;
    }

    // 有効なフレームをキューに保存
    this->_storeValidFrame(outputByteIndex);
    return true;
}

bool HDLC::_validateFrameCRC(size_t frameLength)
{
    uint16_t receivedCRC = (this->m_receiveBuffer[frameLength - 2] << 8) | this->m_receiveBuffer[frameLength - 1];
    uint16_t calculatedCRC = this->calculateCRC16(this->m_receiveBuffer, frameLength - 2);
    return (receivedCRC == calculatedCRC);
}

void HDLC::_storeValidFrame(size_t frameLength)
{
    this->m_frameQueue.length = frameLength - 2; // CRCを除く
    for (size_t i = 0; i < this->m_frameQueue.length && i < MAX_FRAME_SIZE; i++)
    {
        this->m_frameQueue.data[i] = this->m_receiveBuffer[i];
    }
    this->m_frameQueue.valid = true;
    this->m_frameQueue.hasData = true;
}

size_t HDLC::readFrame(uint8_t *buffer, size_t bufferSize)
{
    if (!buffer || bufferSize == 0 || !this->m_frameQueue.hasData)
    {
        return 0;
    }

    size_t copyLength = (this->m_frameQueue.length < bufferSize) ? this->m_frameQueue.length : bufferSize;
    for (size_t i = 0; i < copyLength; i++)
    {
        buffer[i] = this->m_frameQueue.data[i];
    }

    // フレームキューをクリア
    this->m_frameQueue.hasData = false;
    this->m_frameQueue.valid = false;
    this->m_frameQueue.length = 0;

    return copyLength;
}

// RS485制御メソッド
void HDLC::_enableTransmit()
{
    this->m_pinInterface.digitalWrite(this->m_dePin, HIGH);
    this->m_pinInterface.digitalWrite(this->m_rePin, HIGH);
    this->m_isTransmitting = true;
    this->m_pinInterface.delayMicroseconds(this->m_halfBitTimeMicros);
}

void HDLC::_enableReceive()
{
    this->m_pinInterface.digitalWrite(this->m_dePin, LOW);
    this->m_pinInterface.digitalWrite(this->m_rePin, LOW);
    this->m_isTransmitting = false;
    this->m_pinInterface.delayMicroseconds(this->m_halfBitTimeMicros);
}

void HDLC::_transmitBit(uint8_t bit)
{
    this->m_pinInterface.digitalWrite(this->m_txPin, bit ? HIGH : LOW);
}

uint8_t HDLC::_readBit()
{
    return this->m_pinInterface.digitalRead(this->m_rxPin) ? 1 : 0;
}

void HDLC::_waitBitTime()
{
    this->m_pinInterface.delayMicroseconds(this->m_bitTimeMicros);
}

void HDLC::_waitHalfBitTime()
{
    this->m_pinInterface.delayMicroseconds(this->m_halfBitTimeMicros);
}

void HDLC::_waitBitTime(uint32_t elapsedMicros)
{
    if (elapsedMicros < this->m_bitTimeMicros)
    {
        this->m_pinInterface.delayMicroseconds(this->m_bitTimeMicros - elapsedMicros);
    }
}

// 内部フレーム送信メソッド
bool HDLC::_transmitFrame(const uint8_t *data, size_t length)
{
    if (!this->m_initialized || !data || length == 0)
    {
        return false;
    }

#ifndef NATIVE_TEST
    Serial.print("Transmitting HDLC frame (");
    Serial.print(length);
    Serial.println(" bytes of data)");
    Serial.print("Data: ");
    for (size_t i = 0; i < length; i++)
    {
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif

    // 送信モードに切り替え
    this->_enableTransmit();
    this->m_pinInterface.delayMicroseconds(100); // 安定化待機

    // 開始フラグの送信 (0x7E = 01111110)
    this->_transmitByte(HDLC::FLAG_SEQUENCE);

    // データ送信（ビットスタッフィング付き）
    uint8_t consecutiveOnes = 0;

    for (size_t i = 0; i < length; i++)
    {
        this->_transmitByteWithStuffing(data[i], consecutiveOnes);
    }

    // 終了フラグの送信 (0x7E = 01111110)
    // フラグ送信前に連続1カウントをリセット
    consecutiveOnes = 0;
    this->_transmitByte(HDLC::FLAG_SEQUENCE);

    return true;
}

// HDLCフレーム作成メソッド
size_t HDLC::_createHDLCFrame(
    uint8_t address,
    uint8_t control,
    const uint8_t *info,
    size_t infoLength,
    uint8_t *frameBuffer,
    size_t maxLength)
{
    if (!frameBuffer || maxLength < 4) // 最低限: アドレス + コントロール + CRC(2)
    {
        return 0;
    }

    size_t frameIndex = 0;

    // アドレスフィールド
    frameBuffer[frameIndex++] = address;

    // コントロールフィールド
    frameBuffer[frameIndex++] = control;

    // 情報フィールド（存在する場合）
    if (info && infoLength > 0)
    {
        if (frameIndex + infoLength + 2 > maxLength) // CRC分も考慮
        {
            return 0; // バッファ不足
        }

        for (size_t i = 0; i < infoLength; i++)
        {
            frameBuffer[frameIndex++] = info[i];
        }
    }

    // CRC計算と追加
    uint16_t crc = this->calculateCRC16(frameBuffer, frameIndex);
    frameBuffer[frameIndex++] = (crc >> 8) & 0xFF; // CRC上位
    frameBuffer[frameIndex++] = crc & 0xFF;        // CRC下位

    return frameIndex;
}

uint16_t HDLC::calculateCRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF; // CRC-16-CCITT初期値

    for (size_t i = 0; i < length; i++)
    {
        crc ^= (data[i] << 8); // 上位8ビットにXOR
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021; // CRC-16-CCITT polynomial
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void HDLC::_processBit(uint8_t bit)
{
    // ビットデスタッフィング処理
    if (bit == 1)
    {
        this->m_consecutiveOnes++;
    }
    else
    {
        if (this->m_consecutiveOnes == 5)
        {
            // スタッフィングされた0ビット - 無視
            this->m_consecutiveOnes = 0;
            return;
        }
        else if (this->m_consecutiveOnes == 6)
        {
            // フラグシーケンスの可能性
            this->m_currentByte = (this->m_currentByte >> 1) | (bit << 7);
            this->m_bitCount++;

            if (this->m_bitCount == 8 && this->m_currentByte == FLAG_SEQUENCE)
            {
                // フレーム終了
                this->_processReceivedFrame();
                this->m_consecutiveOnes = 0;
                this->m_bitCount = 0;
                this->m_currentByte = 0;
                return;
            }
        }
        this->m_consecutiveOnes = 0;
    }

    // 通常のデータビット処理
    this->m_currentByte = (this->m_currentByte >> 1) | (bit << 7);
    this->m_bitCount++;

    if (this->m_bitCount == 8)
    {
        // 1バイト完了
        if (this->m_receiveIndex < MAX_FRAME_SIZE)
        {
            this->m_receiveBuffer[this->m_receiveIndex++] = this->m_currentByte;
        }
        this->m_currentByte = 0;
        this->m_bitCount = 0;
    }
}

void HDLC::_processReceivedFrame()
{
    if (this->m_receiveIndex < 2)
    {
        return; // CRCが含まれていない
    }

    // データ部分とCRC部分を分離
    size_t dataLength = this->m_receiveIndex - 2;
    uint8_t receivedCrcLow = this->m_receiveBuffer[this->m_receiveIndex - 2];
    uint8_t receivedCrcHigh = this->m_receiveBuffer[this->m_receiveIndex - 1];
    uint16_t receivedCrc = receivedCrcLow | (receivedCrcHigh << 8);

    // CRC検証
    uint16_t calculatedCrc = calculateCRC16(this->m_receiveBuffer, dataLength);
    bool isValid = (receivedCrc == calculatedCrc);

    // キューに格納 (簡易実装では1つのフレームのみ)
    if (dataLength <= MAX_FRAME_SIZE)
    {
        for (size_t i = 0; i < dataLength; i++)
        {
            this->m_frameQueue.data[i] = this->m_receiveBuffer[i];
        }
        this->m_frameQueue.length = dataLength;
        this->m_frameQueue.valid = isValid;
        this->m_frameQueue.hasData = true;
    }
}

// レスポンス判定ヘルパーメソッド
bool HDLC::_isRRFrame(uint8_t control)
{
    // RRフレーム: ビット0が1、ビット3が0（S形式フレーム）
    return (control & 0x01) == 0x01 && (control & 0x08) == 0x00;
}

bool HDLC::_isREJFrame(uint8_t control)
{
    // REJフレーム: 下位4ビットが0x09（S形式フレーム）
    return (control & 0x0F) == CMD_REJ;
}

uint8_t HDLC::_extractSequenceNumber(uint8_t control)
{
    // シーケンス番号はビット1-3に格納
    return (control >> 1) & 0x07;
}

void HDLC::_transmitByte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--)
    {
        uint8_t bit = (byte >> i) & 1;
        this->_transmitBit(bit);
        this->_waitBitTime();
    }
}

void HDLC::_transmitByteWithStuffing(uint8_t byte, uint8_t &consecutiveOnes)
{
    for (int i = 7; i >= 0; i--)
    {
        uint8_t bit = (byte >> i) & 1;

        // データビットを送信
        this->_transmitBit(bit);
        this->_waitBitTime();

        // ビットスタッフィング処理
        if (bit == 1)
        {
            consecutiveOnes++;
            if (consecutiveOnes == 5)
            {
                // 5個の連続する1の後に0を挿入
                this->_transmitBit(0);
                this->_waitBitTime();
                consecutiveOnes = 0;
            }
        }
        else
        {
            consecutiveOnes = 0;
        }
    }
}
