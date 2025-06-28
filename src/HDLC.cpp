#include "HDLC.h"

#ifdef NATIVE_TEST
// ネイティブテスト環境用のString互換関数
#include <algorithm>
#include <cctype>
#include <sstream>
#include <iomanip>

namespace
{
    void replace_all(std::string &str, const std::string &from, const std::string &to)
    {
        size_t start_pos = 0;
        while ((start_pos = str.find(from, start_pos)) != std::string::npos)
        {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length();
        }
    }

    void to_upper_case(std::string &str)
    {
        std::transform(str.begin(), str.end(), str.begin(),
                       [](unsigned char c)
                       { return std::toupper(c); });
    }

    char char_at(const std::string &str, size_t index)
    {
        return (index < str.length()) ? str[index] : '\0';
    }

    std::string int_to_hex(int value)
    {
        std::stringstream ss;
        ss << std::hex << std::uppercase << value;
        return ss.str();
    }
}
#endif

// 静的メンバは不要になったので削除

/**
 * @brief 16進数文字を数値に変換するユーティリティ関数
 * @param hexChar 16進数文字 ('0'-'9', 'A'-'F', 'a'-'f')
 * @return 対応する数値 (0-15)、無効な文字の場合は0
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
        return 0; // 無効な文字
    }
}

HDLC::HDLC(
    RS485Driver &driver)
    : m_driver(driver),
      m_initialized(false),
      m_receiveIndex(0),
      m_currentByte(0),
      m_bitCount(0),
      m_consecutiveOnes(0)
{
    this->m_frameQueue.hasData = false;
    this->m_frameQueue.valid = false;
    this->m_frameQueue.length = 0;

    // 待機時間を事前計算
    uint32_t baudRate = this->m_driver.getBaudRate();
    this->m_shortDelayMicros = (1000000UL / baudRate) / 8; // 1/8ビット時間
}

bool HDLC::begin()
{
#ifndef NATIVE_TEST
    Serial.println("HDLC: Begin init");
#endif

    if (this->m_initialized)
    {
#ifndef NATIVE_TEST
        Serial.println("HDLC: Already init");
#endif
        return true;
    }

    // RS485Driverの初期化
    if (!this->m_driver.begin())
    {
#ifndef NATIVE_TEST
        Serial.println("HDLC: Driver init failed");
#endif
        return false;
    }

    this->m_initialized = true;
#ifndef NATIVE_TEST
    Serial.println("HDLC: Init OK");
#endif
    return true;
}

bool HDLC::transmitFrame(const uint8_t *data, size_t length)
{
    if (!this->m_initialized || !data || length == 0)
    {
#ifndef NATIVE_TEST
        Serial.println("HDLC: TX invalid params");
#endif
        return false;
    }

#ifndef NATIVE_TEST
    Serial.print("HDLC: TX frame, len=");
    Serial.println(length);
#endif

    uint8_t frameBuffer[MAX_FRAME_SIZE * 2]; // ビットスタッフィングを考慮
    size_t frameBitCount = this->_createFrameBits(data, length, frameBuffer, sizeof(frameBuffer) * 8);

    if (frameBitCount == 0)
    {
#ifndef NATIVE_TEST
        Serial.println("HDLC: Frame creation failed");
#endif
        return false;
    }

#ifndef NATIVE_TEST
    Serial.print("HDLC: Frame bits=");
    Serial.println(frameBitCount);
#endif

    // フレームをビット単位で送信（バイト配列として効率的にパッキングされている）
    // RS485Driver::transmitはビット数を期待するので、frameBitCountを直接渡す
    bool result = this->m_driver.transmit(frameBuffer, frameBitCount);

#ifndef NATIVE_TEST
    Serial.print("HDLC: TX result=");
    Serial.println(result ? "OK" : "FAIL");
#endif

    return result;
}

bool HDLC::transmitHexString(const String &hexString)
{
    uint8_t buffer[MAX_FRAME_SIZE / 2]; // 16進数文字列の場合、バイト数は文字数の半分
    size_t length = this->_hexStringToBytes(hexString, buffer, sizeof(buffer));

    if (length == 0)
    {
        return false;
    }

    return this->transmitFrame(buffer, length);
}

bool HDLC::receiveFrameWithBitControl(uint32_t timeoutMs)
{
    if (!this->m_initialized)
    {
#ifndef NATIVE_TEST
        Serial.println("HDLC: RX not init");
#endif
        return false;
    }

#ifndef NATIVE_TEST
    // ログ出力が多いためコメントアウト
    // Serial.print("HDLC: RX start, timeout=");
    // Serial.println(timeoutMs);
#endif

    // 受信状態を初期化
    this->m_receiveIndex = 0;
    this->m_currentByte = 0;
    this->m_bitCount = 0;
    this->m_consecutiveOnes = 0;
    this->m_frameQueue.hasData = false;

    uint32_t startTime = this->m_driver.getPinInterface().millis();
    uint8_t flagBuffer = 0; // フラグシーケンス検出用
    size_t flagBitCount = 0;
    const uint8_t FLAG_SEQUENCE = 0x7E; // 01111110
    bool inSync = false;

#ifndef NATIVE_TEST
    // ログ出力が多いためコメントアウト
    // Serial.println("HDLC: Waiting for flag");
#endif

    // フラグシーケンスを待つ（高頻度でビット読み取り）
    while (!inSync)
    {
        // タイムアウトチェック
        if (timeoutMs > 0 && (this->m_driver.getPinInterface().millis() - startTime) > timeoutMs)
        {
#ifndef NATIVE_TEST
            // ログ出力が多いためコメントアウト
            // Serial.println("HDLC: Flag timeout");
#endif
            return false;
        }

        // 高頻度でビット読み取り（同期を取るため）
        uint32_t bitStartTime = this->m_driver.getPinInterface().micros();
        uint8_t bit = this->m_driver.readBit();
        uint32_t bitReadTime = this->m_driver.getPinInterface().micros() - bitStartTime;

        // フラグシーケンス検出のためビットを左シフトして追加
        flagBuffer = (flagBuffer << 1) | bit;
        flagBitCount++;

        if (flagBitCount >= 8)
        {
            if (flagBuffer == FLAG_SEQUENCE)
            {
                // フラグシーケンス検出！同期確立
#ifndef NATIVE_TEST
                Serial.println("HDLC: Flag found, sync OK");
#endif
                inSync = true;
                // 最初のフラグビット立ち上がり直後は半ビット時間で待機
                this->m_driver.waitHalfBitTime();
                break;
            }
        }

        // 短い間隔で次の読み取りまで待機（事前計算された1/8ビット時間）
        if (bitReadTime < this->m_shortDelayMicros)
        {
            this->m_driver.getPinInterface().delayMicroseconds(this->m_shortDelayMicros - bitReadTime);
        }
    }

#ifndef NATIVE_TEST
    Serial.println("HDLC: Reading frame data");
#endif

    // フレームデータ受信（同期確立後）
    while (true)
    {
        // タイムアウトチェック
        if (timeoutMs > 0 && (this->m_driver.getPinInterface().millis() - startTime) > timeoutMs)
        {
#ifndef NATIVE_TEST
            Serial.println("HDLC: Frame timeout");
#endif
            return false;
        }

        // 1ビット読み取り（時間測定付き）
        uint32_t bitStartTime = this->m_driver.getPinInterface().micros();
        uint8_t bit = this->m_driver.readBit();
        uint32_t bitReadTime = this->m_driver.getPinInterface().micros() - bitStartTime;

        // HDLCフレーム処理
        this->_processBit(bit);

        // フレーム受信完了チェック
        if (this->m_frameQueue.hasData)
        {
#ifndef NATIVE_TEST
            Serial.print("HDLC: Frame RX OK, len=");
            Serial.print(this->m_frameQueue.length);
            Serial.print(" valid=");
            Serial.println(this->m_frameQueue.valid ? "Y" : "N");
#endif
            return true;
        }

        // フラグシーケンス再検出（フレーム終了）
        flagBuffer = (flagBuffer << 1) | bit;
        if (flagBuffer == FLAG_SEQUENCE)
        {
            // フレーム終了
#ifndef NATIVE_TEST
            Serial.println("HDLC: End flag found");
#endif
            break;
        }

        // 次のビットタイミングまで待機（経過時間を考慮）
        this->m_driver.waitBitTime(bitReadTime);
    }

    bool result = this->m_frameQueue.hasData;
#ifndef NATIVE_TEST
    Serial.print("HDLC: RX result=");
    Serial.println(result ? "OK" : "FAIL");
#endif
    return result;
}

size_t HDLC::readFrame(uint8_t *buffer, size_t bufferSize)
{
    if (!this->m_frameQueue.hasData || !buffer || bufferSize == 0)
    {
        return 0;
    }

    size_t copyLength = (this->m_frameQueue.length < bufferSize) ? this->m_frameQueue.length : bufferSize;
    memcpy(buffer, this->m_frameQueue.data, copyLength);

    this->m_frameQueue.hasData = false;
    return copyLength;
}

String HDLC::readFrameAsHexString()
{
    if (!this->m_frameQueue.hasData)
    {
        return "";
    }

    String result = this->_bytesToHexString(this->m_frameQueue.data, this->m_frameQueue.length);
    this->m_frameQueue.hasData = false;
    return result;
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

size_t HDLC::_hexStringToBytes(const String &hexString, uint8_t *buffer, size_t maxLength)
{
    String cleanHex = hexString;

#ifdef NATIVE_TEST
    replace_all(cleanHex, " ", ""); // スペースを除去
    to_upper_case(cleanHex);
#else
    cleanHex.replace(" ", ""); // スペースを除去
    cleanHex.toUpperCase();
#endif

    if (cleanHex.length() % 2 != 0)
    {
        return 0; // 奇数文字は無効
    }

    size_t byteCount = cleanHex.length() / 2;
    if (byteCount > maxLength)
    {
        byteCount = maxLength;
    }

    for (size_t i = 0; i < byteCount; i++)
    {
#ifdef NATIVE_TEST
        char highChar = char_at(cleanHex, i * 2);
        char lowChar = char_at(cleanHex, i * 2 + 1);
#else
        char highChar = cleanHex.charAt(i * 2);
        char lowChar = cleanHex.charAt(i * 2 + 1);
#endif

        // 16進数文字を数値に変換
        uint8_t high = hexCharToValue(highChar);
        uint8_t low = hexCharToValue(lowChar);

        buffer[i] = (high << 4) | low;
    }

    return byteCount;
}

String HDLC::_bytesToHexString(const uint8_t *data, size_t length)
{
    String result = "";
    for (size_t i = 0; i < length; i++)
    {
        if (data[i] < 0x10)
        {
            result += "0";
        }
#ifdef NATIVE_TEST
        result += int_to_hex(data[i]);
#else
        result += String(data[i], HEX);
#endif
        if (i < length - 1)
        {
            result += " ";
        }
    }
#ifdef NATIVE_TEST
    to_upper_case(result);
#else
    result.toUpperCase();
#endif
    return result;
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
        memcpy(this->m_frameQueue.data, this->m_receiveBuffer, dataLength);
        this->m_frameQueue.length = dataLength;
        this->m_frameQueue.valid = isValid;
        this->m_frameQueue.hasData = true;
    }
}

size_t HDLC::_bitStuff(const uint8_t *data, size_t length, uint8_t *stuffedBits, size_t maxBits)
{
    if (!data || length == 0 || !stuffedBits || maxBits == 0)
    {
        return 0;
    }

    size_t outputBitIndex = 0;
    uint8_t consecutiveOnes = 0;

    // 出力バッファを初期化
    size_t maxBytes = (maxBits + 7) / 8;
    for (size_t i = 0; i < maxBytes; i++)
    {
        stuffedBits[i] = 0;
    }

    // 入力データをビット単位で処理
    for (size_t byteIdx = 0; byteIdx < length; byteIdx++)
    {
        for (int bitIdx = 7; bitIdx >= 0; bitIdx--)
        {
            uint8_t bit = (data[byteIdx] >> bitIdx) & 1;

            // 出力バッファの容量チェック
            if (outputBitIndex >= maxBits)
            {
                return 0; // バッファ不足
            }

            // ビットを出力バッファに書き込み
            this->_writeBitToBuffer(stuffedBits, outputBitIndex, bit);
            outputBitIndex++;

            if (bit == 1)
            {
                consecutiveOnes++;
                if (consecutiveOnes == 5)
                {
                    // 5個の連続する1の後に0を挿入
                    if (outputBitIndex >= maxBits)
                    {
                        return 0; // バッファ不足
                    }
                    this->_writeBitToBuffer(stuffedBits, outputBitIndex, 0);
                    outputBitIndex++;
                    consecutiveOnes = 0;
                }
            }
            else
            {
                consecutiveOnes = 0;
            }
        }
    }

    // 実際のビット数を返す
    return outputBitIndex;
}

size_t HDLC::_bitDestuff(const uint8_t *stuffedBytes, size_t bitCount, uint8_t *destuffedData, size_t maxLength)
{
    if (!stuffedBytes || bitCount == 0 || !destuffedData || maxLength == 0)
    {
        return 0;
    }

    uint8_t consecutiveOnes = 0;
    uint8_t currentByte = 0;
    int8_t bitPosition = 7;
    size_t outputByteIndex = 0;

    // 入力バイト配列をビット単位で処理（bitCountまで）
    for (size_t bitIdx = 0; bitIdx < bitCount; bitIdx++)
    {
        // ビットインデックスからバイトインデックスとビット位置を計算
        size_t byteIdx = bitIdx / 8;
        int bitPos = 7 - (bitIdx % 8);
        uint8_t bit = (stuffedBytes[byteIdx] >> bitPos) & 1;

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
            if (outputByteIndex >= maxLength)
            {
                return 0; // バッファオーバーフロー
            }
            destuffedData[outputByteIndex++] = currentByte;
            currentByte = 0;
            bitPosition = 7;
        }
    }

    // 最後の不完全なバイトがある場合は処理しない（ビット境界でのみ完成したバイトを返す）
    return outputByteIndex;
}

void HDLC::_writeBitToBuffer(uint8_t *buffer, size_t bitIndex, uint8_t bit)
{
    size_t byteIndex = bitIndex / 8;
    size_t bitPos = 7 - (bitIndex % 8); // MSBファースト

    if (bit)
    {
        buffer[byteIndex] |= (1 << bitPos);
    }
    else
    {
        buffer[byteIndex] &= ~(1 << bitPos);
    }
}

void HDLC::_writeBitsToBuffer(uint8_t *buffer, size_t bitIndex, uint8_t value, size_t numBits)
{
    for (size_t i = 0; i < numBits; i++)
    {
        uint8_t bit = (value >> (numBits - 1 - i)) & 1;
        this->_writeBitToBuffer(buffer, bitIndex + i, bit);
    }
}

size_t HDLC::_createFrameBits(const uint8_t *data, size_t length, uint8_t *frameBits, size_t maxBits)
{
    if (!data || length == 0 || !frameBits || maxBits < 32)
    { // 最小フレーム長チェック
        return 0;
    }

    // CRC計算
    uint16_t crc = calculateCRC16(data, length);

    // フレーム構築用の一時バッファ
    uint8_t tempFrame[MAX_FRAME_SIZE];
    size_t tempIndex = 0;

    // ペイロード + CRC
    if (tempIndex + length + 2 > MAX_FRAME_SIZE)
    {
        return 0;
    }

    memcpy(tempFrame + tempIndex, data, length);
    tempIndex += length;

    // CRC追加（リトルエンディアン）
    tempFrame[tempIndex++] = crc & 0xFF;
    tempFrame[tempIndex++] = (crc >> 8) & 0xFF;

    // ビットスタッフィング用の一時バッファ
    uint8_t stuffedData[MAX_FRAME_SIZE * 2]; // 最悪ケースを考慮
    size_t stuffedBitCount = this->_bitStuff(tempFrame, tempIndex, stuffedData, sizeof(stuffedData) * 8);

    if (stuffedBitCount == 0)
    {
        return 0;
    }

    // フレーム構造: FLAG + スタッフィング済みデータ + FLAG
    // ビット単位で計算
    size_t totalBits = 8 + stuffedBitCount + 8; // FLAG(8bit) + データ + FLAG(8bit)

    if (totalBits > maxBits)
    {
        return 0; // バッファサイズ不足
    }

    // 出力バッファを初期化
    size_t maxBytes = (maxBits + 7) / 8;
    for (size_t i = 0; i < maxBytes; i++)
    {
        frameBits[i] = 0;
    }

    size_t bitIndex = 0;

    // 開始フラグ (0x7E = 01111110)
    this->_writeBitsToBuffer(frameBits, bitIndex, HDLC::FLAG_SEQUENCE, 8);
    bitIndex += 8;

    // スタッフィング済みデータをビット単位で書き込み
    for (size_t i = 0; i < stuffedBitCount; i++)
    {
        // ビットインデックスからバイトインデックスとビット位置を計算
        size_t srcByteIdx = i / 8;
        int srcBitPos = 7 - (i % 8);
        uint8_t bit = (stuffedData[srcByteIdx] >> srcBitPos) & 1;
        this->_writeBitToBuffer(frameBits, bitIndex, bit);
        bitIndex++;
    }

    // 終了フラグ (0x7E = 01111110)
    this->_writeBitsToBuffer(frameBits, bitIndex, HDLC::FLAG_SEQUENCE, 8);
    bitIndex += 8;

    return bitIndex; // 総ビット数を返す
}
