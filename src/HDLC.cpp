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

HDLC::HDLC(
    RS485Driver &driver)
    : m_driver(driver),
      m_initialized(false),
      m_receiveState(WAITING_FOR_FLAG),
      m_receiveIndex(0),
      m_currentByte(0),
      m_bitCount(0),
      m_consecutiveOnes(0)
{
    this->m_frameQueue.hasData = false;
    this->m_frameQueue.valid = false;
    this->m_frameQueue.length = 0;
}

bool HDLC::begin()
{
    if (this->m_initialized)
    {
        return true;
    }

    // RS485Driverの初期化
    if (!this->m_driver.begin())
    {
        return false;
    }

    this->m_initialized = true;
    return true;
}

bool HDLC::transmitFrame(const uint8_t *data, size_t length)
{
    if (!this->m_initialized || !data || length == 0)
    {
        return false;
    }

    uint8_t frameBits[MAX_FRAME_SIZE * 10]; // ビットスタッフィングを考慮した最大サイズ
    size_t frameBitCount = this->_createFrameBits(data, length, frameBits, sizeof(frameBits));

    if (frameBitCount == 0)
    {
        return false;
    }

    // フレームをビット単位で送信
    return this->m_driver.transmit(frameBits, frameBitCount);
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
        return false;
    }

    // 受信状態を初期化
    this->m_receiveState = WAITING_FOR_FLAG;
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

    // フラグシーケンスを待つ（高頻度でビット読み取り）
    while (!inSync)
    {
        // タイムアウトチェック
        if (timeoutMs > 0 && (this->m_driver.getPinInterface().millis() - startTime) > timeoutMs)
        {
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
                inSync = true;
                // 最初のフラグビット立ち上がり直後は半ビット時間で待機
                this->m_driver.waitHalfBitTime();
                break;
            }
        }

        // 短い間隔で次の読み取りまで待機（通常のビット時間の1/8程度）
        uint32_t shortDelayMicros = (1000000UL / this->m_driver.getBaudRate()) / 8;
        if (bitReadTime < shortDelayMicros)
        {
            this->m_driver.getPinInterface().delayMicroseconds(shortDelayMicros - bitReadTime);
        }
    }

    // フレームデータ受信（同期確立後）
    while (true)
    {
        // タイムアウトチェック
        if (timeoutMs > 0 && (this->m_driver.getPinInterface().millis() - startTime) > timeoutMs)
        {
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
            return true;
        }

        // フラグシーケンス再検出（フレーム終了）
        flagBuffer = (flagBuffer << 1) | bit;
        if (flagBuffer == FLAG_SEQUENCE && this->m_receiveState == RECEIVING_DATA)
        {
            // フレーム終了
            break;
        }

        // 次のビットタイミングまで待機（経過時間を考慮）
        this->m_driver.waitBitTime(bitReadTime);
    }

    return this->m_frameQueue.hasData;
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
        uint8_t high = (highChar >= '0' && highChar <= '9') ? (highChar - '0') : (highChar >= 'A' && highChar <= 'F') ? (highChar - 'A' + 10)
                                                                                                                      : 0;
        uint8_t low = (lowChar >= '0' && lowChar <= '9') ? (lowChar - '0') : (lowChar >= 'A' && lowChar <= 'F') ? (lowChar - 'A' + 10)
                                                                                                                : 0;

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
    switch (this->m_receiveState)
    {
    case WAITING_FOR_FLAG:
        // フラグシーケンス検出
        this->m_currentByte = (this->m_currentByte >> 1) | (bit << 7);
        this->m_bitCount++;

        if (this->m_bitCount == 8)
        {
            if (this->m_currentByte == FLAG_SEQUENCE)
            {
                this->m_receiveState = RECEIVING_DATA;
                this->m_receiveIndex = 0;
                this->m_consecutiveOnes = 0;
                this->m_bitCount = 0;
                this->m_currentByte = 0;
            }
            this->m_currentByte = 0;
            this->m_bitCount = 0;
        }
        break;

    case RECEIVING_DATA:
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
                    this->m_receiveState = WAITING_FOR_FLAG;
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
        break;
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

    size_t bitIndex = 0;
    uint8_t consecutiveOnes = 0;

    for (size_t byteIndex = 0; byteIndex < length; byteIndex++)
    {
        uint8_t currentByte = data[byteIndex];

        for (int bitPos = 7; bitPos >= 0; bitPos--)
        {
            if (bitIndex >= maxBits)
            {
                return 0; // バッファオーバーフロー
            }

            uint8_t bit = (currentByte >> bitPos) & 1;
            stuffedBits[bitIndex] = bit;
            bitIndex++;

            if (bit == 1)
            {
                consecutiveOnes++;
                if (consecutiveOnes == 5)
                {
                    // 連続する5個の1の後に0を挿入
                    if (bitIndex >= maxBits)
                    {
                        return 0; // バッファオーバーフロー
                    }
                    stuffedBits[bitIndex] = 0;
                    bitIndex++;
                    consecutiveOnes = 0;
                }
            }
            else
            {
                consecutiveOnes = 0;
            }
        }
    }

    return bitIndex;
}

size_t HDLC::_bitDestuff(const uint8_t *stuffedBits, size_t bitCount, uint8_t *destuffedData, size_t maxLength)
{
    if (!stuffedBits || bitCount == 0 || !destuffedData || maxLength == 0)
    {
        return 0;
    }

    size_t destuffedBits = 0;
    uint8_t consecutiveOnes = 0;
    uint8_t currentByte = 0;
    int8_t bitPosition = 7; // int8_tに変更
    size_t byteIndex = 0;

    for (size_t i = 0; i < bitCount; i++)
    {
        uint8_t bit = stuffedBits[i];

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
        destuffedBits++;

        if (bitPosition < 0)
        {
            // 1バイト完成
            if (byteIndex >= maxLength)
            {
                return 0; // バッファオーバーフロー
            }
            destuffedData[byteIndex++] = currentByte;
            currentByte = 0;
            bitPosition = 7;
        }
    }

    // 最後の不完全なバイトがある場合は処理しない
    return byteIndex;
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

    // ビットスタッフィング
    uint8_t stuffedBits[MAX_FRAME_SIZE * 10]; // 最大拡張を考慮
    size_t stuffedBitCount = this->_bitStuff(tempFrame, tempIndex, stuffedBits, sizeof(stuffedBits));

    if (stuffedBitCount == 0)
    {
        return 0;
    }

    // フラグシーケンス（開始）
    size_t frameIndex = 0;
    uint8_t flagBits[] = {0, 1, 1, 1, 1, 1, 1, 0}; // 0x7E

    // 開始フラグ
    if (frameIndex + 8 > maxBits)
        return 0;
    memcpy(frameBits + frameIndex, flagBits, 8);
    frameIndex += 8;

    // スタッフィング済みデータ
    if (frameIndex + stuffedBitCount > maxBits)
        return 0;
    memcpy(frameBits + frameIndex, stuffedBits, stuffedBitCount);
    frameIndex += stuffedBitCount;

    // 終了フラグ
    if (frameIndex + 8 > maxBits)
        return 0;
    memcpy(frameBits + frameIndex, flagBits, 8);
    frameIndex += 8;

    return frameIndex;
}
