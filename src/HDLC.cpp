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

HDLC::HDLC(RS485Driver &driver)
    : m_driver(driver), m_initialized(false), m_receiveState(WAITING_FOR_FLAG), m_receiveIndex(0), m_currentByte(0), m_bitCount(0), m_consecutiveOnes(0), m_receiveCallback(nullptr)
{
    m_frameQueue.hasData = false;
    m_frameQueue.valid = false;
    m_frameQueue.length = 0;
}

bool HDLC::begin()
{
    if (m_initialized)
    {
        return true;
    }

    // RS485Driverの初期化
    if (!m_driver.begin())
    {
        return false;
    }

    m_initialized = true;
    return true;
}

bool HDLC::transmitFrame(const uint8_t *data, size_t length)
{
    if (!m_initialized || !data || length == 0)
    {
        return false;
    }

    uint8_t frameBits[MAX_FRAME_SIZE * 10]; // ビットスタッフィングを考慮した最大サイズ
    size_t frameBitCount = createFrameBits(data, length, frameBits, sizeof(frameBits));

    if (frameBitCount == 0)
    {
        return false;
    }

    // フレームをビット単位で送信
    return m_driver.transmit(frameBits, frameBitCount);
}

bool HDLC::transmitHexString(const String &hexString)
{
    uint8_t buffer[MAX_FRAME_SIZE / 2]; // 16進数文字列の場合、バイト数は文字数の半分
    size_t length = hexStringToBytes(hexString, buffer, sizeof(buffer));

    if (length == 0)
    {
        return false;
    }

    return transmitFrame(buffer, length);
}

void HDLC::setReceiveCallback(FrameReceivedCallback callback)
{
    m_receiveCallback = callback;
}

void HDLC::startReceive()
{
    if (!m_initialized)
    {
        return;
    }

    m_receiveState = WAITING_FOR_FLAG;
    m_receiveIndex = 0;
    m_currentByte = 0;
    m_bitCount = 0;
    m_consecutiveOnes = 0;

    // ポーリングベースの実装では特に何もしない
    // 実際の受信は receiveFrame() で行う
}

void HDLC::stopReceive()
{
    // ポーリングベースの実装では特に何もしない
}

bool HDLC::receiveFrame(uint32_t timeoutMs)
{
    if (!m_initialized)
    {
        return false;
    }

    // 受信状態を初期化
    m_receiveState = WAITING_FOR_FLAG;
    m_receiveIndex = 0;
    m_currentByte = 0;
    m_bitCount = 0;
    m_consecutiveOnes = 0;
    m_frameQueue.hasData = false;

    // バッファサイズはフレーム最大長の8倍（ビット単位）
    const size_t maxBits = MAX_FRAME_SIZE * 8;
    uint8_t rawBuffer[MAX_FRAME_SIZE];
    memset(rawBuffer, 0, sizeof(rawBuffer));

    // RS485Driverからデータを受信
    size_t bitsReceived = m_driver.read(rawBuffer, maxBits, timeoutMs);
    if (bitsReceived == 0)
    {
        return false; // タイムアウトまたはエラー
    }

    // 受信したビットをHDLCフレームとして処理
    for (size_t i = 0; i < bitsReceived; i++)
    {
        size_t byteIndex = i / 8;
        size_t bitIndex = 7 - (i % 8);
        uint8_t bit = (rawBuffer[byteIndex] >> bitIndex) & 1;

        processBit(bit);

        // フレーム受信完了チェック
        if (m_frameQueue.hasData)
        {
            return true;
        }
    }

    return m_frameQueue.hasData;
}

size_t HDLC::readFrame(uint8_t *buffer, size_t bufferSize)
{
    if (!m_frameQueue.hasData || !buffer || bufferSize == 0)
    {
        return 0;
    }

    size_t copyLength = (m_frameQueue.length < bufferSize) ? m_frameQueue.length : bufferSize;
    memcpy(buffer, m_frameQueue.data, copyLength);

    m_frameQueue.hasData = false;
    return copyLength;
}

String HDLC::readFrameAsHexString()
{
    if (!m_frameQueue.hasData)
    {
        return "";
    }

    String result = bytesToHexString(m_frameQueue.data, m_frameQueue.length);
    m_frameQueue.hasData = false;
    return result;
}

uint16_t HDLC::calculateCRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001; // CRC-16-IBM polynomial
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

size_t HDLC::stuffData(const uint8_t *data, size_t length, uint8_t *stuffedData, size_t maxLength)
{
    size_t stuffedIndex = 0;

    for (size_t i = 0; i < length && stuffedIndex < maxLength - 1; i++)
    {
        if (data[i] == FLAG_SEQUENCE || data[i] == ESCAPE_CHAR)
        {
            stuffedData[stuffedIndex++] = ESCAPE_CHAR;
            if (stuffedIndex < maxLength)
            {
                stuffedData[stuffedIndex++] = data[i] ^ 0x20;
            }
        }
        else
        {
            stuffedData[stuffedIndex++] = data[i];
        }
    }

    return stuffedIndex;
}

size_t HDLC::destuffData(const uint8_t *stuffedData, size_t length, uint8_t *destuffedData, size_t maxLength)
{
    size_t destuffedIndex = 0;
    bool escapeNext = false;

    for (size_t i = 0; i < length && destuffedIndex < maxLength; i++)
    {
        if (escapeNext)
        {
            destuffedData[destuffedIndex++] = stuffedData[i] ^ 0x20;
            escapeNext = false;
        }
        else if (stuffedData[i] == ESCAPE_CHAR)
        {
            escapeNext = true;
        }
        else
        {
            destuffedData[destuffedIndex++] = stuffedData[i];
        }
    }

    return destuffedIndex;
}

size_t HDLC::createFrame(const uint8_t *data, size_t length, uint8_t *frame, size_t maxFrameLength)
{
    if (length + 4 > maxFrameLength)
    { // データ + CRC(2) + フラグ(2)
        return 0;
    }

    size_t frameIndex = 0;

    // 開始フラグ
    frame[frameIndex++] = FLAG_SEQUENCE;

    // データのスタッフィング
    uint8_t stuffedData[MAX_FRAME_SIZE];
    size_t stuffedLength = stuffData(data, length, stuffedData, MAX_FRAME_SIZE - 4);

    // CRC計算
    uint16_t crc = calculateCRC16(data, length);
    uint8_t crcBytes[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};

    // CRCもスタッフィング
    uint8_t stuffedCrc[4];
    size_t stuffedCrcLength = stuffData(crcBytes, 2, stuffedCrc, 4);

    // スタッフィングされたデータをフレームにコピー
    if (frameIndex + stuffedLength + stuffedCrcLength + 1 <= maxFrameLength)
    {
        memcpy(&frame[frameIndex], stuffedData, stuffedLength);
        frameIndex += stuffedLength;

        memcpy(&frame[frameIndex], stuffedCrc, stuffedCrcLength);
        frameIndex += stuffedCrcLength;

        // 終了フラグ
        frame[frameIndex++] = FLAG_SEQUENCE;

        return frameIndex;
    }

    return 0;
}

size_t HDLC::hexStringToBytes(const String &hexString, uint8_t *buffer, size_t maxLength)
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

String HDLC::bytesToHexString(const uint8_t *data, size_t length)
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

void HDLC::processBit(uint8_t bit)
{
    switch (m_receiveState)
    {
    case WAITING_FOR_FLAG:
        // フラグシーケンス検出
        m_currentByte = (m_currentByte >> 1) | (bit << 7);
        m_bitCount++;

        if (m_bitCount == 8)
        {
            if (m_currentByte == FLAG_SEQUENCE)
            {
                m_receiveState = RECEIVING_DATA;
                m_receiveIndex = 0;
                m_consecutiveOnes = 0;
                m_bitCount = 0;
                m_currentByte = 0;
            }
            m_currentByte = 0;
            m_bitCount = 0;
        }
        break;

    case RECEIVING_DATA:
        // ビットデスタッフィング処理
        if (bit == 1)
        {
            m_consecutiveOnes++;
        }
        else
        {
            if (m_consecutiveOnes == 5)
            {
                // スタッフィングされた0ビット - 無視
                m_consecutiveOnes = 0;
                return;
            }
            else if (m_consecutiveOnes == 6)
            {
                // フラグシーケンスの可能性
                m_currentByte = (m_currentByte >> 1) | (bit << 7);
                m_bitCount++;

                if (m_bitCount == 8 && m_currentByte == FLAG_SEQUENCE)
                {
                    // フレーム終了
                    processReceivedFrame();
                    m_receiveState = WAITING_FOR_FLAG;
                    m_consecutiveOnes = 0;
                    m_bitCount = 0;
                    m_currentByte = 0;
                    return;
                }
            }
            m_consecutiveOnes = 0;
        }

        // 通常のデータビット処理
        m_currentByte = (m_currentByte >> 1) | (bit << 7);
        m_bitCount++;

        if (m_bitCount == 8)
        {
            // 1バイト完了
            if (m_receiveIndex < MAX_FRAME_SIZE)
            {
                m_receiveBuffer[m_receiveIndex++] = m_currentByte;
            }
            m_currentByte = 0;
            m_bitCount = 0;
        }
        break;

    case ESCAPE_NEXT:
        // 現在は使用しない（ビットスタッフィングで対応）
        break;
    }
}

void HDLC::processReceivedFrame()
{
    if (m_receiveIndex < 2)
    {
        return; // CRCが含まれていない
    }

    // データ部分とCRC部分を分離
    size_t dataLength = m_receiveIndex - 2;
    uint8_t receivedCrcLow = m_receiveBuffer[m_receiveIndex - 2];
    uint8_t receivedCrcHigh = m_receiveBuffer[m_receiveIndex - 1];
    uint16_t receivedCrc = receivedCrcLow | (receivedCrcHigh << 8);

    // CRC検証
    uint16_t calculatedCrc = calculateCRC16(m_receiveBuffer, dataLength);
    bool isValid = (receivedCrc == calculatedCrc);

    // コールバック呼び出しまたはキューに格納
    if (m_receiveCallback)
    {
        m_receiveCallback(m_receiveBuffer, dataLength, isValid);
    }
    else
    {
        // キューに格納 (簡易実装では1つのフレームのみ)
        if (dataLength <= MAX_FRAME_SIZE)
        {
            memcpy(m_frameQueue.data, m_receiveBuffer, dataLength);
            m_frameQueue.length = dataLength;
            m_frameQueue.valid = isValid;
            m_frameQueue.hasData = true;
        }
    }
}

size_t HDLC::bitStuff(const uint8_t *data, size_t length, uint8_t *stuffedBits, size_t maxBits)
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

size_t HDLC::bitDestuff(const uint8_t *stuffedBits, size_t bitCount, uint8_t *destuffedData, size_t maxLength)
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

size_t HDLC::createFrameBits(const uint8_t *data, size_t length, uint8_t *frameBits, size_t maxBits)
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
    size_t stuffedBitCount = bitStuff(tempFrame, tempIndex, stuffedBits, sizeof(stuffedBits));

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
