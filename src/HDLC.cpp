#include "HDLC.h"

#ifdef NATIVE_TEST
// ネイティブテスト環境用のString互換関数
#include <algorithm>
#include <cctype>
#include <sstream>
#include <iomanip>

namespace {
    void replace_all(std::string& str, const std::string& from, const std::string& to) {
        size_t start_pos = 0;
        while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length();
        }
    }
    
    void to_upper_case(std::string& str) {
        std::transform(str.begin(), str.end(), str.begin(), 
                      [](unsigned char c) { return std::toupper(c); });
    }
    
    char char_at(const std::string& str, size_t index) {
        return (index < str.length()) ? str[index] : '\0';
    }
    
    std::string int_to_hex(int value) {
        std::stringstream ss;
        ss << std::hex << std::uppercase << value;
        return ss.str();
    }
}
#endif

// 静的メンバの初期化
HDLC* HDLC::s_instance = nullptr;

HDLC::HDLC(RS485Driver& driver)
    : m_driver(driver)
    , m_initialized(false)
    , m_receiveState(WAITING_FOR_FLAG)
    , m_receiveIndex(0)
    , m_currentByte(0)
    , m_bitCount(0)
    , m_receiveCallback(nullptr)
{
    s_instance = this;
    m_frameQueue.hasData = false;
    m_frameQueue.valid = false;
    m_frameQueue.length = 0;
}

bool HDLC::begin() {
    if (m_initialized) {
        return true;
    }

    // RS485Driverの初期化
    if (!m_driver.begin()) {
        return false;
    }

    // 受信コールバックの設定
    m_driver.setReceiveCallback(bitReceivedCallback);

    m_initialized = true;
    return true;
}

bool HDLC::transmitFrame(const uint8_t* data, size_t length) {
    if (!m_initialized || !data || length == 0) {
        return false;
    }

    uint8_t frame[MAX_FRAME_SIZE];
    size_t frameLength = createFrame(data, length, frame, MAX_FRAME_SIZE);
    
    if (frameLength == 0) {
        return false;
    }

    // フレームをビット単位で送信
    return m_driver.transmit(frame, frameLength * 8);
}

bool HDLC::transmitHexString(const String& hexString) {
    uint8_t buffer[MAX_FRAME_SIZE / 2]; // 16進数文字列の場合、バイト数は文字数の半分
    size_t length = hexStringToBytes(hexString, buffer, sizeof(buffer));
    
    if (length == 0) {
        return false;
    }

    return transmitFrame(buffer, length);
}

void HDLC::setReceiveCallback(FrameReceivedCallback callback) {
    m_receiveCallback = callback;
}

void HDLC::startReceive() {
    if (!m_initialized) {
        return;
    }

    m_receiveState = WAITING_FOR_FLAG;
    m_receiveIndex = 0;
    m_currentByte = 0;
    m_bitCount = 0;
    
    m_driver.startReceive();
}

void HDLC::stopReceive() {
    m_driver.stopReceive();
}

size_t HDLC::readFrame(uint8_t* buffer, size_t bufferSize) {
    if (!m_frameQueue.hasData || !buffer || bufferSize == 0) {
        return 0;
    }

    size_t copyLength = (m_frameQueue.length < bufferSize) ? m_frameQueue.length : bufferSize;
    memcpy(buffer, m_frameQueue.data, copyLength);
    
    m_frameQueue.hasData = false;
    return copyLength;
}

String HDLC::readFrameAsHexString() {
    if (!m_frameQueue.hasData) {
        return "";
    }

    String result = bytesToHexString(m_frameQueue.data, m_frameQueue.length);
    m_frameQueue.hasData = false;
    return result;
}

uint16_t HDLC::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // CRC-16-IBM polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

size_t HDLC::stuffData(const uint8_t* data, size_t length, uint8_t* stuffedData, size_t maxLength) {
    size_t stuffedIndex = 0;
    
    for (size_t i = 0; i < length && stuffedIndex < maxLength - 1; i++) {
        if (data[i] == FLAG_SEQUENCE || data[i] == ESCAPE_CHAR) {
            stuffedData[stuffedIndex++] = ESCAPE_CHAR;
            if (stuffedIndex < maxLength) {
                stuffedData[stuffedIndex++] = data[i] ^ 0x20;
            }
        } else {
            stuffedData[stuffedIndex++] = data[i];
        }
    }
    
    return stuffedIndex;
}

size_t HDLC::destuffData(const uint8_t* stuffedData, size_t length, uint8_t* destuffedData, size_t maxLength) {
    size_t destuffedIndex = 0;
    bool escapeNext = false;
    
    for (size_t i = 0; i < length && destuffedIndex < maxLength; i++) {
        if (escapeNext) {
            destuffedData[destuffedIndex++] = stuffedData[i] ^ 0x20;
            escapeNext = false;
        } else if (stuffedData[i] == ESCAPE_CHAR) {
            escapeNext = true;
        } else {
            destuffedData[destuffedIndex++] = stuffedData[i];
        }
    }
    
    return destuffedIndex;
}

size_t HDLC::createFrame(const uint8_t* data, size_t length, uint8_t* frame, size_t maxFrameLength) {
    if (length + 4 > maxFrameLength) { // データ + CRC(2) + フラグ(2)
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
    if (frameIndex + stuffedLength + stuffedCrcLength + 1 <= maxFrameLength) {
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

size_t HDLC::hexStringToBytes(const String& hexString, uint8_t* buffer, size_t maxLength) {
    String cleanHex = hexString;
    
#ifdef NATIVE_TEST
    replace_all(cleanHex, " ", ""); // スペースを除去
    to_upper_case(cleanHex);
#else
    cleanHex.replace(" ", ""); // スペースを除去
    cleanHex.toUpperCase();
#endif
    
    if (cleanHex.length() % 2 != 0) {
        return 0; // 奇数文字は無効
    }
    
    size_t byteCount = cleanHex.length() / 2;
    if (byteCount > maxLength) {
        byteCount = maxLength;
    }
    
    for (size_t i = 0; i < byteCount; i++) {
#ifdef NATIVE_TEST
        char highChar = char_at(cleanHex, i * 2);
        char lowChar = char_at(cleanHex, i * 2 + 1);
#else
        char highChar = cleanHex.charAt(i * 2);
        char lowChar = cleanHex.charAt(i * 2 + 1);
#endif
        
        // 16進数文字を数値に変換
        uint8_t high = (highChar >= '0' && highChar <= '9') ? (highChar - '0') :
                      (highChar >= 'A' && highChar <= 'F') ? (highChar - 'A' + 10) : 0;
        uint8_t low = (lowChar >= '0' && lowChar <= '9') ? (lowChar - '0') :
                     (lowChar >= 'A' && lowChar <= 'F') ? (lowChar - 'A' + 10) : 0;
        
        buffer[i] = (high << 4) | low;
    }
    
    return byteCount;
}

String HDLC::bytesToHexString(const uint8_t* data, size_t length) {
    String result = "";
    for (size_t i = 0; i < length; i++) {
        if (data[i] < 0x10) {
            result += "0";
        }
#ifdef NATIVE_TEST
        result += int_to_hex(data[i]);
#else
        result += String(data[i], HEX);
#endif
        if (i < length - 1) {
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

void HDLC::bitReceivedCallback(uint8_t bit) {
    if (s_instance) {
        s_instance->processBit(bit);
    }
}

void HDLC::processBit(uint8_t bit) {
    // 現在のバイトに新しいビットを追加
    m_currentByte = (m_currentByte >> 1) | (bit << 7);
    m_bitCount++;
    
    if (m_bitCount == 8) {
        // 1バイト完了
        switch (m_receiveState) {
            case WAITING_FOR_FLAG:
                if (m_currentByte == FLAG_SEQUENCE) {
                    m_receiveState = RECEIVING_DATA;
                    m_receiveIndex = 0;
                }
                break;
                
            case RECEIVING_DATA:
                if (m_currentByte == FLAG_SEQUENCE) {
                    // フレーム終了
                    processReceivedFrame();
                    m_receiveState = WAITING_FOR_FLAG;
                } else if (m_currentByte == ESCAPE_CHAR) {
                    m_receiveState = ESCAPE_NEXT;
                } else {
                    if (m_receiveIndex < MAX_FRAME_SIZE) {
                        m_receiveBuffer[m_receiveIndex++] = m_currentByte;
                    }
                }
                break;
                
            case ESCAPE_NEXT:
                if (m_receiveIndex < MAX_FRAME_SIZE) {
                    m_receiveBuffer[m_receiveIndex++] = m_currentByte ^ 0x20;
                }
                m_receiveState = RECEIVING_DATA;
                break;
        }
        
        m_currentByte = 0;
        m_bitCount = 0;
    }
}

void HDLC::processReceivedFrame() {
    if (m_receiveIndex < 2) {
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
    if (m_receiveCallback) {
        m_receiveCallback(m_receiveBuffer, dataLength, isValid);
    } else {
        // キューに格納 (簡易実装では1つのフレームのみ)
        if (dataLength <= MAX_FRAME_SIZE) {
            memcpy(m_frameQueue.data, m_receiveBuffer, dataLength);
            m_frameQueue.length = dataLength;
            m_frameQueue.valid = isValid;
            m_frameQueue.hasData = true;
        }
    }
}
