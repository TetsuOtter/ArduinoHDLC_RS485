#include <gtest/gtest.h>
#include "RS485Driver.h"
#include "HDLC.h"
#include "MockPinInterface.h"

// テスト用のピン定義
const uint8_t TEST_TX_PIN = 2;
const uint8_t TEST_RX_PIN = 3;
const uint8_t TEST_DE_PIN = 4;
const uint8_t TEST_RE_PIN = 5;
const uint32_t TEST_BAUD_RATE = 9600;

// ===== RS485Driver Tests =====

class RS485DriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockPin = std::make_unique<MockPinInterface>();
        driver = std::make_unique<RS485Driver>(*mockPin, TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    }

    void TearDown() override {
        driver.reset();
        mockPin.reset();
    }

    std::unique_ptr<MockPinInterface> mockPin;
    std::unique_ptr<RS485Driver> driver;
};

TEST_F(RS485DriverTest, Initialization) {
    EXPECT_TRUE(driver->begin());
    EXPECT_FALSE(driver->isTransmitting()); // 初期状態は受信モード
    
    // ピンモードが正しく設定されているかチェック
    EXPECT_EQ(OUTPUT, mockPin->getPinMode(TEST_TX_PIN));
    EXPECT_EQ(INPUT, mockPin->getPinMode(TEST_RX_PIN));
    EXPECT_EQ(OUTPUT, mockPin->getPinMode(TEST_DE_PIN));
    EXPECT_EQ(OUTPUT, mockPin->getPinMode(TEST_RE_PIN));
    
    // 初期状態は受信モード（DE=LOW, RE=LOW）
    EXPECT_EQ(LOW, mockPin->getPinValue(TEST_DE_PIN));
    EXPECT_EQ(LOW, mockPin->getPinValue(TEST_RE_PIN));
}

TEST_F(RS485DriverTest, ModeSwitching) {
    driver->begin();
    mockPin->clearLog();
    
    // 送信モードに切り替え
    driver->enableTransmit();
    EXPECT_TRUE(driver->isTransmitting());
    EXPECT_EQ(HIGH, mockPin->getPinValue(TEST_DE_PIN));
    EXPECT_EQ(HIGH, mockPin->getPinValue(TEST_RE_PIN));
    
    // 受信モードに切り替え
    driver->enableReceive();
    EXPECT_FALSE(driver->isTransmitting());
    EXPECT_EQ(LOW, mockPin->getPinValue(TEST_DE_PIN));
    EXPECT_EQ(LOW, mockPin->getPinValue(TEST_RE_PIN));
}

TEST_F(RS485DriverTest, BasicTransmit) {
    driver->begin();
    mockPin->clearLog();
    
    // テストデータ (1バイト = 8ビット)
    uint8_t testData[] = {0xA5}; // 10100101
    
    EXPECT_TRUE(driver->transmit(testData, 8));
    
    // 送信中にピンが適切に制御されたかチェック
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0);
    EXPECT_GT(mockPin->countDelays(), 0);
}

TEST_F(RS485DriverTest, TransmitInvalidParams) {
    driver->begin();
    
    // NULLポインタでのテスト
    EXPECT_FALSE(driver->transmit(nullptr, 8));
    
    // データ長0でのテスト
    uint8_t testData[] = {0xA5};
    EXPECT_FALSE(driver->transmit(testData, 0));
}

TEST_F(RS485DriverTest, TransmitBitPattern) {
    driver->begin();
    mockPin->clearLog();
    
    // 既知のビットパターンをテスト
    uint8_t testData[] = {0xFF}; // 11111111
    EXPECT_TRUE(driver->transmit(testData, 8));
    
    // TXピンへのHIGH書き込みが8回あることを確認
    EXPECT_EQ(8, mockPin->countDigitalWrites(TEST_TX_PIN, HIGH));
    
    mockPin->clearLog();
    testData[0] = 0x00; // 00000000
    EXPECT_TRUE(driver->transmit(testData, 8));
    
    // TXピンへのLOW書き込みが8回あることを確認
    EXPECT_EQ(8, mockPin->countDigitalWrites(TEST_TX_PIN, LOW));
}

// ===== HDLC Tests =====

class HDLCTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockPin = std::make_unique<MockPinInterface>();
        driver = std::make_unique<RS485Driver>(*mockPin, TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
        hdlc = std::make_unique<HDLC>(*driver);
    }

    void TearDown() override {
        hdlc.reset();
        driver.reset();
        mockPin.reset();
    }

    std::unique_ptr<MockPinInterface> mockPin;
    std::unique_ptr<RS485Driver> driver;
    std::unique_ptr<HDLC> hdlc;
};

TEST_F(HDLCTest, Initialization) {
    EXPECT_TRUE(hdlc->begin());
}

TEST_F(HDLCTest, CRCCalculation) {
    // 既知のテストデータでCRC計算をテスト
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = HDLC::calculateCRC16(testData, sizeof(testData));
    
    // CRCが0でないことを確認
    EXPECT_NE(0, crc);
    
    // 同じデータで再計算して一貫性を確認
    uint16_t crc2 = HDLC::calculateCRC16(testData, sizeof(testData));
    EXPECT_EQ(crc, crc2);
}

TEST_F(HDLCTest, TransmitFrame) {
    hdlc->begin();
    mockPin->clearLog();
    
    uint8_t testData[] = {0x01, 0x02, 0x03};
    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));
    
    // 何らかの送信が行われたことを確認
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0);
}

TEST_F(HDLCTest, TransmitInvalidParams) {
    hdlc->begin();
    
    // NULLポインタでのテスト
    EXPECT_FALSE(hdlc->transmitFrame(nullptr, 5));
    
    // データ長0でのテスト
    uint8_t testData[] = {0x01};
    EXPECT_FALSE(hdlc->transmitFrame(testData, 0));
}

TEST_F(HDLCTest, HexStringConversion) {
    hdlc->begin();
    
    // 正常な16進数文字列のテスト
    EXPECT_TRUE(hdlc->transmitHexString("01 02 03 FF"));
    EXPECT_TRUE(hdlc->transmitHexString("A0B1C2"));
    
    // 空文字列は失敗する
    EXPECT_FALSE(hdlc->transmitHexString(""));
}

TEST_F(HDLCTest, FrameStructure) {
    hdlc->begin();
    mockPin->clearLog();
    
    uint8_t testData[] = {0x42}; // Simple test data
    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));
    
    // HDLCフレームには少なくともフラグが含まれるはず
    // フラグは0x7E = 01111110なので、HIGHとLOWの両方の書き込みがある
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN, HIGH), 0);
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN, LOW), 0);
}

// コールバックテスト用のグローバル変数
static bool g_frameReceived = false;
static std::vector<uint8_t> g_receivedData;
static bool g_frameValid = false;

void testFrameCallback(const uint8_t* data, size_t length, bool isValid) {
    g_frameReceived = true;
    g_receivedData.assign(data, data + length);
    g_frameValid = isValid;
}

TEST_F(HDLCTest, CallbackSetting) {
    hdlc->begin();
    
    // コールバック関数の設定
    hdlc->setReceiveCallback(testFrameCallback);
    
    // コールバック設定自体はエラーにならない
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
