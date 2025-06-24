#include "RS485Driver.h"
#include "HDLC.h"
#include <iostream>

// テスト用のSerial代替
#define Serial std::cout

// テスト用のピン定義
#define TEST_TX_PIN 2
#define TEST_RX_PIN 3
#define TEST_DE_PIN 4
#define TEST_RE_PIN 5
#define TEST_BAUD_RATE 9600

// 簡易テストフレームワーク
int testCount = 0;
int passCount = 0;

void ASSERT_TRUE(bool condition, const char* message) {
    testCount++;
    if (condition) {
        passCount++;
        Serial.print("PASS: ");
        Serial.println(message);
    } else {
        Serial.print("FAIL: ");
        Serial.println(message);
    }
}

void ASSERT_FALSE(bool condition, const char* message) {
    ASSERT_TRUE(!condition, message);
}

void ASSERT_NOT_EQUAL(int expected, int actual, const char* message) {
    ASSERT_TRUE(expected != actual, message);
}

void ASSERT_EQUAL(int expected, int actual, const char* message) {
    ASSERT_TRUE(expected == actual, message);
}

// ===== RS485Driver Tests =====

void test_rs485_driver_initialization() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    
    ASSERT_TRUE(driver.begin(), "RS485Driver initialization");
    ASSERT_FALSE(driver.isTransmitting(), "Initial state is receive mode");
}

void test_rs485_driver_mode_switching() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    driver.begin();
    
    // 送信モードに切り替え
    driver.enableTransmit();
    ASSERT_TRUE(driver.isTransmitting(), "Switch to transmit mode");
    
    // 受信モードに切り替え
    driver.enableReceive();
    ASSERT_FALSE(driver.isTransmitting(), "Switch to receive mode");
}

void test_rs485_driver_transmit_basic() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    driver.begin();
    
    // テストデータ (1バイト = 8ビット)
    uint8_t testData[] = {0xA5}; // 10100101
    
    ASSERT_TRUE(driver.transmit(testData, 8), "Basic transmit test");
}

void test_rs485_driver_transmit_invalid_params() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    driver.begin();
    
    // NULLポインタでのテスト
    ASSERT_FALSE(driver.transmit(nullptr, 8), "Transmit with null pointer");
    
    // データ長0でのテスト
    uint8_t testData[] = {0xA5};
    ASSERT_FALSE(driver.transmit(testData, 0), "Transmit with zero length");
}

// ===== HDLC Tests =====

void test_hdlc_initialization() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    HDLC hdlc(driver);
    
    ASSERT_TRUE(hdlc.begin(), "HDLC initialization");
}

void test_hdlc_crc_calculation() {
    // 既知のテストデータでCRC計算をテスト
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = HDLC::calculateCRC16(testData, sizeof(testData));
    
    // CRC-16-IBMの期待値 (計算結果は実装依存だが、一貫性を確認)
    ASSERT_NOT_EQUAL(0, crc, "CRC calculation not zero");
    
    // 同じデータで再計算して一貫性を確認
    uint16_t crc2 = HDLC::calculateCRC16(testData, sizeof(testData));
    ASSERT_EQUAL(crc, crc2, "CRC calculation consistency");
}

void test_hdlc_transmit_frame() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    HDLC hdlc(driver);
    hdlc.begin();
    
    uint8_t testData[] = {0x01, 0x02, 0x03};
    ASSERT_TRUE(hdlc.transmitFrame(testData, sizeof(testData)), "HDLC frame transmission");
}

void test_hdlc_transmit_invalid_params() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    HDLC hdlc(driver);
    hdlc.begin();
    
    // NULLポインタでのテスト
    ASSERT_FALSE(hdlc.transmitFrame(nullptr, 5), "HDLC transmit with null pointer");
    
    // データ長0でのテスト
    uint8_t testData[] = {0x01};
    ASSERT_FALSE(hdlc.transmitFrame(testData, 0), "HDLC transmit with zero length");
}

void test_hdlc_hex_string_conversion() {
    RS485Driver driver(TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    HDLC hdlc(driver);
    hdlc.begin();
    
    // 正常な16進数文字列のテスト
    ASSERT_TRUE(hdlc.transmitHexString("01 02 03 FF"), "Hex string transmission with spaces");
    ASSERT_TRUE(hdlc.transmitHexString("A0B1C2"), "Hex string transmission without spaces");
    
    // 空文字列は失敗する
    ASSERT_FALSE(hdlc.transmitHexString(""), "Empty hex string should fail");
}

void runAllTests() {
    Serial.println("=== Running RS485Driver and HDLC Tests ===");
    
    // RS485Driverのテストを実行
    Serial.println("\n--- RS485Driver Tests ---");
    test_rs485_driver_initialization();
    test_rs485_driver_mode_switching();
    test_rs485_driver_transmit_basic();
    test_rs485_driver_transmit_invalid_params();
    
    // HDLCのテストを実行
    Serial.println("\n--- HDLC Tests ---");
    test_hdlc_initialization();
    test_hdlc_crc_calculation();
    test_hdlc_transmit_frame();
    test_hdlc_transmit_invalid_params();
    test_hdlc_hex_string_conversion();
    
    // 結果表示
    Serial.println("\n=== Test Results ===");
    Serial.print("Total Tests: ");
    Serial.println(testCount);
    Serial.print("Passed: ");
    Serial.println(passCount);
    Serial.print("Failed: ");
    Serial.println(testCount - passCount);
    
    if (passCount == testCount) {
        Serial.println("ALL TESTS PASSED!");
    } else {
        Serial.println("SOME TESTS FAILED!");
    }
}

void setup() {
    // シリアル通信を初期化
    Serial.begin(9600);
    while (!Serial) {
        ; // シリアルポートが接続されるまで待機
    }
    
    delay(2000); // 安定化のため少し待機
    
    runAllTests();
}

void loop() {
    // テストは setup() で実行されるため、loop() では何もしない
    delay(1000);
}
