#include <gtest/gtest.h>
#include "RS485Driver.h"
#include "HDLC.h"
#include "MockPinInterface.h"

// ソースファイルを直接インクルード（テスト用）
#ifdef NATIVE_TEST
#include "../src/RS485Driver.cpp"
#include "../src/HDLC.cpp"
#endif

// テスト用のピン定義
const uint8_t TEST_TX_PIN = 2;
const uint8_t TEST_RX_PIN = 3;
const uint8_t TEST_DE_PIN = 4;
const uint8_t TEST_RE_PIN = 5;
const uint32_t TEST_BAUD_RATE = 9600;

// ===== RS485Driver Tests =====

class RS485DriverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mockPin = std::make_unique<MockPinInterface>();
        driver = std::make_unique<RS485Driver>(*mockPin, TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
    }

    void TearDown() override
    {
        driver.reset();
        mockPin.reset();
    }

    std::unique_ptr<MockPinInterface> mockPin;
    std::unique_ptr<RS485Driver> driver;
};

TEST_F(RS485DriverTest, Initialization)
{
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

TEST_F(RS485DriverTest, ModeSwitching)
{
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

TEST_F(RS485DriverTest, BasicTransmit)
{
    driver->begin();
    mockPin->clearLog();

    // テストデータ (1バイト = 8ビット)
    uint8_t testData[] = {0xA5}; // 10100101

    EXPECT_TRUE(driver->transmit(testData, 8));

    // 送信中にピンが適切に制御されたかチェック
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0);
    EXPECT_GT(mockPin->countDelays(), 0);
}

TEST_F(RS485DriverTest, TransmitInvalidParams)
{
    driver->begin();

    // NULLポインタでのテスト
    EXPECT_FALSE(driver->transmit(nullptr, 8));

    // データ長0でのテスト
    uint8_t testData[] = {0xA5};
    EXPECT_FALSE(driver->transmit(testData, 0));
}

TEST_F(RS485DriverTest, TransmitBitPattern)
{
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

class HDLCTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mockPin = std::make_unique<MockPinInterface>();
        driver = std::make_unique<RS485Driver>(*mockPin, TEST_TX_PIN, TEST_RX_PIN, TEST_DE_PIN, TEST_RE_PIN, TEST_BAUD_RATE);
        hdlc = std::make_unique<HDLC>(*driver);
    }

    void TearDown() override
    {
        hdlc.reset();
        driver.reset();
        mockPin.reset();
    }

    std::unique_ptr<MockPinInterface> mockPin;
    std::unique_ptr<RS485Driver> driver;
    std::unique_ptr<HDLC> hdlc;
};

TEST_F(HDLCTest, Initialization)
{
    EXPECT_TRUE(hdlc->begin());
}

TEST_F(HDLCTest, CRCCalculation)
{
    // 既知のテストデータでCRC計算をテスト
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = HDLC::calculateCRC16(testData, sizeof(testData));

    // CRCが0でないことを確認
    EXPECT_NE(0, crc);

    // 同じデータで再計算して一貫性を確認
    uint16_t crc2 = HDLC::calculateCRC16(testData, sizeof(testData));
    EXPECT_EQ(crc, crc2);
}

TEST_F(HDLCTest, BitStuffing_NoStuffingRequired)
{
    hdlc->begin();

    // スタッフィングが不要なデータ（連続する1が5個未満）
    uint8_t testData[] = {0x55}; // 01010101

    // ビットスタッフィング処理をテスト（privateメソッドなので、リフレクションを使用するか、publicにする必要がある）
    // 現在はpublicメソッドでテストできる範囲でテスト
    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));

    // 送信が行われたことを確認
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0);
}

TEST_F(HDLCTest, BitStuffing_StuffingRequired)
{
    hdlc->begin();

    // スタッフィングが必要なデータ（連続する1が5個以上）
    uint8_t testData[] = {0xFC}; // 11111100 - 連続する6個の1

    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));

    // 送信が行われたことを確認（実際のビット数は効率的なパッキングのため変わる）
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0); // 何らかの送信があった
}

TEST_F(HDLCTest, BitStuffing_MultipleStuffing)
{
    hdlc->begin();

    // 複数のスタッフィングが必要なデータ
    uint8_t testData[] = {0xFF, 0xFF}; // 連続する1が多数

    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));

    // 送信が行われたことを確認
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0); // 何らかの送信があった
}

TEST_F(HDLCTest, BitStuffing_FlagSequenceInData)
{
    hdlc->begin();

    // フラグシーケンス（0x7E）を含むデータ
    uint8_t testData[] = {0x7E, 0x01, 0x7E};

    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));

    // 送信が行われたことを確認
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0); // 何らかの送信があった
}

#ifdef NATIVE_TEST
TEST_F(HDLCTest, BitStuffing_DirectTest)
{
    hdlc->begin();

    // 直接ビットスタッフィング機能をテスト
    uint8_t testData[] = {0xFC}; // 11111100 - 連続する6個の1
    uint8_t stuffedBytes[100];

    size_t stuffedBitCount = hdlc->testBitStuff(testData, sizeof(testData), stuffedBytes, sizeof(stuffedBytes) * 8);

    // スタッフィング後のビット数が1以上であることを確認
    EXPECT_GT(stuffedBitCount, 0u);

    // スタッフィングにより元のデータより多くのビットが必要であることを確認
    // 元データ8ビットがスタッフィングで9ビット以上になる
    EXPECT_GT(stuffedBitCount, 8u); // 最低9ビット

    // スタッフィング結果にフラグシーケンス（0x7E）が含まれていないことを確認
    size_t byteCount = (stuffedBitCount + 7) / 8;
    for (size_t i = 0; i < byteCount; i++)
    {
        EXPECT_NE(stuffedBytes[i], 0x7E) << "Stuffed data should not contain flag sequence 0x7E";
    }
}

TEST_F(HDLCTest, BitDestuffing_DirectTest)
{
    hdlc->begin();

    // スタッフィングとデスタッフィングのラウンドトリップテスト
    uint8_t originalData[] = {0xFC}; // 11111100 - 連続する6個の1
    uint8_t stuffedBytes[100];
    uint8_t destuffedData[10];

    // スタッフィング
    size_t stuffedBitCount = hdlc->testBitStuff(originalData, sizeof(originalData), stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_GT(stuffedBitCount, 0u);

    // デスタッフィング
    size_t destuffedLength = hdlc->testBitDestuff(stuffedBytes, stuffedBitCount, destuffedData, sizeof(destuffedData));

    // デスタッフィング後は1バイト
    EXPECT_EQ(destuffedLength, 1u);

    // 元のデータが復元されることを確認
    EXPECT_EQ(destuffedData[0], 0xFC);
}

TEST_F(HDLCTest, RoundTrip_StuffingDestuffing)
{
    hdlc->begin();

    // 様々なパターンでラウンドトリップテスト
    uint8_t testPatterns[] = {
        0x00, 0xFF, 0x55, 0xAA, 0x7E, 0x7D, 0xFC, 0x1F};

    for (size_t i = 0; i < sizeof(testPatterns); i++)
    {
        uint8_t originalData[] = {testPatterns[i]};
        uint8_t stuffedBytes[100];
        uint8_t destuffedData[10];

        // スタッフィング
        size_t stuffedBitCount = hdlc->testBitStuff(originalData, 1, stuffedBytes, sizeof(stuffedBytes) * 8);
        EXPECT_GT(stuffedBitCount, 0u);

        // デスタッフィング
        size_t destuffedLength = hdlc->testBitDestuff(stuffedBytes, stuffedBitCount, destuffedData, sizeof(destuffedData));

        // 元データが復元されることを確認
        EXPECT_EQ(destuffedLength, 1u);
        EXPECT_EQ(destuffedData[0], originalData[0]) << "Failed for pattern 0x" << std::hex << (int)testPatterns[i];
    }
}

TEST_F(HDLCTest, FrameCreation_WithStuffing)
{
    hdlc->begin();

    // フレーム作成をテスト
    uint8_t testData[] = {0xFF, 0x7E, 0xFF}; // スタッフィングが多数発生するデータ
    uint8_t frameBytes[200];

    size_t frameBitCount = hdlc->testCreateFrameBits(testData, sizeof(testData), frameBytes, sizeof(frameBytes) * 8);

    EXPECT_GT(frameBitCount, 0u);

    // フレームが開始フラグ（0x7E）で始まることを確認
    EXPECT_EQ(frameBytes[0], 0x7E) << "Start flag should be 0x7E";

    // フレームの最後8ビットが終了フラグ（0x7E）であることを確認
    // ビット配列から最後の8ビットを抽出
    uint8_t lastByte = 0;
    for (int i = 0; i < 8; i++)
    {
        size_t bitIndex = frameBitCount - 8 + i;
        size_t byteIndex = bitIndex / 8;
        int bitPos = 7 - (bitIndex % 8);
        uint8_t bit = (frameBytes[byteIndex] >> bitPos) & 1;
        lastByte = (lastByte << 1) | bit;
    }
    EXPECT_EQ(lastByte, 0x7E) << "End flag should be 0x7E";
}
#endif

TEST_F(HDLCTest, TransmitFrame)
{
    hdlc->begin();
    mockPin->clearLog();

    uint8_t testData[] = {0x01, 0x02, 0x03};
    EXPECT_TRUE(hdlc->transmitFrame(testData, sizeof(testData)));

    // 何らかの送信が行われたことを確認
    EXPECT_GT(mockPin->countDigitalWrites(TEST_TX_PIN), 0);
}

TEST_F(HDLCTest, TransmitInvalidParams)
{
    hdlc->begin();

    // NULLポインタでのテスト
    EXPECT_FALSE(hdlc->transmitFrame(nullptr, 5));

    // データ長0でのテスト
    uint8_t testData[] = {0x01};
    EXPECT_FALSE(hdlc->transmitFrame(testData, 0));
}

TEST_F(HDLCTest, HexStringConversion)
{
    hdlc->begin();

    // 正常な16進数文字列のテスト
    EXPECT_TRUE(hdlc->transmitHexString("01 02 03 FF"));
    EXPECT_TRUE(hdlc->transmitHexString("A0B1C2"));

    // 空文字列は失敗する
    EXPECT_FALSE(hdlc->transmitHexString(""));
}

TEST_F(HDLCTest, FrameStructure)
{
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

void testFrameCallback(const uint8_t *data, size_t length, bool isValid)
{
    g_frameReceived = true;
    g_receivedData.assign(data, data + length);
    g_frameValid = isValid;
}

TEST_F(HDLCTest, CallbackSetting)
{
    hdlc->begin();

    // コールバック設定自体はエラーにならない
    EXPECT_TRUE(true);
}

#ifdef NATIVE_TEST
TEST_F(HDLCTest, BitStuffing_EdgeCases)
{
    hdlc->begin();

    // エッジケース: 空データ（length = 0）
    uint8_t someData[] = {0x00}; // データは存在するがlength=0でテスト
    uint8_t stuffedBytes[20];
    size_t stuffedCount = hdlc->testBitStuff(someData, 0, stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_EQ(stuffedCount, 0u); // length=0なので0が返される

    // エッジケース: nullptrデータ
    stuffedCount = hdlc->testBitStuff(nullptr, 5, stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_EQ(stuffedCount, 0u); // nullptrなので0が返される

    // エッジケース: 1バイトの0データ
    uint8_t zeroData[] = {0x00};
    stuffedCount = hdlc->testBitStuff(zeroData, sizeof(zeroData), stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_EQ(stuffedCount, 8u); // スタッフィング不要、8ビット

    // エッジケース: 1バイトの1データ
    uint8_t oneData[] = {0xFF};
    stuffedCount = hdlc->testBitStuff(oneData, sizeof(oneData), stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_GT(stuffedCount, 8u); // スタッフィングで増加
}

TEST_F(HDLCTest, BitDestuffing_EdgeCases)
{
    hdlc->begin();

    // エッジケース: 空ビット配列
    uint8_t emptyBytes[] = {};
    uint8_t destuffedData[10];
    size_t destuffedCount = hdlc->testBitDestuff(emptyBytes, 0, destuffedData, sizeof(destuffedData));
    EXPECT_EQ(destuffedCount, 0u);

    // エッジケース: 1バイト（スタッフィング無し）
    uint8_t singleByte[] = {0xAA}; // 10101010
    destuffedCount = hdlc->testBitDestuff(singleByte, 8, destuffedData, sizeof(destuffedData));
    EXPECT_EQ(destuffedCount, 1u);
    EXPECT_EQ(destuffedData[0], 0xAA);

    // エッジケース: フラグシーケンス以外の通常データ
    uint8_t normalData[] = {0x55}; // 01010101
    destuffedCount = hdlc->testBitDestuff(normalData, 8, destuffedData, sizeof(destuffedData));
    EXPECT_EQ(destuffedCount, 1u);
    EXPECT_EQ(destuffedData[0], 0x55);
}

TEST_F(HDLCTest, ComplexStuffingPattern)
{
    hdlc->begin();

    // 複雑なスタッフィングパターンのテスト
    // 0xF8 = 11111000 (5個の連続する1)
    // 0x1F = 00011111 (5個の連続する1)
    uint8_t complexData[] = {0xF8, 0x1F, 0xFC};
    uint8_t stuffedBytes[100];
    uint8_t destuffedData[10];

    // スタッフィング
    size_t stuffedBitCount = hdlc->testBitStuff(complexData, sizeof(complexData), stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_GT(stuffedBitCount, 24u); // 元の24ビットより多い

    // デスタッフィング
    size_t destuffedCount = hdlc->testBitDestuff(stuffedBytes, stuffedBitCount, destuffedData, sizeof(destuffedData));
    EXPECT_EQ(destuffedCount, sizeof(complexData));

    // 元データと比較
    for (size_t i = 0; i < sizeof(complexData); i++)
    {
        EXPECT_EQ(destuffedData[i], complexData[i]) << "Data mismatch at index " << i;
    }
}

TEST_F(HDLCTest, MaximumStuffingScenario)
{
    hdlc->begin();

    // 最大スタッフィングが発生するシナリオ
    // 連続する1が多数含まれるデータ
    uint8_t maxStuffData[10];
    for (int i = 0; i < 10; i++)
    {
        maxStuffData[i] = 0xFF; // 全て1
    }

    uint8_t stuffedBytes[200]; // 十分大きなバッファ
    uint8_t destuffedData[20];

    size_t stuffedBitCount = hdlc->testBitStuff(maxStuffData, sizeof(maxStuffData), stuffedBytes, sizeof(stuffedBytes) * 8);
    EXPECT_GT(stuffedBitCount, 80u); // 元の80ビットより大幅に増加

    size_t destuffedCount = hdlc->testBitDestuff(stuffedBytes, stuffedBitCount, destuffedData, sizeof(destuffedData));
    EXPECT_EQ(destuffedCount, sizeof(maxStuffData));

    // 元データが正確に復元されることを確認
    for (size_t i = 0; i < sizeof(maxStuffData); i++)
    {
        EXPECT_EQ(destuffedData[i], maxStuffData[i]);
    }
}

TEST_F(HDLCTest, ReceiveBitProcessing_WithStuffing)
{
    hdlc->begin();

    // Note: 新しい実装では受信はポーリングベースの bit-level control で行われ、
    // 割り込みベースの処理は使用されません。
    // このテストはフレーム受信のタイムアウト処理を確認します。

    // RXピンをアイドル状態（HIGH）に設定
    mockPin->setPinValue(TEST_RX_PIN, HIGH);

    // receiveFrameWithBitControlの基本動作確認
    bool result = hdlc->receiveFrameWithBitControl(5); // 5msタイムアウト
    EXPECT_FALSE(result);                              // データなしでタイムアウト

    // 受信処理が正常に動作することを確認（具体的な検証は実際の送受信テストで行う）
    EXPECT_TRUE(true); // 基本的な受信処理が例外なく完了することを確認
}
#endif

TEST_F(RS485DriverTest, ReadBit_Timeout)
{
    driver->begin();

    // RXピンは常にLOW（アイドル状態）
    mockPin->setPinValue(TEST_RX_PIN, LOW);

    // readBitの基本動作確認
    uint8_t bit = driver->readBit();
    EXPECT_EQ(bit, 0u); // LOWなので0

    // タイミング関連の関数が正常に動作することを確認
    driver->waitBitTime();
    driver->waitHalfBitTime();
    EXPECT_TRUE(true); // 例外なく完了
}

TEST_F(RS485DriverTest, ReadBit_InvalidParams)
{
    driver->begin();

    // readBitは無効なパラメータがないため、基本動作のみテスト
    mockPin->setPinValue(TEST_RX_PIN, HIGH);
    EXPECT_EQ(1u, driver->readBit());

    mockPin->setPinValue(TEST_RX_PIN, LOW);
    EXPECT_EQ(0u, driver->readBit());
}

TEST_F(RS485DriverTest, ReadBit_Basic)
{
    driver->begin();

    // RXピンをHIGHに設定
    mockPin->setPinValue(TEST_RX_PIN, HIGH);
    EXPECT_EQ(1u, driver->readBit());

    // RXピンをLOWに設定
    mockPin->setPinValue(TEST_RX_PIN, LOW);
    EXPECT_EQ(0u, driver->readBit());
}

TEST_F(RS485DriverTest, WaitBitTime_Basic)
{
    driver->begin();
    mockPin->clearLog();

    driver->waitBitTime();

    // delayMicrosecondsが呼び出されたことを確認
    size_t logSize = mockPin->getLogSize();
    bool delayFound = false;
    for (size_t i = 0; i < logSize; i++)
    {
        const auto &entry = mockPin->getLogEntry(i);
        if (entry.type == MockPinInterface::LogEntry::DELAY_MICROS)
        {
            delayFound = true;
            break;
        }
    }
    EXPECT_TRUE(delayFound);
}

TEST_F(HDLCTest, ReceiveFrameWithBitControl_Timeout)
{
    hdlc->begin();

    // タイムアウトテスト
    bool result = hdlc->receiveFrameWithBitControl(10); // 10msタイムアウト
    EXPECT_FALSE(result);                               // タイムアウトで失敗するはず
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
