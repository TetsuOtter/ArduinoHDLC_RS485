#ifndef HDLC_H
#define HDLC_H

#ifdef NATIVE_TEST
// テスト環境用のArduino定数定義
#include <stdint.h>
#include <cstddef> // size_t用
#include <string>  // std::string用
#include <cstdlib> // malloc, free用
#include <cstring> // memset用
// ArduinoのString型の代替
using String = std::string;
#else
#include <Arduino.h>
#endif

#include "IPinInterface.h"

/**
 * @brief 統合HDLC/RS485通信クラス
 *
 * HDLCプロトコルとRS485物理層を統合し、SNRMやIコマンドをサポートします。
 */
class HDLC
{
public:
    /**
     * @brief HDLCフレームの最大サイズ（メモリ削減のため64バイトに制限）
     */
    static const size_t MAX_FRAME_SIZE = 64;

    /**
     * @brief HDLCフラグシーケンス
     */
    static const uint8_t FLAG_SEQUENCE = 0x7E;

    /**
     * @brief HDLCコマンドタイプ
     */
    enum CommandType
    {
        CMD_SNRM = 0x83, // Set Normal Response Mode
        CMD_UA = 0x63,   // Unnumbered Acknowledgment
        CMD_I = 0x00     // Information (下位3ビットに送信シーケンス番号)
    };

    /**
     * @brief コンストラクタ
     * @param pinInterface ピンインターフェース
     * @param txPin 送信ピン
     * @param rxPin 受信ピン
     * @param dePin ドライバイネーブルピン
     * @param rePin レシーバイネーブルピン
     * @param baudRate ボーレート
     */
    HDLC(IPinInterface &pinInterface, uint8_t txPin, uint8_t rxPin,
         uint8_t dePin, uint8_t rePin, uint32_t baudRate);

    /**
     * @brief 初期化（アドレス入力を含む）
     * @return true 成功, false 失敗
     */
    bool begin();

    /**
     * @brief 指定アドレスにSNRMコマンドを送信してUAを待機
     * @return true 成功, false 失敗
     */
    bool sendSNRMAndWaitUA();

    /**
     * @brief Iコマンドでデータを送信
     * @param data 送信するデータ
     * @param length データ長
     * @return true 成功, false 失敗
     */
    bool sendICommand(const uint8_t *data, size_t length);

    /**
     * @brief フレーム受信（低レベルビット制御）
     * @param timeoutMs タイムアウト時間（ミリ秒）
     * @return true フレーム受信成功, false タイムアウトまたはエラー
     */
    bool receiveFrameWithBitControl(uint32_t timeoutMs = 5000);

    /**
     * @brief 受信データキューから読み出し
     * @param buffer 読み出し先バッファ
     * @param bufferSize バッファサイズ
     * @return 読み出したデータ長 (0の場合はデータなし)
     */
    size_t readFrame(uint8_t *buffer, size_t bufferSize);

    /**
     * @brief 送信先アドレスの設定
     * @param address 送信先アドレス
     */
    void setAddress(uint8_t address);

    /**
     * @brief CRC-16計算
     * @param data データ
     * @param length データ長
     * @return CRC値
     */
    static uint16_t calculateCRC16(const uint8_t *data, size_t length);

// テスト用public関数
#ifdef NATIVE_TEST
    /**
     * @brief ビットスタッフィング（テスト用公開）
     */
    size_t testBitStuff(const uint8_t *data, size_t length, uint8_t *stuffedBits, size_t maxBits)
    {
        return _bitStuff(data, length, stuffedBits, maxBits);
    }

    /**
     * @brief ビットデスタッフィング（テスト用公開）
     */
    size_t testBitDestuff(const uint8_t *stuffedBytes, size_t bitCount, uint8_t *destuffedData, size_t maxLength)
    {
        return _bitDestuff(stuffedBytes, bitCount, destuffedData, maxLength);
    }

    /**
     * @brief フレーム作成（テスト用公開）
     */
    size_t testCreateFrameBits(const uint8_t *data, size_t length, uint8_t *frameBytes, size_t maxBits)
    {
        return _createFrameBits(data, length, frameBytes, maxBits);
    }
#endif

private:
    // 受信コンテキスト構造体
    struct ReceiveContext
    {
        uint8_t flagBuffer;                  ///< フラグシーケンス検出用
        size_t flagBitCount;                 ///< フラグビットカウント
        bool inFrame;                        ///< フレーム内フラグ
        uint8_t rawData[MAX_FRAME_SIZE * 2]; ///< スタッフィング済みデータ用
        size_t rawBitIndex;                  ///< 生データビットインデックス
        bool frameComplete;                  ///< フレーム完了フラグ
    };

    // RS485物理層パラメータ
    IPinInterface &m_pinInterface;
    uint8_t m_txPin;
    uint8_t m_rxPin;
    uint8_t m_dePin;
    uint8_t m_rePin;
    uint32_t m_baudRate;
    uint32_t m_bitTimeMicros;
    uint32_t m_halfBitTimeMicros;
    bool m_isTransmitting;

    // HDLC状態
    bool m_initialized;
    uint8_t m_targetAddress;   ///< 送信先アドレス
    uint8_t m_sendSequence;    ///< 送信シーケンス番号（0-7）
    uint8_t m_receiveSequence; ///< 受信シーケンス番号（0-7）

    uint8_t m_receiveBuffer[MAX_FRAME_SIZE];
    size_t m_receiveIndex;
    uint8_t m_currentByte;
    uint8_t m_bitCount;
    uint8_t m_consecutiveOnes; ///< 連続する1ビットのカウント（デスタッフィング用）

    // 事前計算された待機時間
    uint32_t m_shortDelayMicros; ///< フラグ検出時の短い待機時間（1/8ビット時間）

    // 受信データキュー (簡易実装)
    struct FrameQueue
    {
        uint8_t data[MAX_FRAME_SIZE];
        size_t length;
        bool valid;
        bool hasData;
    } m_frameQueue;

    // RS485制御メソッド
    /**
     * @brief 送信モードに切り替え
     */
    void _enableTransmit();

    /**
     * @brief 受信モードに切り替え
     */
    void _enableReceive();

    /**
     * @brief 1ビット送信
     * @param bit 送信ビット
     */
    void _transmitBit(uint8_t bit);

    /**
     * @brief 1ビット受信
     * @return 受信ビット
     */
    uint8_t _readBit();

    /**
     * @brief ビット時間待機
     */
    void _waitBitTime();

    /**
     * @brief 半ビット時間待機
     */
    void _waitHalfBitTime();

    /**
     * @brief 経過時間を考慮したビット時間待機
     * @param elapsedMicros 経過時間（マイクロ秒）
     */
    void _waitBitTime(uint32_t elapsedMicros);

    // HDLCプロトコルメソッド
    /**
     * @brief 生フレーム送信（内部用）
     * @param data 送信データ
     * @param length データ長
     * @return true 成功, false 失敗
     */
    bool _transmitFrame(const uint8_t *data, size_t length);

    /**
     * @brief HDLCフレームの作成
     * @param address アドレス
     * @param control コントロールフィールド
     * @param info 情報フィールド
     * @param infoLength 情報フィールド長
     * @param frameBuffer 出力バッファ
     * @param maxLength 最大長
     * @return フレーム長
     */
    size_t _createHDLCFrame(uint8_t address, uint8_t control,
                            const uint8_t *info, size_t infoLength,
                            uint8_t *frameBuffer, size_t maxLength);

    /**
     * @brief ビットスタッフィング（送信用）
     * @param data 元データ
     * @param length 元データ長（バイト）
     * @param stuffedBits スタッフィング後のバイト配列（効率的にパッキング）
     * @param maxBits 最大ビット数
     * @return スタッフィング後のビット数
     */
    size_t _bitStuff(const uint8_t *data, size_t length, uint8_t *stuffedBits, size_t maxBits);

    /**
     * @brief ビットデスタッフィング（受信用）
     * @param stuffedBytes スタッフィング済みバイト配列
     * @param bitCount ビット数
     * @param destuffedData デスタッフィング後のデータ
     * @param maxLength 最大データ長（バイト）
     * @return デスタッフィング後のデータ長（バイト）
     */
    size_t _bitDestuff(const uint8_t *stuffedBytes, size_t bitCount, uint8_t *destuffedData, size_t maxLength);

    /**
     * @brief バイトバッファに1ビットを書き込み
     * @param buffer 書き込み先バッファ
     * @param bitIndex ビット位置
     * @param bit 書き込むビット値
     */
    void _writeBitToBuffer(uint8_t *buffer, size_t bitIndex, uint8_t bit);

    /**
     * @brief バイトバッファに複数ビットを書き込み
     * @param buffer 書き込み先バッファ
     * @param bitIndex 開始ビット位置
     * @param value 書き込む値
     * @param numBits ビット数
     */
    void _writeBitsToBuffer(uint8_t *buffer, size_t bitIndex, uint8_t value, size_t numBits);

    /**
     * @brief HDLCフレームの作成（ビットスタッフィング対応）
     * @param data ペイロードデータ
     * @param length ペイロード長
     * @param frameBits 作成されたフレーム（ビット配列）
     * @param maxBits フレームの最大ビット数
     * @return フレームのビット数
     */
    size_t _createFrameBits(const uint8_t *data, size_t length, uint8_t *frameBits, size_t maxBits);

    /**
     * @brief 受信ビットの処理
     * @param bit 受信したビット
     */
    void _processBit(uint8_t bit);

    /**
     * @brief 受信フレームの処理
     */
    void _processReceivedFrame();

    // 受信フレーム処理の分割メソッド
    /**
     * @brief 受信状態の初期化
     */
    void _initializeReceiveState();

    /**
     * @brief 受信コンテキストの初期化
     * @param context 受信コンテキスト
     */
    void _initializeReceiveContext(ReceiveContext &context);

    /**
     * @brief 受信ビットの処理
     * @param bit 受信ビット
     * @param context 受信コンテキスト
     */
    void _processReceivedBit(uint8_t bit, ReceiveContext &context);

    /**
     * @brief フラグ検出の更新
     * @param bit 受信ビット
     * @param context 受信コンテキスト
     */
    void _updateFlagDetection(uint8_t bit, ReceiveContext &context);

    /**
     * @brief フラグシーケンスの確認
     * @param context 受信コンテキスト
     * @return true フラグシーケンス検出, false 未検出
     */
    bool _isFlagSequence(const ReceiveContext &context);

    /**
     * @brief フラグシーケンスの処理
     * @param context 受信コンテキスト
     */
    void _handleFlagSequence(ReceiveContext &context);

    /**
     * @brief フレーム開始処理
     * @param context 受信コンテキスト
     */
    void _startFrame(ReceiveContext &context);

    /**
     * @brief フレーム終了処理
     * @param context 受信コンテキスト
     */
    void _endFrame(ReceiveContext &context);

    /**
     * @brief フレーム内へのビット保存
     * @param bit 受信ビット
     * @param context 受信コンテキスト
     */
    void _storeBitInFrame(uint8_t bit, ReceiveContext &context);

    /**
     * @brief 完了フレームの処理
     * @param rawData 生データ
     * @param rawBitCount 生データビット数
     * @return true 有効フレーム, false 無効フレーム
     */
    bool _processCompleteFrame(const uint8_t *rawData, size_t rawBitCount);

    /**
     * @brief フレームCRCの検証
     * @param frameLength フレーム長
     * @return true CRC正常, false CRC異常
     */
    bool _validateFrameCRC(size_t frameLength);

    /**
     * @brief 有効フレームの保存
     * @param frameLength フレーム長
     */
    void _storeValidFrame(size_t frameLength);
};

#endif // HDLC_H
