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
        CMD_I = 0x00,    // Information (下位3ビットに送信シーケンス番号)
        CMD_RR = 0x01,   // Receive Ready (下位3ビットに受信シーケンス番号)
        CMD_REJ = 0x09   // Reject (下位3ビットに受信シーケンス番号)
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
     * @brief Iコマンドでデータを送信（タイムアウト指定）
     * @param data 送信するデータ
     * @param length データ長
     * @param timeoutMs レスポンス待機タイムアウト時間（ミリ秒）
     * @return true 成功, false 失敗
     */
    bool sendICommand(const uint8_t *data, size_t length, uint32_t timeoutMs);

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
     * @brief 1バイト送信（フラグ用）
     * @param byte 送信バイト
     */
    void _transmitByte(uint8_t byte);

    /**
     * @brief 1バイト送信（ビットスタッフィング付き）
     * @param byte 送信バイト
     * @param consecutiveOnes 連続1カウント（参照渡し）
     */
    void _transmitByteWithStuffing(uint8_t byte, uint8_t &consecutiveOnes);

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

    // レスポンス判定ヘルパーメソッド
    /**
     * @brief RRフレーム（Receive Ready）かチェック
     * @param control コントロールフィールド
     * @return true RRフレーム, false その他
     */
    bool _isRRFrame(uint8_t control);

    /**
     * @brief REJフレーム（Reject）かチェック
     * @param control コントロールフィールド
     * @return true REJフレーム, false その他
     */
    bool _isREJFrame(uint8_t control);

    /**
     * @brief レスポンスフレームからシーケンス番号を抽出
     * @param control コントロールフィールド
     * @return シーケンス番号（0-7）
     */
    uint8_t _extractSequenceNumber(uint8_t control);
};

#endif // HDLC_H
