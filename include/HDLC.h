#ifndef HDLC_H
#define HDLC_H

#ifdef NATIVE_TEST
// テスト環境用のArduino定数定義
#include <stdint.h>
#include <cstddef>  // size_t用
#include <string>   // std::string用
// ArduinoのString型の代替
using String = std::string;
#else
#include <Arduino.h>
#endif

#include "RS485Driver.h"

/**
 * @brief HDLCプロトコルに基づくデータの送受信を行うクラス
 * 
 * データのフレーミング、CRC計算、送受信、データ検証を行います。
 */
class HDLC {
public:
    /**
     * @brief 受信完了コールバック関数の型定義
     * @param data 受信したデータ
     * @param length データ長
     * @param isValid CRCチェックの結果
     */
    typedef void (*FrameReceivedCallback)(const uint8_t* data, size_t length, bool isValid);

    /**
     * @brief HDLCフレームの最大サイズ
     */
    static const size_t MAX_FRAME_SIZE = 256;

    /**
     * @brief HDLCフラグシーケンス
     */
    static const uint8_t FLAG_SEQUENCE = 0x7E;

    /**
     * @brief HDLCエスケープ文字
     */
    static const uint8_t ESCAPE_CHAR = 0x7D;

    /**
     * @brief コンストラクタ
     * @param driver RS485Driverのインスタンス
     */
    HDLC(RS485Driver& driver);

    /**
     * @brief 初期化
     * @return true 成功, false 失敗
     */
    bool begin();

    /**
     * @brief データの送信
     * @param data 送信するデータ
     * @param length データ長
     * @return true 送信成功, false 送信失敗
     */
    bool transmitFrame(const uint8_t* data, size_t length);

    /**
     * @brief 16進数文字列をバイト配列に変換して送信
     * @param hexString 16進数文字列 (例: "00F0A1")
     * @return true 送信成功, false 送信失敗
     */
    bool transmitHexString(const String& hexString);

    /**
     * @brief 受信コールバック関数の設定
     * @param callback 受信完了時に呼び出されるコールバック関数
     */
    void setReceiveCallback(FrameReceivedCallback callback);

    /**
     * @brief 受信開始
     */
    void startReceive();

    /**
     * @brief 受信停止
     */
    void stopReceive();

    /**
     * @brief 受信データキューから読み出し
     * @param buffer 読み出し先バッファ
     * @param bufferSize バッファサイズ
     * @return 読み出したデータ長 (0の場合はデータなし)
     */
    size_t readFrame(uint8_t* buffer, size_t bufferSize);

    /**
     * @brief 受信データを16進数文字列として取得
     * @return 16進数文字列 (データがない場合は空文字列)
     */
    String readFrameAsHexString();

    /**
     * @brief CRC-16計算
     * @param data データ
     * @param length データ長
     * @return CRC値
     */
    static uint16_t calculateCRC16(const uint8_t* data, size_t length);

private:
    RS485Driver& m_driver;      ///< RS485ドライバの参照
    bool m_initialized;         ///< 初期化フラグ
    
    // 受信状態管理
    enum ReceiveState {
        WAITING_FOR_FLAG,
        RECEIVING_DATA,
        ESCAPE_NEXT
    };
    
    ReceiveState m_receiveState;
    uint8_t m_receiveBuffer[MAX_FRAME_SIZE];
    size_t m_receiveIndex;
    uint8_t m_currentByte;
    uint8_t m_bitCount;
    
    FrameReceivedCallback m_receiveCallback; ///< 受信コールバック関数
    
    // 受信データキュー (簡易実装)
    struct FrameQueue {
        uint8_t data[MAX_FRAME_SIZE];
        size_t length;
        bool valid;
        bool hasData;
    } m_frameQueue;

    /**
     * @brief データのバイト・ビット・スタッフィング
     * @param data 元データ
     * @param length 元データ長
     * @param stuffedData スタッフィング後のデータ
     * @param maxLength 最大長
     * @return スタッフィング後のデータ長
     */
    size_t stuffData(const uint8_t* data, size_t length, uint8_t* stuffedData, size_t maxLength);

    /**
     * @brief データのデスタッフィング
     * @param stuffedData スタッフィングされたデータ
     * @param length データ長
     * @param destuffedData デスタッフィング後のデータ
     * @param maxLength 最大長
     * @return デスタッフィング後のデータ長
     */
    size_t destuffData(const uint8_t* stuffedData, size_t length, uint8_t* destuffedData, size_t maxLength);

    /**
     * @brief HDLCフレームの作成
     * @param data ペイロードデータ
     * @param length ペイロード長
     * @param frame 作成されたフレーム
     * @param maxFrameLength フレームの最大長
     * @return フレーム長
     */
    size_t createFrame(const uint8_t* data, size_t length, uint8_t* frame, size_t maxFrameLength);

    /**
     * @brief 16進数文字列をバイト配列に変換
     * @param hexString 16進数文字列
     * @param buffer 変換結果のバッファ
     * @param maxLength バッファの最大長
     * @return 変換されたバイト数
     */
    size_t hexStringToBytes(const String& hexString, uint8_t* buffer, size_t maxLength);

    /**
     * @brief バイト配列を16進数文字列に変換
     * @param data バイト配列
     * @param length データ長
     * @return 16進数文字列
     */
    String bytesToHexString(const uint8_t* data, size_t length);

    /**
     * @brief RS485Driverからの受信ビット処理コールバック
     * @param bit 受信したビット
     */
    static void bitReceivedCallback(uint8_t bit);

    /**
     * @brief 受信ビットの処理
     * @param bit 受信したビット
     */
    void processBit(uint8_t bit);

    /**
     * @brief 受信フレームの処理
     */
    void processReceivedFrame();

    static HDLC* s_instance; ///< コールバック用のインスタンス参照
};

#endif // HDLC_H
