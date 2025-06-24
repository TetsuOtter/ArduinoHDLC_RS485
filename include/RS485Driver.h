#ifndef RS485DRIVER_H
#define RS485DRIVER_H

#ifdef NATIVE_TEST
// テスト環境用のArduino定数定義
#include <stdint.h>
#include <cstddef>  // size_t用
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW  
#define LOW 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef INPUT
#define INPUT 0
#endif
#ifndef CHANGE
#define CHANGE 1
#endif
#else
#include <Arduino.h>
#endif

#include "IPinInterface.h"

/**
 * @brief RS485通信の基本的な制御を行うクラス
 * 
 * LTC485CN8を使用したRS485ドライバの制御を行います。
 * 送信はブロッキング、受信は割り込みベースで実装されています。
 * ピン操作はIPinInterfaceを通して行い、DIによりテスト可能。
 */
class RS485Driver {
public:
    /**
     * @brief 受信データのコールバック関数の型定義
     * @param bit 受信したビット (0 or 1)
     */
    typedef void (*BitReceivedCallback)(uint8_t bit);

    /**
     * @brief コンストラクタ
     * @param pinInterface ピン操作インターフェース
     * @param txPin 送信ピン
     * @param rxPin 受信ピン  
     * @param dePin ドライバイネーブルピン (LTC485CN8のDE)
     * @param rePin レシーバイネーブルピン (LTC485CN8のRE)
     * @param baudRate ボーレート
     */
    RS485Driver(IPinInterface& pinInterface, uint8_t txPin, uint8_t rxPin, uint8_t dePin, uint8_t rePin, uint32_t baudRate);

    /**
     * @brief 初期化
     * @return true 成功, false 失敗
     */
    bool begin();

    /**
     * @brief データの送信
     * @param data 送信するデータ (ビット配列)
     * @param bitLength 送信するビット数
     * @return true 送信成功, false 送信失敗
     */
    bool transmit(const uint8_t* data, size_t bitLength);

    /**
     * @brief 受信コールバック関数の設定
     * @param callback 受信時に呼び出されるコールバック関数
     */
    void setReceiveCallback(BitReceivedCallback callback);

    /**
     * @brief 受信開始
     */
    void startReceive();

    /**
     * @brief 受信停止
     */
    void stopReceive();

    /**
     * @brief 送信モードに切り替え
     */
    void enableTransmit();

    /**
     * @brief 受信モードに切り替え
     */
    void enableReceive();

    /**
     * @brief 現在の状態を取得
     * @return true 送信モード, false 受信モード
     */
    bool isTransmitting() const;

private:
    IPinInterface& m_pinInterface; ///< ピン操作インターフェース
    uint8_t m_txPin;        ///< 送信ピン
    uint8_t m_rxPin;        ///< 受信ピン
    uint8_t m_dePin;        ///< ドライバイネーブルピン
    uint8_t m_rePin;        ///< レシーバイネーブルピン
    uint32_t m_baudRate;    ///< ボーレート
    bool m_isTransmitting;  ///< 送信モードフラグ
    bool m_initialized;     ///< 初期化フラグ
    
    BitReceivedCallback m_receiveCallback; ///< 受信コールバック関数

    /**
     * @brief 1ビット送信
     * @param bit 送信するビット (0 or 1)
     */
    void transmitBit(uint8_t bit);

    /**
     * @brief ビット時間の遅延
     */
    void bitDelay();

    /**
     * @brief 受信割り込みハンドラ (静的関数)
     */
    static void receiveInterruptHandler();

    /**
     * @brief 受信処理 (インスタンスメソッド)
     */
    void handleReceive();

    static RS485Driver* s_instance; ///< 割り込みハンドラ用のインスタンス参照
};

#endif // RS485DRIVER_H
