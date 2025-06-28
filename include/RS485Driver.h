#ifndef RS485DRIVER_H
#define RS485DRIVER_H

#ifdef NATIVE_TEST
// テスト環境用のArduino定数定義
#include <stdint.h>
#include <cstddef> // size_t用
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
class RS485Driver
{
public:
    /**
     * @brief コンストラクタ
     * @param pinInterface ピン操作インターフェース
     * @param txPin 送信ピン
     * @param rxPin 受信ピン
     * @param dePin ドライバイネーブルピン (LTC485CN8のDE)
     * @param rePin レシーバイネーブルピン (LTC485CN8のRE)
     * @param baudRate ボーレート
     */
    RS485Driver(IPinInterface &pinInterface, uint8_t txPin, uint8_t rxPin, uint8_t dePin, uint8_t rePin, uint32_t baudRate);

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
    bool transmit(const uint8_t *data, size_t bitLength);

    /**
     * @brief データの受信（ポーリングベース）
     * @param buffer 受信データを格納するバッファ
     * @param maxBits 受信可能な最大ビット数
     * @param timeoutMs タイムアウト時間（ミリ秒）
     * @return 受信したビット数（0の場合はタイムアウトまたはエラー）
     */
    size_t read(uint8_t *buffer, size_t maxBits, uint32_t timeoutMs = 1000);

    /**
     * @brief 現在の受信ピンのビット状態を読み取り
     * @return 現在のビット状態 (0 or 1)
     */
    uint8_t readBit();

    /**
     * @brief ボーレートに基づいて1ビット分の時間待機
     */
    void waitBitTime();

    /**
     * @brief ボーレートに基づいて半ビット分の時間待機
     */
    void waitHalfBitTime();

    /**
     * @brief 経過時間を考慮して1ビット分の時間待機
     * @param elapsedMicros 既に経過した時間（マイクロ秒）
     */
    void waitBitTime(uint32_t elapsedMicros);

    /**
     * @brief ピンインターフェースの参照を取得
     * @return ピンインターフェースの参照
     */
    IPinInterface &getPinInterface();

    /**
     * @brief ボーレートを取得
     * @return 設定されているボーレート
     */
    uint32_t getBaudRate() const;

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
    IPinInterface &m_pinInterface; ///< ピン操作インターフェース
    uint8_t m_txPin;               ///< 送信ピン
    uint8_t m_rxPin;               ///< 受信ピン
    uint8_t m_dePin;               ///< ドライバイネーブルピン
    uint8_t m_rePin;               ///< レシーバイネーブルピン
    uint32_t m_baudRate;           ///< ボーレート
    bool m_isTransmitting;         ///< 送信モードフラグ
    bool m_initialized;            ///< 初期化フラグ

    /**
     * @brief 1ビット送信
     * @param bit 送信するビット (0 or 1)
     */
    void _transmitBit(uint8_t bit);
};

#endif // RS485DRIVER_H
