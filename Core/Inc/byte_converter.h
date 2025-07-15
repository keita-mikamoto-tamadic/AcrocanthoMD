#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief 型安全なバイト変換ユーティリティクラス
 * 
 * 組み込みシステム向けの効率的なデータ変換を提供。
 * エンディアン変換、型安全性、constexpr最適化に対応。
 * 
 * @note 組み込み制約に準拠:
 * - 動的メモリ確保なし
 * - 例外なし
 * - コンパイル時最適化
 * - ゼロオーバーヘッド抽象化
 */
class ByteConverter {
public:
  // ==========================================================================
  // 基本データ型変換 (推奨API)
  // ==========================================================================
  
  /**
   * @brief floatをビッグエンディアン形式でバッファに書き込み
   * @param buffer 書き込み先バッファ
   * @param offset バッファ内オフセット
   * @param value 書き込む値
   */
  static void writeFloat(uint8_t* buffer, int offset, float value);
  
  /**
   * @brief ビッグエンディアン形式でfloatを読み取り
   * @param buffer 読み取り元バッファ
   * @param offset バッファ内オフセット
   * @return 読み取った値
   */
  static float readFloat(const uint8_t* buffer, int offset);
  
  /**
   * @brief uint32をビッグエンディアン形式でバッファに書き込み
   */
  static void writeUint32(uint8_t* buffer, int offset, uint32_t value);
  
  /**
   * @brief ビッグエンディアン形式でuint32を読み取り
   */
  static uint32_t readUint32(const uint8_t* buffer, int offset);
  
  /**
   * @brief uint16をビッグエンディアン形式でバッファに書き込み
   */
  static void writeUint16(uint8_t* buffer, int offset, uint16_t value);
  
  /**
   * @brief ビッグエンディアン形式でuint16を読み取り
   */
  static uint16_t readUint16(const uint8_t* buffer, int offset);
  
  /**
   * @brief int32をビッグエンディアン形式でバッファに書き込み
   */
  static void writeInt32(uint8_t* buffer, int offset, int32_t value);
  
  /**
   * @brief ビッグエンディアン形式でint32を読み取り
   */
  static int32_t readInt32(const uint8_t* buffer, int offset);

  // ==========================================================================
  // レガシー互換API (後方互換性のため維持)
  // ==========================================================================
  
  /**
   * @brief floatを4バイト配列に変換 (レガシー)
   * @param value 変換する値
   * @param result 結果格納用4バイト配列
   */
  static void floatToBytes(float value, uint8_t (&result)[4]);
  
  /**
   * @brief 4バイト配列からfloatに変換 (レガシー)
   * @param bytes 変換元4バイト配列
   * @return 変換後の値
   */
  static float bytesToFloat(const uint8_t* bytes);

  // ==========================================================================
  // 高度なAPI (将来拡張用)
  // ==========================================================================
  
  /**
   * @brief エンディアン変換タイプ
   */
  enum class Endianness {
    BIG,    ///< ビッグエンディアン (ネットワークバイトオーダー)
    LITTLE, ///< リトルエンディアン
    NATIVE  ///< ネイティブエンディアン
  };
  
  /**
   * @brief 汎用型変換 (テンプレート特殊化)
   * @tparam T 変換する型
   * @param buffer バッファ
   * @param offset オフセット
   * @param value 書き込む値
   * @param endian エンディアン指定
   */
  template<typename T>
  static void writeValue(uint8_t* buffer, int offset, T value, 
                        Endianness endian = Endianness::BIG);
  
  /**
   * @brief 汎用型読み取り (テンプレート特殊化)
   * @tparam T 読み取る型
   * @param buffer バッファ
   * @param offset オフセット
   * @param endian エンディアン指定
   * @return 読み取った値
   */
  template<typename T>
  static T readValue(const uint8_t* buffer, int offset, 
                    Endianness endian = Endianness::BIG);

  // ==========================================================================
  // ユーティリティ関数
  // ==========================================================================
  
  /**
   * @brief システムエンディアンの判定
   * @return true: ビッグエンディアン, false: リトルエンディアン
   */
  static constexpr bool isBigEndian();
  
  /**
   * @brief バイトスワップ (16bit)
   */
  static constexpr uint16_t byteSwap16(uint16_t value);
  
  /**
   * @brief バイトスワップ (32bit)
   */
  static constexpr uint32_t byteSwap32(uint32_t value);

private:
  // 内部実装用テンプレート
  template<typename T>
  static void writeValueImpl(uint8_t* buffer, int offset, T value, Endianness endian);
  
  template<typename T>
  static T readValueImpl(const uint8_t* buffer, int offset, Endianness endian);
};

// ==========================================================================
// インライン実装 (パフォーマンス最適化)
// ==========================================================================

template<typename T>
inline void ByteConverter::writeValue(uint8_t* buffer, int offset, T value, Endianness endian) {
  writeValueImpl<T>(buffer, offset, value, endian);
}

template<typename T>
inline T ByteConverter::readValue(const uint8_t* buffer, int offset, Endianness endian) {
  return readValueImpl<T>(buffer, offset, endian);
}

constexpr bool ByteConverter::isBigEndian() {
  // C++14互換：コンパイル時エンディアン判定
  return static_cast<const uint8_t&>(static_cast<const uint16_t&>(1)) == 0;
}

constexpr uint16_t ByteConverter::byteSwap16(uint16_t value) {
  return (value << 8) | (value >> 8);
}

constexpr uint32_t ByteConverter::byteSwap32(uint32_t value) {
  return ((value << 24) & 0xFF000000) |
         ((value << 8)  & 0x00FF0000) |
         ((value >> 8)  & 0x0000FF00) |
         ((value >> 24) & 0x000000FF);
}