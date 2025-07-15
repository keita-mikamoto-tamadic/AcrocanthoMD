/**
 * @file byte_converter.cpp
 * @brief 型安全なバイト変換ユーティリティクラス実装
 * 
 * 組み込みシステム向けの高効率データ変換ライブラリ。
 * エンディアン対応、型安全性、ゼロオーバーヘッド設計。
 * 
 * @note 組み込み制約準拠:
 * - ROM使用量最小化
 * - 実行時性能最適化  
 * - スタック使用量最小化
 * - 動的メモリ確保なし
 */

#include "byte_converter.h"

// ==========================================================================
// 基本データ型変換実装
// ==========================================================================

void ByteConverter::writeFloat(uint8_t* buffer, int offset, float value) {
  writeValue<float>(buffer, offset, value);
}

float ByteConverter::readFloat(const uint8_t* buffer, int offset) {
  return readValue<float>(buffer, offset);
}

void ByteConverter::writeUint32(uint8_t* buffer, int offset, uint32_t value) {
  writeValue<uint32_t>(buffer, offset, value);
}

uint32_t ByteConverter::readUint32(const uint8_t* buffer, int offset) {
  return readValue<uint32_t>(buffer, offset);
}

void ByteConverter::writeUint16(uint8_t* buffer, int offset, uint16_t value) {
  writeValue<uint16_t>(buffer, offset, value);
}

uint16_t ByteConverter::readUint16(const uint8_t* buffer, int offset) {
  return readValue<uint16_t>(buffer, offset);
}

void ByteConverter::writeInt32(uint8_t* buffer, int offset, int32_t value) {
  writeValue<int32_t>(buffer, offset, value);
}

int32_t ByteConverter::readInt32(const uint8_t* buffer, int offset) {
  return readValue<int32_t>(buffer, offset);
}

// ==========================================================================
// レガシー互換API実装
// ==========================================================================

void ByteConverter::floatToBytes(float value, uint8_t (&result)[4]) {
  writeFloat(result, 0, value);
}

float ByteConverter::bytesToFloat(const uint8_t* bytes) {
  return readFloat(bytes, 0);
}

// ==========================================================================
// 汎用テンプレート実装
// ==========================================================================

template<typename T>
void ByteConverter::writeValueImpl(uint8_t* buffer, int offset, T value, Endianness endian) {
  static_assert(sizeof(T) <= 8, "Type size must be <= 8 bytes");
  
  union {
    T val;
    uint8_t bytes[sizeof(T)];
  } converter;
  
  converter.val = value;
  
  // エンディアン変換
  switch (endian) {
    case Endianness::BIG:
      // ビッグエンディアン形式で書き込み
      for (size_t i = 0; i < sizeof(T); ++i) {
        buffer[offset + i] = converter.bytes[sizeof(T) - 1 - i];
      }
      break;
      
    case Endianness::LITTLE:
      // リトルエンディアン形式で書き込み
      for (size_t i = 0; i < sizeof(T); ++i) {
        buffer[offset + i] = converter.bytes[i];
      }
      break;
      
    case Endianness::NATIVE:
      // ネイティブエンディアンで書き込み（組み込み環境向け）
      for (size_t i = 0; i < sizeof(T); ++i) {
        buffer[offset + i] = converter.bytes[i];
      }
      break;
  }
}

template<typename T>
T ByteConverter::readValueImpl(const uint8_t* buffer, int offset, Endianness endian) {
  static_assert(sizeof(T) <= 8, "Type size must be <= 8 bytes");
  
  union {
    T val;
    uint8_t bytes[sizeof(T)];
  } converter;
  
  // エンディアン変換
  switch (endian) {
    case Endianness::BIG:
      // ビッグエンディアン形式で読み込み
      for (size_t i = 0; i < sizeof(T); ++i) {
        converter.bytes[sizeof(T) - 1 - i] = buffer[offset + i];
      }
      break;
      
    case Endianness::LITTLE:
      // リトルエンディアン形式で読み込み
      for (size_t i = 0; i < sizeof(T); ++i) {
        converter.bytes[i] = buffer[offset + i];
      }
      break;
      
    case Endianness::NATIVE:
      // ネイティブエンディアンで読み込み（組み込み環境向け）
      for (size_t i = 0; i < sizeof(T); ++i) {
        converter.bytes[i] = buffer[offset + i];
      }
      break;
  }
  
  return converter.val;
}

// ==========================================================================
// 明示的テンプレート特殊化 (コンパイラ最適化用)
// ==========================================================================

// よく使われる型の明示的特殊化でコード最適化
template void ByteConverter::writeValueImpl<float>(uint8_t*, int, float, Endianness);
template void ByteConverter::writeValueImpl<uint32_t>(uint8_t*, int, uint32_t, Endianness);
template void ByteConverter::writeValueImpl<uint16_t>(uint8_t*, int, uint16_t, Endianness);
template void ByteConverter::writeValueImpl<int32_t>(uint8_t*, int, int32_t, Endianness);

template float ByteConverter::readValueImpl<float>(const uint8_t*, int, Endianness);
template uint32_t ByteConverter::readValueImpl<uint32_t>(const uint8_t*, int, Endianness);
template uint16_t ByteConverter::readValueImpl<uint16_t>(const uint8_t*, int, Endianness);
template int32_t ByteConverter::readValueImpl<int32_t>(const uint8_t*, int, Endianness);