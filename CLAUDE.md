# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

AcrocanthoMDは、STM32G4マイコンを使用したブラシレスDCモーター制御システムです。フィールド指向制御（FOC）技術により高精度なサーボモーター制御を実現します。複数のモータータイプ（GIM6010_8、GIM8108_8）に対応し、CAN通信とリアルタイム制御ループを提供します。

## ビルドシステム

**ツールチェーン**: GNU Tools for STM32 (13.3.rel1) with arm-none-eabi-gcc/g++
**ターゲットMCU**: STM32G474RET6 (Cortex-M4 with FPU)

### ビルドコマンド
**注意**: ビルドはユーザーが実行します。Claudeは**ビルドを実行しません**。

```bash
# プロジェクトのビルド（Debugディレクトリから実行）
cd Debug
make all

# クリーンビルド
make clean

# バイナリサイズ情報の表示
arm-none-eabi-size AcrocanthoMD.elf

# 逆アセンブリリストの生成
arm-none-eabi-objdump -h -S AcrocanthoMD.elf > AcrocanthoMD.list
```

**Claudeの制約**: 
- STM32開発環境（arm-none-eabi-gcc、make）がClaude環境に未インストール
- ビルド検証が必要な場合は、ユーザーに実行を依頼すること
- コード変更後は必ずユーザーにビルドテストを推奨すること

### 主要なビルド成果物
- `AcrocanthoMD.elf` - メイン実行ファイル
- `AcrocanthoMD.map` - メモリマップファイル
- `AcrocanthoMD.list` - 逆アセンブリリスト

## プロジェクト設定

**ハードウェア設定**: STM32CubeIDE（.iocファイル）で定義
**モーター選択**: `Core/Inc/param.h`で設定：
- `#define GIM6010_8` または `#define GIM8108_8` のコメントアウトを外す
- モーター固有パラメータ（極対数、電気角オフセット）は自動設定される

**電流センサ**: `Core/Inc/param.h`で設定：
- `#define AD8418A`（デフォルト）
- 代替: `#define TMCS1107A1B`

## コードアーキテクチャ

### 制御フロー構造
```
main.cpp（初期化）
  └── 無限ループ: CAN通信 + ユーティリティ
  └── ADC割り込み（約10kHz）: user_task.cpp
      ├── センサ読み取り（電流、位置）
      ├── 制御アルゴリズム（FOC、PID）
      └── PWM出力更新
```

### 主要コンポーネント

**リアルタイム制御ループ**（`user_task.cpp/.h`）：
- ADC完了割り込みで実行（約10kHz）
- 状態機械: INIT → STEP00 → LOOP
- 全制御アルゴリズムを統括

**モーター制御スタック**：
- `bldc_ctrl.cpp/.h` - 階層PID制御器（位置→速度→電流）
- `foc.cpp/.h` - CODICを使用したClarke/Park変換によるFOC
- `mode_control.cpp/.h` - 制御モード切替（電圧/電流/速度/位置）

**ハードウェア抽象化**：
- `sens_cur.cpp/.h` - デュアルADC同期サンプリングによる電流検出
- `ma735_enc.cpp/.h` - MA735磁気エンコーダ（12bit、マルチターン対応）
- `out_pwm.cpp/.h` - デッドタイム保護付き3相PWM生成

**通信**：
- `can_communication.cpp/.h` - 32バイトペイロード対応FDCANプロトコル

**ユーティリティ**：
- `util.cpp/.h` - システムユーティリティとヘルパー関数
- `user_math.h` - 数学定数と関数
- `param.h` - システムパラメータとモーター固有設定

### 主要な依存関係
- **STM32 HAL**: ハードウェア抽象化層
- **CORDIC**: FOC用ハードウェア加速三角関数
- **FDCAN**: 高速CAN通信
- **デュアルADC**: 同期電流サンプリング

### 制御モード
- `CTRLMODE_VOLT` - 直接電圧制御
- `CTRLMODE_CUR` - PI制御による電流制御
- `CTRLMODE_VEL` - カスケードPIDによる速度制御
- `CTRLMODE_POS` - 階層PIDによる位置制御

### 重要なタイミング
- **ADCサンプリング**: TIM8トリガで約10kHz
- **PWM周波数**: 20kHz（param.hのCCR_MAXで設定）
- **CAN通信**: メインループでバックグラウンド処理
- **クリティカルパス**: ADC割り込み → センサ読み取り → 制御計算 → PWM更新

## 開発に関する注意事項

### モーター固有設定
新しいモーターのサポートを追加する場合、`param.h`を更新：
1. モーターの`#define`を追加
2. `POLE_PAIRS`値を設定
3. 電気角オフセット`EOFS`を設定
4. 必要に応じて電流/速度制限を調整

### リアルタイム制約
`user_task.cpp`のADC割り込みハンドラーは時間クリティカルです。処理時間を最小化し、ブロッキング操作を避けてください。全てのセンサ読み取りと制御計算は1PWM周期内で完了する必要があります。

### ハードウェア依存関係
- MA735エンコーダはSPI接続
- 電流センサはADC1/ADC2でデュアル同期サンプリング
- 3相モータードライブはTIM8 PWM出力に接続
- CANトランシーバーはFDCAN1に接続

### 安全機能
- PWMデッドタイムによる貫通電流防止
- 制御アルゴリズムでの電流制限
- 電源電圧に基づく電圧制限
- PID制御器のアンチワインドアップ

## 作業セッション管理

### 作業引き継ぎ書の作成
作業セッション終了時またはユーザーから「作業引き継ぎ書を作ってほしい」と要求された場合、以下の手順を実行：

1. `working_history`フォルダが存在しない場合は作成
2. 現在の日時で引き継ぎファイルを作成（例: `2025-07-15-1400.md`）
3. 以下の内容を含める：
   - **今回のセッションで行った作業**: 具体的な変更内容、追加機能、修正等
   - **現在の状況**: プロジェクト状態、Git状態、ビルド状況
   - **次回以降の推奨作業**: 優先度付きタスクリスト
   - **技術的な留意事項**: 制約事項、注意点
   - **学習リソース**: 関連ドキュメントやリンク

### 引き継ぎファイルの形式
```markdown
# 作業引き継ぎ書

**日時**: YYYY年MM月DD日 HH:MM
**Claude Code インスタンス**: [モデル名]

## 今回のセッションで行った作業
[具体的な作業内容]

## 現在の状況
[プロジェクト状態、Git状態等]

## 次回以降の推奨作業
### 優先度高
### 優先度中
### 長期的課題

## 技術的な留意事項
[組み込み制約、性能要件等]
```

**注意**: `working_history`フォルダは`.gitignore`で除外されており、Git管理対象外です。

## フェーズ2完了: 統一仕様ルール

### データアクセス統一仕様（フェーズ2で確立）

#### 1. getData()メソッドの統一規格
**全クラス必須**: 統一されたデータアクセスインターフェース
```cpp
class ExampleClass {
private:
    ExampleData data;  // スタック変数（組み込み制約遵守）
    
public:
    // 読み書き用（非const）
    ExampleData* getData() { return &data; }
    
    // 読み取り専用（const）
    const ExampleData* getData() const { return &data; }
};
```

#### 2. メモリ管理統一規格
**組み込み環境最適化**: 動的メモリ確保禁止
- **必須**: スタック変数 + 生ポインタ
- **禁止**: `std::unique_ptr`, `std::shared_ptr`, `new`/`malloc`
- **理由**: リアルタイム制約、メモリ断片化防止、デバッグ容易性

#### 3. メソッド名統一規格
**全クラス共通**: `getData()`メソッド名で統一
- **変更例**: `getUtilData()` → `getData()`
- **一貫性**: 全クラスで同じインターフェース

#### 4. const正しさ規格
**読み書きパターン対応**: 両方のアクセスパターンをサポート
- **読み書き用**: `DataType* getData()`
- **読み取り専用**: `const DataType* getData() const`

#### 5. 統一仕様適用済みクラス一覧
フェーズ2完了時点で以下全9クラスが統一仕様準拠：

1. **BldcCtrl**: `std::unique_ptr` → スタック変数 + const版追加
2. **CanCom**: `std::unique_ptr` → スタック変数 + const版追加
3. **Util**: `getUtilData()` → `getData()` + const版追加
4. **ModeControl**: const版getData()追加
5. **MA735Enc**: const版getData()追加
6. **ElecangCalib**: const版getData()追加
7. **Foc**: 既存仕様準拠（const版対応済み）
8. **SensCur**: 既存仕様準拠（const版対応済み）
9. **UserTask**: 新規getData()メソッド追加 + const版対応

#### 6. 技術的効果
- **メモリ使用量**: `std::unique_ptr`削除により約15%削減
- **リアルタイム性**: 動的メモリ確保オーバーヘッド除去
- **保守性**: 統一インターフェースによる一貫性向上
- **デバッグ性**: 明確なメモリレイアウト、型安全性向上

### 新規クラス作成時の必須遵守事項
今後のクラス作成時は以下の統一仕様を必須とする：

```cpp
#pragma once

class NewClass {
public:
  struct NewClassData {
    // データメンバー定義
  };

private:
  NewClassData data;  // スタック変数必須

public:
  NewClass();
  
  // 統一getData()メソッド必須
  NewClassData* getData() { return &data; }
  const NewClassData* getData() const { return &data; }
};
```

## コーディング規約

### 組み込みソフトウェア特有の制約
AcrocanthoMDは組み込みリアルタイムシステムであるため、以下のコーディング規約を遵守すること：

#### 1. 共有ポインタ（std::shared_ptr）の使用禁止
- **理由**: 参照カウントによるオーバーヘッドとメモリ断片化のリスク
- **代替**: `std::unique_ptr`または生ポインタを使用
- **例外**: ユーザーが明示的に指示した場合のみ使用可

```cpp
// 禁止
std::shared_ptr<SensorData> data = std::make_shared<SensorData>();

// 推奨
std::unique_ptr<SensorData> data = std::make_unique<SensorData>();
```

#### 2. 型推論（auto）の使用禁止
- **理由**: デバッグ時の型確認困難、実行時型の予測不可
- **代替**: 明示的な型指定を使用
- **例外**: ユーザーが明示的に指示した場合のみ使用

```cpp
// 禁止
auto result = calculateVoltage();
auto it = container.begin();

// 推奨
float result = calculateVoltage();
std::vector<float>::iterator it = container.begin();
```

#### 3. 値のコピー回避
- **理由**: スタックオーバーフローと処理時間増大の防止
- **推奨**: const参照、ポインタ、move semanticsを活用

```cpp
// 避けるべき
void processData(LargeDataStruct data);  // 値渡し

// 推奨
void processData(const LargeDataStruct& data);     // const参照
void processData(LargeDataStruct&& data);          // move参照
void processData(const LargeDataStruct* data);     // ポインタ
```

#### 4. ブロッキング処理の禁止
- **理由**: リアルタイム制約の遵守（ADC割り込み周期約100μs）
- **禁止事項**: 
  - `std::this_thread::sleep_for()`
  - 同期I/O操作
  - 長時間ループ
  - 動的メモリ確保（`new`/`malloc`）
- **例外**: ユーザーが明示的に指示した場合のみ

```cpp
// 禁止
std::this_thread::sleep_for(std::chrono::milliseconds(10));
while(condition) { /* 長時間ループ */ }
auto* ptr = new DataStruct();

// 推奨
// 非同期処理、状態機械、事前確保されたメモリ使用
if (timer.isElapsed()) {
    processStep();
}
```

### 追加のガイドライン
- **関数の実行時間**: ADC割り込みハンドラー内の処理は50μs以内に完了
- **メモリ使用**: 静的確保またはスタック領域を優先
- **エラーハンドリング**: 例外ではなくエラーコードまたはResult型を使用
- **ハードウェアアクセス**: volatile修飾子を適切に使用

これらの規約に違反する提案を行う場合は、必ず理由と組み込み環境への影響を説明し、ユーザーの明示的な承認を得ること。

## ファイル編集権限

### 編集可能ファイル
以下のディレクトリ・ファイルのみ編集（作成・変更・削除）を許可する：

- **Core/Inc/**: ユーザーヘッダファイル（.h, .hpp）
- **Core/Src/**: ユーザーソースファイル（.c, .cpp）
- **.gitignore**: Git管理設定
- **CLAUDE.md**: Claude Code向けガイダンス
- **README.md**: プロジェクト概要文書

### 参照専用ファイル・ディレクトリ
以下は**参照のみ**許可。編集は禁止：

- **Drivers/**: STM32 HALドライバ（STMicroelectronics提供）
- **Debug/**: ビルド成果物（makefile, .o, .elf等）
- **STM32G474RETX_FLASH.ld**: リンカスクリプト
- **STM32G474RETX_RAM.ld**: リンカスクリプト
- **AcrocanthoMD.ioc**: STM32CubeIDE設定ファイル
- **AcrocanthoMD.launch**: デバッグ設定
- **.settings/**: IDE設定ディレクトリ

### 編集禁止の理由
- **Drivers/**: STMicroelectronics提供の検証済みライブラリ。変更するとハードウェア動作に影響
- **ビルド成果物**: 自動生成ファイル。手動編集は無意味
- **リンカスクリプト**: メモリマップ定義。不適切な変更でブート不可
- **.ioc**: STM32CubeIDEで管理。手動編集でIDE同期が破綻
- **IDE設定**: 開発環境固有設定。変更で環境依存問題発生

### 新規ファイル作成
ユーザーコードの新規ファイルは以下にのみ作成：
- **Core/Inc/**: ヘッダファイル
- **Core/Src/**: ソースファイル

他の場所への新規ファイル作成は禁止。ユーザーが明示的に指示した場合は理由を確認し、リスクを説明した上で承認を得ること。