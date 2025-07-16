# AcrocanthoMD

## 仕様書
- [CANFDプロトコル仕様書](README_IF.md)

## 概要
AcrocanthoMDは、STM32G4マイコンを使用したブラシレスDCモーター制御システムです。フィールド指向制御（FOC）技術を用いて高精度なサーボモーター制御を実現します。

## システムアーキテクチャ

### 制御システム概要
```
CAN指令 → モード制御 → BLDC制御 → FOC → PWM出力
    ↑                                    ↓
MA735エンコーダ ← 電流センサ ← モーター応答
```

### 主要コンポーネント

#### 1. **制御ループ構造**
- **メインループ**: `main.cpp` - 初期化と無限ループ
- **リアルタイム制御**: `user_task.cpp` - ADC割り込みで駆動（約10kHz）
- **状態機械**: INIT → STEP00 → LOOP の順次実行

#### 2. **ハードウェア抽象化層**
- **電流センサ**: `sens_cur.cpp/.h` - AD8418A対応、デュアルADC同期サンプリング
- **位置センサ**: `ma735_enc.cpp/.h` - MA735磁気エンコーダ、12bit分解能
- **PWM出力**: `out_pwm.cpp/.h` - 3相PWM生成、デッドタイム保護

#### 3. **モーター制御アルゴリズム**
- **FOC実装**: `foc.cpp/.h` - Clarke/Park変換、CORDIC使用
- **制御ループ**: `bldc_ctrl.cpp/.h` - 階層PID制御（位置→速度→電流）
- **モード制御**: `mode_control.cpp/.h` - 複数制御モード対応

#### 4. **通信プロトコル**
- **CAN通信**: `can_communication.cpp/.h` - FDCAN、32バイトペイロード
- **制御指令**: 電圧・電流・速度・位置の各モード対応

## ファイル関係性

### 依存関係グラフ
```
main.cpp
├── user_task.cpp (制御オーケストレーション)
    ├── mode_control.cpp (制御モード管理)
    │   ├── can_communication.cpp (CAN通信)
    │   ├── bldc_ctrl.cpp (PID制御)
    │   └── util.cpp (ユーティリティ)
    ├── foc.cpp (FOC変換)
    │   ├── sens_cur.cpp (電流センサ)
    │   ├── ma735_enc.cpp (エンコーダ)
    │   └── user_math.h (数学関数)
    └── out_pwm.cpp (PWM出力)
```

### 各ファイルの役割

#### Core/Inc & Core/Src
- **main.cpp/.h**: システム初期化、ペリフェラル設定
- **user_task.cpp/.h**: 制御タスク管理、状態機械
- **bldc_ctrl.cpp/.h**: 階層PID制御器実装
- **foc.cpp/.h**: フィールド指向制御変換
- **ma735_enc.cpp/.h**: MA735エンコーダ制御
- **sens_cur.cpp/.h**: 電流センサ処理
- **out_pwm.cpp/.h**: PWM生成・出力制御
- **can_communication.cpp/.h**: CAN通信プロトコル
- **mode_control.cpp/.h**: 制御モード切替
- **util.cpp/.h**: ユーティリティ機能
- **param.h**: システムパラメータ定義
- **user_math.h**: 数学関数・定数定義

## 制御フロー

### 高優先度リアルタイムループ（ADC割り込み）
```
ADC完了 → 電流センサ読み取り → 角度読み取り → 
制御アルゴリズム → FOC → PWM更新
```

### 低優先度バックグラウンドループ（メインループ）
```
CAN通信 → ユーティリティ機能 → モード管理
```

## 対応モーター
- **GIM6010_8**: 14極対、±10A電流、±9rad/s速度
- **GIM8108_8**: 21極対、±10A電流、±5rad/s速度

## 制御モード
- **CTRLMODE_VOLT**: 電圧制御
- **CTRLMODE_CUR**: 電流制御
- **CTRLMODE_VEL**: 速度制御
- **CTRLMODE_POS**: 位置制御

## 主要機能
- 12bit分解能位置センサ
- マルチターン対応
- CORDIC ハードウェア加速
- デッドタイム保護
- アンチワインドアップPID
- 電圧・電流制限
- サーボチェック機能