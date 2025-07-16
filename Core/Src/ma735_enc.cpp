#include "ma735_enc.h"
#include <cstring>

#include "main.h"
#include "param.h"
#include "can_communication.h"

// main.cppと同じインスタンスを使用
extern SPI_HandleTypeDef hspi1;

MA735Enc ma735enc(hspi1);
extern CanCom cancom;

MA735Enc::MA735Enc(SPI_HandleTypeDef& spiHandle)
  : hspi1(spiHandle), readStart(false), actAngle(0.0f), spi_rx_complete(false), spi_tx_complete(false),
    diffRaw(0) {}

bool MA735Enc::ma735Init(){
  static uint8_t count = 0;
  // 初回のみreadで直読み
  if (count == 0) read(CMD_MA735_READ);
  getAngle();
  zeroPosOffset();
  count++;

  if (count > 3) return true;
  return false;
};

// MGH/MGLレジスタを読込取付距離のチェックに使う。
// MGH: 0, MGL: 0 適正
// MGH: 1, MGL: 0 近すぎる
// MGH: 0, MGL: 1 遠すぎる
void MA735Enc::magFieldTh(){
  read(CMD_MA735_MGLHT);
  if (spi_rx_complete) {
    // CS非アクティブ
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    
    data.rawAng = (static_cast<uint16_t>(rawEnc[1] >> 6) & 0b00000011);
    // 磁界強度チェック用のビットフィールド抽出
    // uint8_t mght = (rawEnc[1] >> 2) & 0b00000111;
    // uint8_t mglt = (rawEnc[1] >> 5) & 0b00000111;

    data.rawAngtest = data.rawAng;
    
    readStart = false;
    spi_rx_complete = false;
  }
}

void MA735Enc::ma735angle(){
  getAngle();
  getVel();
  elecAngleIn();
  mechAngleIn();
};

void MA735Enc::read(uint16_t reg) {
  if (!readStart) {
    uint16_t command = reg;
    
    // CSアクティブLow
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    //HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&command, (uint8_t*)rawEnc, 2);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&command, (uint8_t*)rawEnc, 2);
    //HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&command, 2);
    //HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)&command, 2);
    //HAL_SPI_Receive_IT(&hspi1, (uint8_t*)rawEnc, 2);
    readStart = true;
  }
}

void MA735Enc::getAngle() {
  if (spi_rx_complete) {
    // CS非アクティブ
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    data.rawAngPast = data.rawAng;
    // 16ビットデータを受信し、後ろ4ビットを削除して12ビットに変換
    data.rawAng = (static_cast<uint16_t>(rawEnc[1] << 8) | rawEnc[0]) >> 4;

    data.rawAngtest = data.rawAng;
    
    readStart = false;
    spi_rx_complete = false;
    // 速度時間割りのためのカウンタ保存
    compTime = comp;
    comp = 0;

    read(CMD_MA735_READ);
  } else {
    ++comp;
  }
  data.eleccomp = comp;
}

void MA735Enc::getVel() {
  if (comp) {
    // 更新なしのため速度更新不要
    return;
  }
  
  // 速度計算
  diff = static_cast<int16_t>(data.rawAng - data.rawAngPast);

  // 0またぎ判定処理
  if (diff > ANG_RESL_HALF) {
    diff -= ANG_RESL_12BIT;
    zeroPointTh = MULT_TURN_NEG;
  } else if (diff < -ANG_RESL_HALF) {
    diff += ANG_RESL_12BIT;
    zeroPointTh = MULT_TURN_POS;
  }
  
  // 電気角用に符号反転前のdiffを保存
  diffRaw = diff;
  
  // 制御量と出力量の符号をあわせる
  if (mechAngDir == 0) {
    diff = -diff;
  }
  
  // CWとCCWをユーザー任意の方向にあわせる
  if (rotDir > 0) {
    diff = -diff; // 極性反転
  }

  data.mechAngVel = raw2rads(diff);
  mechAngleVelLPF();
  data.testdiff = diff;
}

void MA735Enc::mechAngleVelLPF(){
  const float alpha = kalpha;
  data.mechAngVelLPF = (alpha * data.mechAngVel + (1.0f - alpha) * data.mechAngVelLPF) * GR_RATIO;
}

float MA735Enc::elecAng(float _eofs) {
  // comp = 0のときサンプル値更新あり
  if (comp == 0) {
    rawElecComp = data.rawAng;
  } else {
    rawElecComp = data.rawAng + (diffRaw / 2);
  }
  
  // 電気角反転
  uint16_t elecAngtemp;
  if (elecAngDir > 0) {
    elecAngtemp = rawElecComp;
  } else {
    elecAngtemp = ANG_RESL_12BIT - rawElecComp;
  }

  // CWとCCWを切替
  const float ofs = (rotDir > 0) ? (_eofs + user2pi) : _eofs;
  
  const uint16_t offset = static_cast<uint16_t>(ofs * invResol / user2pi * ANG_RESL_12BIT);
  const uint16_t elecAngtemp2 = (elecAngtemp * POLE_PAIRS + offset) % ANG_RESL_12BIT;

  return static_cast<float>(elecAngtemp2) * invResol * user2pi;
}

float MA735Enc::elecAngVirtual(float _virFreqRef) {
  // 仮想電気角
  static float theta_ = 0.0f;
  // タスク周期で分割して足しこむ
  float _deltatheta = _virFreqRef * TASK_TIME;
  theta_ += _deltatheta;

  // 0~1の範囲に収める
  if (theta_ > 1.0f) theta_ -= 1.0f;
  if (theta_ < 0.0f) theta_ += 1.0f;

  // ラジアンで返却
  return theta_ * user2pi;
}

void MA735Enc::elecAngleIn(){
  CanCom::CanData* candata = cancom.getData();
  
  data.elecAngTest = elecAng(EOFS);
  if (candata->virAngFreq > 0.0f) {
    data.elecAng = elecAngVirtual(candata->virAngFreq);
  } else {
    data.elecAng = elecAng(EOFS);
  }
}

void MA735Enc::mechAngleIn() {
  // シングルターン値をラジアンに変換（0-2πの範囲）
  const float tempMtAng = raw2rad(data.rawAng);
  
  // 一回転の判定とカウント更新
  if (zeroPointTh == MULT_TURN_NEG) {
    mtCount--;
  } else if (zeroPointTh == MULT_TURN_POS) {
    mtCount++;
  }
  
  // マルチターン分を加算（2πの倍数を加算）
  data.mechAng = (tempMtAng - data.zeroPosOffs + (static_cast<float>(mtCount) * user2pi)) * GR_RATIO;
  
  // 制御量と出力量の符号をあわせる
  if (mechAngDir == 0) {
    data.mechAng = -data.mechAng;
  }
  
  // CWとCCWをユーザー任意の方向にあわせる
  if (rotDir > 0) {
    data.mechAng = -data.mechAng; // 極性反転
  }
  
  // マルチターンフラグリセット
  zeroPointTh = MULT_TURN_NONE;
}

void MA735Enc::zeroPosOffset() {
  data.zeroPosOffs = raw2rad(data.rawAng);
}

void MA735Enc::spiRxCallback() {
  spi_rx_complete = true;
}

void MA735Enc::spiTxCallback() {
  // 現在未使用
}

// SPIコールバック関数
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    ma735enc.spiRxCallback();
  }
}
