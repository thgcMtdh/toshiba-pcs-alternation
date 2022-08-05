#include <Arduino.h>
//#include <avr/interrupt.h>
#include <avr/io.h>

#define PIN_PCS_PWM 8    // PCSからのゲートPWM信号
#define PIN_RELAY 9      // ゲート入力切替リレー
#define PIN_PWM 10       // ゲートPWM信号OC1B。PB2から出力され、PB2は基板上でpin10につながっている
#define PIN_PHOTOD A0    // 連系LED検出用フォトダイオードからのアナログ入力
#define PIN_SENSE_I A1   // 電流センサからのアナログ入力
#define PIN_SENSE_PV A2  // PV入力電圧の分圧入力
#define PIN_SENSE_DC A3  // DCリンク電圧の分圧入力

#define FREQ_PWM 8000  // PWM周波数[Hz]
#define MAX_DUTY 80
#define MIN_DUTY 0
#define MAX_SERIAL_BUF 32

uint32_t voltage_dc = 15;  // 計測したDCリンク電圧[mV]
uint32_t voltage_pv = 15;  // 計測した入力電圧[mV]
uint32_t current_pv = 30;  // 計測した入力電流[mA]
uint16_t val_photod = 0;   // フォトダイオードの読み値
uint8_t dutyCommand = 0;   // シリアルで受信したデューティー比の指令値[%]
uint8_t dutyActual = 0;    // 実際に昇圧チョッパのIGBTを駆動するデューティー比[%]

// ISR(TIMER1_OVF_vect)
// {
// }

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_PCS_PWM, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  // PWM設定
  TCCR1A = 0b00110001;              // 比較1A出力=なし,比較1B出力=反転出力,OCR1AをTOPとする位相/周波数基準PWM動作
  TCCR1B = 0b00010001;              // 捕獲入力=無効,OCR1AをTOPとする位相/周波数基準PWM動作,分周なし
  OCR1A = 16000000 / 2 / FREQ_PWM;  // TOP値を設定
  // TIMSK1 = 0b00000001;              // タイマ/カウンタ1溢れ割り込み許可(BOTTOM値でOVF割り込みがかかる)
}

void loop()
{
  // 電圧電流計測
  voltage_dc = ((uint32_t)analogRead(PIN_SENSE_DC) * 443892) / 1000;   // analogRead()/1024 * 5.0V * 300/3.3 * 1000 [mV]
  voltage_pv = ((uint32_t)analogRead(PIN_SENSE_PV) * 443892) / 1000;   // analogRead()/1024 * 5.0V * 300/3.3 * 1000 [mV]
  current_pv = ((uint32_t)analogRead(PIN_SENSE_I) * 837296) / 100000;  // analogRead()/1024 * 5.0V * 0.986/46.0 * 1000 * 80mA/mV [mA]
  Serial.print(voltage_dc);
  Serial.print(",");
  Serial.print(voltage_pv);
  Serial.print(",");
  Serial.println(current_pv);

  // 連系運転検知用フォトダイオードの読みを取得
  val_photod = analogRead(PIN_PHOTOD);

  // 実出力Duty比設定 (本来は過電流/過電圧保護を考慮して0に落としたりする)
  dutyActual = dutyCommand;

  // duty比反映
  OCR1B = 16000000 / 2 / FREQ_PWM / 100 * dutyActual;

  // シリアル通信
  static char buf[MAX_SERIAL_BUF];  // シリアル受信文字列を格納するバッファ
  static size_t i = 0;              // シリアル受信文字列バッファの何文字目に書き込んでいるか

  while (Serial.available()) {  // シリアルから値を読んでバッファに転記
    buf[i] = Serial.read();
    i++;
    if (i >= MAX_SERIAL_BUF) {
      i = 0;
    }
  }

  if (buf[i - 1] == '\n') {  // bufの末尾が改行コードだった場合、コマンドがそこで終わるので、デコード開始

    if (buf[0] == 'g' && buf[1] == 'e' && buf[2] == 't') {  // get要求

      if (buf[3] == 'V') {           // 続く文字が'V'
        Serial.println(voltage_pv);  // mV単位でprint
      } else if (buf[3] == 'I') {    // 続く文字が'I'
        Serial.println(current_pv);  // mA単位でprint
      } else if (buf[3] == 'D') {    // 続く文字が'D'
        Serial.println(dutyActual);  // duty比をprint
      } else {                       // 続く文字がどれにも当てはまらない
        Serial.println("0");         // '0'を送信
      }

    } else if (buf[0] == 's' && buf[1] == 'e' && buf[2] == 't') {  // set要求

      if (buf[3] == 'D') {  // 続く文字がD: デューティー比
        uint8_t newduty = 0;
        switch (i) {  // setD + 数値 + 改行コード の文字数で分岐
          case 6:     // 数値が1文字のとき
            if (isNumber(buf[4])) {
              newduty = buf[4] - '0';
            }
            break;
          case 7:  // 数値が2文字のとき
            if (isNumber(buf[4]) && isNumber(buf[5])) {
              newduty = 10 * (buf[4] - '0') + buf[5] - '0';
            }
            break;
          case 8:  // 数値が3文字のとき
            if (isNumber(buf[4]) && isNumber(buf[5]) && isNumber(buf[6])) {
              newduty = 100 * (buf[4] - '0') + 10 * (buf[5] - '0') + buf[6] - '0';
            }
          default:
            break;
        }
        if (newduty < MIN_DUTY) newduty = MIN_DUTY;
        if (newduty > MAX_DUTY) newduty = MAX_DUTY;

        dutyCommand = newduty;        // 受け取った値を代入
        Serial.println(dutyCommand);  // 代入した値を返す

      } else {                // 続く文字がどれにも当てはまらない
        Serial.println("0");  // 0を返す
      }
    } else {                // getでもsetでもない何か
      Serial.println("0");  // 0を返す
    }
    i = 0;                        // インデックスをクリア
    memset(buf, 0, sizeof(buf));  // バッファをクリア
  }

  delay(1);
}

/**
 * @brief 文字が数字'0'～'9'であるか判定する
 * @param c 文字
 * @return true: 数字を表す文字列である, false: 数字以外を表す文字列である
 */
bool isNumber(char c)
{
  return (c >= '0' && c <= '9');
}