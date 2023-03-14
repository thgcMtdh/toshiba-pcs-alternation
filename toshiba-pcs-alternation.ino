#include <Arduino.h>
#include <avr/io.h>

// #define DEBUG  // 本番ではコメントアウト、デバッグprintするときは宣言する

// #define GRID_TIED  // チョッパ部のみで運転するときはコメントアウト。連系運転するときは宣言

#define PIN_GRID_SW 2     // 連系SW
#define PIN_PCS_PWM 3     // PCSからのゲートPWM信号
#define PIN_DC_RELAY 4    // 保護ボックスのDCリレー指令線
#define PIN_GATE_RELAY 9  // ゲート入力切替リレー
#define PIN_PWM 10        // ゲートPWM信号OC1B。PB2から出力され、PB2は基板上でpin10につながっている
#define PIN_PHOTOD A0     // 連系LED検出用フォトダイオードからのアナログ入力
#define PIN_SENSE_I A1    // 電流センサからのアナログ入力
#define PIN_SENSE_PV A2   // PV入力電圧の分圧入力
#define PIN_SENSE_DC A3   // DCリンク電圧の分圧入力

#define SAMPLE_NUM 32           // 電圧・電流計測の平均化サンプリング数
#define GRID_LED_THRESHOLD 200  // 連系LED点灯判定閾値(0～1023)
#define WAIT_TIME 60000         // 連系LED点灯後、こちらからゲート制御を始めるまでの待機時間[ms]
#define FREQ_PWM 20000          // PWM周波数[Hz]。実測の結果20kHzだったので合わせた
#define MAX_DUTY 80
#define MIN_DUTY 0
#define MAX_SERIAL_BUF 32

#define OVP_SET_MV 385000  // 過電圧フラグをセットする電圧[mV]
#define OCP_SET_MA 4000    // 過電流フラグをセットする電流[mA]

enum ProtectionBit {
  OV,  // 過電圧検出
  OC   // 過電流検出
};

uint16_t voltage_dc_raw[SAMPLE_NUM];
uint16_t voltage_pv_raw[SAMPLE_NUM];
uint16_t current_pv_raw[SAMPLE_NUM];

uint32_t voltage_dc = 15;  // 計測したDCリンク電圧[mV]
uint32_t voltage_pv = 15;  // 計測した入力電圧[mV]
uint32_t current_pv = 30;  // 計測した入力電流[mA]

unsigned int val_photod = 0;            // フォトダイオードの読み値
unsigned long grid_connected_time = 0;  // 連系開始時刻[ms]

uint8_t duty_received = 0;   // シリアルで受信したデューティー比の指令値[%]
bool grid_sw_received = 0;   // シリアルで受信した連系指令(false:OFF, true:ON)
bool dc_relay_received = 0;  // シリアルで受信した保護DCリレー指令(false:OFF, true:ON)

uint8_t duty_command = 0;     // 実際に昇圧チョッパのIGBTを駆動するデューティー比[%]
bool grid_sw_command = 0;     // 連系スイッチON/OFF指令(false:OFF, true:ON)
bool dc_relay_command = 0;    // 保護ボックスのDCリレーON/OFF指令(false:OFF, true:ON)
bool gate_relay_command = 0;  // ゲート入力切替リレー指令(false:PCS側素通し, true:自前の信号利用)

uint16_t pwmfreq = 0;           // PCSのPWM周波数計測結果[Hz]
volatile uint16_t counter = 0;  // PCSのPWM周波数計測用のカウンター

uint8_t protection_flag = 0;  // 保護フラグ

ISR(TIMER1_OVF_vect)
{
}

void count_pwmfreq()
{
  counter++;
}

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_GRID_SW, OUTPUT);
  pinMode(PIN_PCS_PWM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PCS_PWM), count_pwmfreq, FALLING);
  pinMode(PIN_GATE_RELAY, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_GRID_SW, LOW);
  digitalWrite(PIN_GATE_RELAY, LOW);
  // PWM設定
  TCCR1A = 0b00110001;              // 比較1A出力=なし,比較1B出力=反転出力,OCR1AをTOPとする位相/周波数基準PWM動作
  TCCR1B = 0b00010001;              // 捕獲入力=無効,OCR1AをTOPとする位相/周波数基準PWM動作,分周なし
  OCR1A = 16000000 / 2 / FREQ_PWM;  // TOP値を設定
  // TIMSK1 = 0b00000001;              // タイマ/カウンタ1溢れ割り込み許可(BOTTOM値でOVF割り込みがかかる)

  for (size_t i = 0; i < SAMPLE_NUM; i++) {
    voltage_dc_raw[i] = 0;
    voltage_pv_raw[i] = 0;
    current_pv_raw[i] = 0;
  }
}

void loop()
{
  // 現在時刻と1ループの周期を取得
  static unsigned long period = 0;
  static unsigned long t_old = 0;
  static unsigned long t_now = 0;
  t_old = t_now;
  t_now = millis();
  period = t_now - t_old;

  // 電圧電流計測
  static size_t i_adc = 0;
  voltage_dc_raw[i_adc] = analogRead(PIN_SENSE_DC);
  voltage_pv_raw[i_adc] = analogRead(PIN_SENSE_PV);
  current_pv_raw[i_adc] = analogRead(PIN_SENSE_I);
  i_adc++;
  i_adc = i_adc % SAMPLE_NUM;
  voltage_dc = ((uint32_t)average(voltage_dc_raw, SAMPLE_NUM) * 443892) / 1000;    // analogRead()/1024 * 5.0V * 300/3.3 * 1000 [mV]
  voltage_pv = ((uint32_t)average(voltage_pv_raw, SAMPLE_NUM) * 443892) / 1000;    // analogRead()/1024 * 5.0V * 300/3.3 * 1000 [mV]
  current_pv = ((uint32_t)average(current_pv_raw, SAMPLE_NUM) * 837296) / 100000;  // analogRead()/1024 * 5.0V * 0.986/46.0 * 1000 * 80mA/mV [mA]

  // PCSのPWM周波数計測(確認用)
  static unsigned long t_print_pwmfreq = 0;
  if (t_now - t_print_pwmfreq > 1000) {
    pwmfreq = counter;  // 約1secおきにcounterの値を確認して周波数とし、リセット
    counter = 0;
    t_print_pwmfreq = t_now;
  }

  // --------------------
  // 保護判定とDCリレー解列
  // --------------------

  bool ov, oc;
  ov = (voltage_dc > OVP_SET_MV);
  oc = (current_pv > OCP_SET_MA);

  // 保護フラグのセット
  if (ov) {
    bitSet(protection_flag, OV);
  }
  if (oc) {
    bitSet(protection_flag, OC);
  }

  // 保護フラグのリセットは、DCリレー指令がOFFに戻ったとき行う
  if (!ov && !oc && dc_relay_received == false) {
    protection_flag = 0;
  }

  // 保護フラグが立っていないときに限りDCリレーをONする
  if (dc_relay_received == true && protection_flag == 0) {
    dc_relay_command = true;
  } else {
    dc_relay_command = false;
  }

#ifdef GRID_TIED

  // --------------------------
  // 連系運転を利用する場合の処理
  // --------------------------

  // 連系SWは指令に従う
  grid_sw_command = grid_sw_received;

  // 連系運転検知用フォトダイオードの読みを取得
  unsigned int new_val_photod = analogRead(PIN_PHOTOD);

  // 連系LEDの消灯→点灯を検知したら時刻を記録し、点灯→消灯を検知したら時刻をクリア
  if (val_photod < GRID_LED_THRESHOLD && new_val_photod >= GRID_LED_THRESHOLD) {
    grid_connected_time = t_now;
  } else if (val_photod >= GRID_LED_THRESHOLD && new_val_photod < GRID_LED_THRESHOLD) {
    grid_connected_time = 0;
  }
  val_photod = new_val_photod;

  // 連系LED点灯からWAIT_TIME以上経過していたら、ゲート信号をArduinoから出力する
  if (val_photod >= GRID_LED_THRESHOLD && t_now - grid_connected_time >= WAIT_TIME) {
    gate_relay_command = true;
    if (protection_flag) {  // 保護フラグが立っているときはDuty比ゼロ(ゲートブロック)
      duty_command = 0;
    } else {
      duty_command = duty_received;
    }

    // 連系LED消灯時はPCSのゲート信号を素通しする
  } else {
    gate_relay_command = false;
    duty_command = 0;
  }

#else

  // --------------------------------
  // チョッパ部のみで運転する場合の処理
  // --------------------------------

  // 連系しないので連系SWはOFF
  grid_sw_command = false;

  // ゲート信号は常にArduinoから出力
  gate_relay_command = true;

  // ゲートのduty比を反映。保護フラグが立っているときはDuty比ゼロ(ゲートブロック)
  if (protection_flag) {
    duty_command = 0;
  } else {
    duty_command = duty_received;
  }

#endif

  // 指令の反映
  OCR1B = 16000000 / 2 / FREQ_PWM / 100 * duty_command;  // duty比反映
  digitalWrite(PIN_GRID_SW, grid_sw_command);            // 連系SW
  digitalWrite(PIN_GATE_RELAY, gate_relay_command);      // ゲート切替リレー
  digitalWrite(PIN_DC_RELAY, dc_relay_command);          // 保護ボックスDCリレー

#ifdef DEBUG
  Serial.print(voltage_dc);
  Serial.print(",");
  Serial.print(voltage_pv);
  Serial.print(",");
  Serial.print(current_pv);
  Serial.print(",");
  Serial.println(pwmfreq);
  Serial.print(",");
  Serial.println(new_val_photod);
#endif

  // -----------
  // シリアル通信
  // -----------

  static char buf[MAX_SERIAL_BUF];  // シリアル受信文字列を格納するバッファ
  static size_t i = 0;              // シリアル受信文字列バッファの何文字目に書き込んでいるか

  while (Serial.available()) {  // シリアルから値を読んでバッファに転記
    buf[i] = Serial.read();
    i++;
    if (i >= MAX_SERIAL_BUF) {
      i = 0;
    }
  }

  if (buf[i - 1] == '\n') {    // bufの末尾が改行コードだった場合、コマンドがそこで終わるので、デコード開始
    if (buf[i - 2] != '\r') {  // CRLFだった場合は弾く

      if (buf[0] == 'g' && buf[1] == 'e' && buf[2] == 't') {  // get要求

        if (buf[3] == 'V') {                            // 続く文字が'V'
          Serial.println(voltage_pv);                   // mV単位でprint
        } else if (buf[3] == 'I') {                     // 続く文字が'I'
          Serial.println(current_pv);                   // mA単位でprint
        } else if (buf[3] == 'D') {                     // 続く文字が'D'
          Serial.println(duty_command);                 // duty比をprint
        } else if (buf[3] == 'U') {                     // 続く文字が'U'
          Serial.println(voltage_dc);                   // DCリンク電圧をprint
        } else if (buf[3] == 'S') {                     // 続く文字が'S'
          Serial.println(digitalRead(PIN_GATE_RELAY));  // ゲート入力切替リレーのステータスをprint
        } else if (buf[3] == 'G') {                     // 続く文字が'G'
          Serial.println(digitalRead(PIN_GRID_SW));     // 連系SWへの指令値をprint
        } else if (buf[3] == 'P') {                     // 続く文字が'P'
          Serial.println(protection_flag);              // 保護フラグの値をprint
        } else if (buf[3] == 'R') {                     // 続く文字が'R'
          Serial.println(digitalRead(PIN_DC_RELAY));    // 保護ボックスDCリレーへの指令値をprint
        } else {                                        // 続く文字がどれにも当てはまらない
          Serial.println("0");                          // '0'を送信
        }

      } else if (buf[0] == 's' && buf[1] == 'e' && buf[2] == 't') {  // set要求

        if (buf[3] == 'D') {  // 続く文字がD: デューティー比
          uint8_t newduty = 0;
          switch (i) {  // setD + 数値 + 改行コード\n の文字数で分岐(CRLFだとバグる)
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

          duty_received = newduty;        // 受け取った値を代入
          Serial.println(duty_received);  // 代入した値を返す

        } else if (buf[3] == 'G') {        // 続く文字がG: 連系SW
          uint8_t command = buf[4] - '0';  // '0'か'1'が期待されるので、数値に変換
          if (command == 1) {              // 1だった場合に限り連系SW指令をON。他(0または2以上の値)はすべて0とする
            grid_sw_received = 1;
          } else {
            grid_sw_received = 0;
          }
          Serial.println(grid_sw_received);

        } else if (buf[3] == 'R') {        // 続く文字がR: 保護ボックスDCリレー
          uint8_t command = buf[4] - '0';  // '0'か'1'が期待されるので、数値に変換
          if (command == 1) {              // 1だった場合に限りDCリレーをON。他(0または2以上の値)はすべて0とする
            dc_relay_received = 1;
          } else {
            dc_relay_received = 0;
          }
          Serial.println(dc_relay_received);
        
        } else {                // 続く文字がどれにも当てはまらない
          Serial.println("0");  // 0を返す
        }
      } else {                // getでもsetでもない何か
        Serial.println("0");  // 0を返す
      }
    } else {                // 改行コードがCRLFだった場合
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

/**
 * @brief 配列要素を平均する
 * @param p_array 平均したいデータが入った配列
 * @param num 平均するサンプル数
 */
uint16_t average(uint16_t *p_array, uint16_t num)
{
  uint32_t sum = 0;
  for (size_t i = 0; i < num; i++) {
    sum += p_array[i];
  }
  return (uint16_t)(sum / num);
}
