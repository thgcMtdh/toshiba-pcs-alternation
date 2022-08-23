# 東芝製PCSを改造してPV入力電圧を可変できるようにする
## 経緯
研究室同期の実験で、1kW級PVストリングの出力端電圧を変化できる昇圧チョッパが必要になったので、制作を請け負った。

## 手法
東芝製PCS「TPV-PCS0400B」の中古品をヤフオクにて3000円+送料で落札した。中身が生きていることが確認できたので、昇圧チョッパのゲート信号をArduinoから入力し、デューティー比をArduinoで制御することで、PV入力電圧を可変できるようにする。

## シリアル通信API仕様
シリアル通信で以下の文字列を送信することで、デューティー比の指定や電圧電流測定値の読み出しができる。
- 通信速度は **9600bps** を設定
- 改行コードは **LFのみ** にすること
- コマンド
  - `getV` : PV入力電圧[mV]を取得
  - `getI` : PV入力電流[mA]を取得
  - `getU` : DCリンク電圧[mV]を取得
  - `getD` : 実際にPWM出力しているデューティー比[%]を取得
  - `getS` : PWM信号の出力元を取得。0:PCSマイコンのPWM信号を利用 1:Arduinoで生成したPWM信号を利用
    - 連系運転開始から1分後に、ゲートに入力されるPWM信号がPCSマイコン由来からArduino由来に切り替わる。したがって、getSを呼び出すことで連系運転の開始と終了を検出できる
  - `setDxx` : デューティー比[%]を `xx` % に指定。`xx` は最小 0 ～ 最大 80 の範囲
- 返り値
  - getコマンド：数値単体が文字列で返ってくる。不正なコマンドでは`0`が返ってくる
  - setコマンド：setした数値が文字列で返ってくる。不正な値をセットしようとしたときは`0`が返ってくる