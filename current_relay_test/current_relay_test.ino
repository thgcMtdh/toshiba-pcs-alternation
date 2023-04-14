#define PIN_VREF 9
#define PIN_VOUT 10

uint16_t analogVout = 128;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, HIGH);
  Serial.println("start!");
}

void loop() {
  analogWrite(PIN_VREF, 128);  // Vref
  analogWrite(PIN_VOUT, analogVout);  // Vout
}