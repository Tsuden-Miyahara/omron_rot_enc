

#define ENC_A 2
#define ENC_B 3
#define ENC_Z 4

#define SER   5
#define RCLK  6
#define SRCLK 7

#define STATUS_LENGTH 9
#define LENGTH 10

volatile int encoder_cnt = 0;                     //エンコーダカウント用変数
volatile int encoder_rotate_cnt = 0;              //エンコーダ回転数カウント用変数

byte status[STATUS_LENGTH] = {
  0b11111111,
  0b01111111,
  0b00111111,
  0b00011111,
  0b00001111,
  0b00000111,
  0b00000011,
  0b00000001,
  0b00000000
};

const byte digits[LENGTH] = {
  0b00111111,   //ZERO
  0b00000110,   //ONE
  0b01011011,   //TWO
  0b01001111,   //THREE
  0b01100110,   //FOUR
  0b01101101,   //FIVE
  0b01111101,   //SIX
  0b00000111,   //SEVEN
  0b01111111,   //EIGHT
  0b01101111,   //NINE
};

inline void new_digitalWrite(uint8_t pin, uint8_t val) {
  uint8_t bit = digitalPinToBitMask(pin);
  volatile uint8_t *out = portOutputRegister(digitalPinToPort(pin));
  if (val == LOW)
    *out &= ~bit;
  else
    *out |= bit;  
}
void new_shiftOut(uint8_t dataPin, uint8_t clockPin, byte val) {
  uint8_t i;
  uint8_t bit_data = digitalPinToBitMask(dataPin);
  uint8_t bit_clock = digitalPinToBitMask(clockPin);
  volatile uint8_t *out_data = portOutputRegister(digitalPinToPort(dataPin));
  volatile uint8_t *out_clock = portOutputRegister(digitalPinToPort(clockPin));
  for (i = 0; i < 8; i++)  {
    if(val & (1 << i)) {
      *out_data |= bit_data;
    } else {
      *out_data &= ~bit_data;
    }
    *out_clock |= bit_clock;
    *out_clock &= ~bit_clock;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_pulse, RISING);

  pinMode(SRCLK, OUTPUT);
  pinMode(RCLK , OUTPUT);
  pinMode(SER  , OUTPUT);

  // _display(0xff, 0xff);
  // delay(1500);
}

void _display(byte bin1, byte bin2) {
    new_digitalWrite(RCLK, LOW);
    // shiftOut(SER, SRCLK, LSBFIRST, bin2);
    // shiftOut(SER, SRCLK, LSBFIRST, bin1);
    new_shiftOut(SER, SRCLK, bin2);
    new_shiftOut(SER, SRCLK, bin1);
    new_digitalWrite(RCLK, HIGH);
}

volatile byte n1, n2;
void display(uint8_t lv) {
  if (lv < 100) {
    n1 = digits[lv % 10];
    n2 = lv < 10 ? 0 : digits[(lv / 10) % 10];
  }
  else {
    // n1 = digits[9] | 0b10000000;
    // n2 = digits[9];
    n1 = 0;
    n2 = 0;
  }
  // n = lv < 0 ? (digits[0] | 0b10000000) : 9 < lv ? (digits[9] | 0b10000000) : digits[lv];
  /*
  if (lv < 0) {
    n = (digits[0] | 0b10000000);
  }
  else if (9 < lv) {
    n = (digits[9] | 0b10000000);
  }
  else {
    n = digits[lv];
  }
  */
  _display(n1, n2);
}

volatile uint16_t x = 0;
void _up() {
  x = 499 <= x ? 499 : x + 1;
}
void _down() {
  x = x == 0 ? 0 : x - 1;
}

void encoder_pulse() {
  if(digitalRead(ENC_B)==0){
    encoder_cnt++;                    //エンコーダカウントをインクリメント
      _up();
    if(digitalRead(ENC_Z)==0){    //Z相がLOWのとき（原点にいるとき）
      encoder_rotate_cnt++;           //回転数をインクリメント
    }
  }else{
    encoder_cnt--;                    //エンコーダカウントをデクリメント
      _down();
    if(digitalRead(ENC_Z)==0){    //Z相がLOWのとき（原点にいるとき）
      encoder_rotate_cnt--;           //回転数をデクリメント
    }
  }
}

void loop() {
  Serial.print(encoder_rotate_cnt);
  Serial.print(", ");
  Serial.println(encoder_cnt);       //エンコーダカウントをPCに出力
  display(x / 5);
  //delay(5);
}