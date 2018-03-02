#define M_ENABLE_A   9 //пин ШИМа левого мотора
#define M_IN1        8 //пин левого мотора
#define M_IN2        7 //пин левого мотора
#define M_IN3        6 //пин правого мотора
#define M_IN4        4 //пин правого мотора
#define M_ENABLE_B   10 //пин ШИМа правого мотора

#define FAN_1        3 //пин ШИМа турбины

#define LED_GREEN    2 //пин красного светодиода
#define LED_RED      5 //пин зеленого светодиода

#define LEFT_SWITCH  11 //пин левого выключателя
#define RIGHT_SWITCH 12 //пин правого выключателя

#define V_CONTROL    A0 // пин вольтметра

bool IS_CHARGED = true; // статус заряженности аккумулятора
bool IS_STARTED = false; // запуск турбины и начало движения

// резисторы делителя напряжения
const float R1 = 100000.0;  // 100K
const float R2 = 10000.0;;   // 10K

// минимальный допустимый заряд аккумулятора
const double MIN_BATT_VOLTAGE = 13.0;

// эту константу (typVbg) необходимо откалибровать индивидуально
const float typVbg = 1; // 1.0 -- 1.2

float Vcc = 0.0;
float curVoltage;

// Для выравнивания скорости колес
byte MAX_SPEED_LEFT  = 120;
byte MAX_SPEED_RIGHT = 120;
//---------------------------------

byte MIN_SPEED = 0;

void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);  // DEFAULT INTERNAL использовать Vcc как AREF
  delay(100);
  analogWrite(V_CONTROL, 0);
  pinMode(V_CONTROL, INPUT);
  //---------------------------
  randomSeed(analogRead(A7));
  // пины для левого и правого моторов на выход
  pinMode(M_ENABLE_A, OUTPUT);
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_IN3, OUTPUT);
  pinMode(M_IN4, OUTPUT);
  pinMode(M_ENABLE_B, OUTPUT);
  //-------------------------------------------
  // пины левого и правого выключателей на вход
  pinMode(LEFT_SWITCH, INPUT);
  pinMode(RIGHT_SWITCH, INPUT);
  //---------------------------
  // пин турбины
  pinMode(FAN_1, OUTPUT);
  //---------------------------
  // пины светодиодов индикации
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  //---------------------------
  // запуск
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(1000);
  // успешный запуск
  digitalWrite(LED_RED, LOW);
}

void loop() {

  checkBattVoltage();
  // если села батарея выключаем все, зажигаем красный светодиод
  if(!IS_CHARGED) {
    Serial.println("IS_CHARGED = false");
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(FAN_1, LOW);
    delay(200);
    MOVE_STOP();
    delay(200);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
  }

  // для первичного старта
  if(!IS_STARTED) {
    Serial.println("IS_STARTED = false");
    prepareStart();
  }

  // если срабатывает левый выключатель на бампере
  if (digitalRead(LEFT_SWITCH) == HIGH)
  {
    Serial.println("sensor - L");
    MOVE_STOP();
    delay(200);
    MOVE_BACKWARD();
    delay(150);
    MOVE_STOP();
    delay(200);
    ROB_RIGHT();
    delay(random(400, 1500));
    MOVE_STOP();
    delay(200);
    MOVE_FORWARD();
  }
  //-----------------------------------------------
  // если срабатывает правый выключатель на бампере
  if (digitalRead(RIGHT_SWITCH) == HIGH)
  {
    Serial.println("sensor - R");
    MOVE_STOP();
    delay(200);
    MOVE_BACKWARD();
    delay(150);
    MOVE_STOP();
    delay(200);
    ROB_LEFT();
    delay(random(400, 1500));
    MOVE_STOP();
    delay(200);
    MOVE_FORWARD();
  }
}

void prepareStart () {
  Serial.println("Starting");
  delay(3000);
  MOVE_FORWARD();
  delay(3000);
  digitalWrite(FAN_1, HIGH);
  delay(1000);
  analogWrite(FAN_1, 200);
  delay(1000);
  IS_STARTED = true;
}

bool checkBattVoltage () {
  Vcc = readVcc();
  curVoltage= analogRead(V_CONTROL);
  double v  = (curVoltage * Vcc) / 1024.0;
  double v2 = v / (R2 / (R1 + R2));
  if ( v2 < MIN_BATT_VOLTAGE) {
    IS_CHARGED=false;
  }
  Serial.print("V = ");
  Serial.println(v);
  Serial.println(v2);
}

// поворот направо на месте
void ROB_RIGHT() {
  Serial.println("mov - R");
  // левый мотор вперед
  digitalWrite(M_IN1, LOW);
  digitalWrite(M_IN2, HIGH);
  analogWrite(M_ENABLE_A, MAX_SPEED_LEFT);
  // правый мотор назад
  digitalWrite(M_IN3, LOW);
  digitalWrite(M_IN4, HIGH);
  analogWrite(M_ENABLE_B, MAX_SPEED_RIGHT);
}
//-----------------
// поворот налево на месте
void ROB_LEFT() {
  Serial.println("mov - L");
  // правый мотор вперед
  digitalWrite(M_IN3, HIGH);
  digitalWrite(M_IN4, LOW);
  analogWrite(M_ENABLE_B, MAX_SPEED_RIGHT);
  // левый мотор назад
  digitalWrite(M_IN1, HIGH);
  digitalWrite(M_IN2, LOW);
  analogWrite(M_ENABLE_A, MAX_SPEED_LEFT);
}
//---------------------
// езда вперед
void MOVE_FORWARD() {
  Serial.println("mov - F");
  // левый мотор вперед
  digitalWrite(M_IN1, LOW);
  digitalWrite(M_IN2, HIGH);
  analogWrite(M_ENABLE_A, MAX_SPEED_LEFT);
  // правый мотор вперед
  digitalWrite(M_IN3, HIGH);
  digitalWrite(M_IN4, LOW);
  analogWrite(M_ENABLE_B, MAX_SPEED_RIGHT);
}
//-------------------------------------
// езда назад
void MOVE_BACKWARD() {
  Serial.println("mov - R");
  // левый мотор назад
  digitalWrite(M_IN1, HIGH);
  digitalWrite(M_IN2, LOW);
  analogWrite(M_ENABLE_A, MAX_SPEED_LEFT);
  // правый мотор назад
  digitalWrite(M_IN3, LOW);
  digitalWrite(M_IN4, HIGH);
  analogWrite(M_ENABLE_B, MAX_SPEED_RIGHT);
}
//------------------------------------
// стоп
void MOVE_STOP() {
  Serial.println("mov - S");
  // левый мотор стоп
  digitalWrite(M_IN1, LOW);
  digitalWrite(M_IN2, LOW);
  analogWrite(M_ENABLE_A, MIN_SPEED);
  // правый мотор стоп
  digitalWrite(M_IN3, LOW);
  digitalWrite(M_IN4, LOW);
  analogWrite(M_ENABLE_B, MIN_SPEED);
}
//--------------------------------
// вольтметр
float readVcc() {
  byte i;
  float result = 0.0;
  float tmp = 0.0;

  for (i = 0; i < 5; i++) {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
        // works on an Arduino 168 or 328
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    delay(3); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    tmp = (high<<8) | low;
    tmp = (typVbg * 1023.0) / tmp;
    result = result + tmp;
    delay(5);
  }

  result = result / 5;
  return result;
}
//--------------------------------

