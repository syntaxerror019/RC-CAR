// Traxxas RC Car control board
// 2024 - Miles Hilliard
// www.mileshilliard.com

#include <Servo.h>

#define VERSION 3.1

#define LPWM 3
#define RPWM 6
#define ENABLE 2
#define INVERT_PWM true     // use if motor spins inverse

#define RX_DEG A0
#define RX_DEG_MIN 1000
#define RX_DEG_MAX 1946
#define RX_DEG_NEUTRAL 1410
#define RX_DEG_THRESH 30

#define SERVO_MAX 140
#define SERVO_MIN 40

#define RX_PWM A5
#define RX_PW_MIN 975       // pulse width in us.
#define RX_PW_MAX 1967      // pulse width in us.
#define RX_PW_NEUTRAL 1470  // neutral width in us (w/o controller or with no input)
#define RX_PW_THRESH 30     // threshold between drive states

#define LED_BRIGHT 255      // led brightness (0-255) (Not supported, no PWM)

#define BATT A4
#define WARN_BATTERY 3.7 * 2
#define LOW_BATTERY 3.5 * 2 // For 2s battery
#define CRITICAL_BATTERY 3.3 * 2
#define CORRECTION 0.2      // Battery voltage correction. +/-

#define RED 12
#define GREEN 11

#define CONTROL_RX_TIMEOUT 1000 // If there isn't control cmds being sent every 1 seconds, it stops.

int RPWM_VAL = 0;
int LPWM_VAL = 0;

float r1 = 10000.0; // Ohms
float r2 = 10000.0; // Ohms

int THROTTLE_CAP = 255;

bool DEBUG = false;

struct PWM {
  int RPWMVAL;
  int LPWMVAL;
  bool ENBL = false;
};

struct DEG {
  int DEGREES;
};

unsigned long last_red_blink = 0;
bool red_blink = 0;

float battery = 0.0;

const int numSamples = 10;  // samples for averaging.
unsigned long pwmValues[numSamples];
int sampleIndex = 0;

int thr_sent = 0;
int str_sent = 0;

unsigned long last_control_rx = 0;
unsigned long last_battery_read = 0;

bool autonomy = false;

Servo servo;

void setup() {
  Serial.begin(9600);

  pinMode(RX_PWM, INPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);

  for (int i = 0; i < numSamples; i++) {
    pwmValues[i] = 0;
  }

  servo.attach(10);
  servo.write(90);
}


void loop() {
  incoming_bytes();
  handle_main();
}


void blink_red() {
  if ((millis() - last_red_blink) > 200) {
      digitalWrite(GREEN, LOW);
      digitalWrite(RED, red_blink);
      red_blink = !red_blink;
      last_red_blink = millis();
  }
}


void red(int pwm = LED_BRIGHT) {
  digitalWrite(GREEN, LOW);
  analogWrite(RED, pwm);
}


void green(int pwm = LED_BRIGHT) {
  digitalWrite(RED, LOW);
  analogWrite(GREEN, pwm);
}


void off() {
  digitalWrite(RED, 0x0);
  digitalWrite(GREEN, 0x0);
}


float read_battery() {
  int raw = analogRead(BATT);
  float vOut = raw * (5.0 / 1023.0);
  float voltage = (vOut * ((r1 + r2) / r2)) + CORRECTION;
  return voltage;
}


void motor_control(PWM throttle) {
  if (throttle.ENBL) {
    digitalWrite(ENABLE, HIGH);
    if (INVERT_PWM) {
      if (throttle.LPWMVAL > 0 && throttle.RPWMVAL == 0) { // FORWARDS
        analogWrite(RPWM, throttle.LPWMVAL);
        analogWrite(LPWM, 0);
      } else if (throttle.RPWMVAL > 0 && throttle.LPWMVAL == 0) { // REVERSE
        analogWrite(LPWM, throttle.RPWMVAL);
        analogWrite(RPWM, 0);
      } else {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
      }
    } else {
      if (throttle.LPWMVAL > 0 && throttle.RPWMVAL == 0) {
        analogWrite(LPWM, throttle.LPWMVAL);
        analogWrite(RPWM, 0);
      } else if (throttle.RPWMVAL > 0 && throttle.LPWMVAL == 0) {
        analogWrite(RPWM, throttle.RPWMVAL);
        analogWrite(LPWM, 0);
      } else {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
      }
    }
  } else {
     digitalWrite(ENABLE, LOW);
     analogWrite(RPWM, 0);
     analogWrite(LPWM, 0);
  }
}


void steering_control(DEG steering) {
  servo.write(steering.DEGREES);
}


unsigned long smooth(unsigned long pulseWidth) {
  pwmValues[sampleIndex] = pulseWidth;
  sampleIndex = (sampleIndex + 1) % numSamples;

  unsigned long total = 0;
  for (int i = 0; i < numSamples; i++) {
    total += pwmValues[i];
  }

  return (total / numSamples);
}


void incoming_bytes() {
  static String inputText = "";
  static bool messageComplete = false;

  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      messageComplete = true;
      break;
    } else {
      inputText += incomingChar;
    }
  }

  if (messageComplete) {
    handle_commands(inputText);
    inputText = "";
    messageComplete = false;
  }
}



void handle_main() {
  unsigned long throttle_pulse_width = pulseIn(RX_PWM, HIGH);
  unsigned long steering_pulse_width = pulseIn(RX_DEG, HIGH);

  PWM throttle = map_throttle(smooth(throttle_pulse_width), throttle_pulse_width);  // get high duty cycle of pwm in and "smooth" signal and pass on
  DEG steering = map_steering(steering_pulse_width);

  if ((millis()-last_battery_read) > 1000) {
    battery = read_battery();
    last_battery_read = millis();
  }

  if (battery < WARN_BATTERY) {
    red();
    THROTTLE_CAP = 200;
  } else if (battery < LOW_BATTERY) {
    blink_red();
    THROTTLE_CAP = 50;
  } else if (battery < CRITICAL_BATTERY) {
    blink_red();
    THROTTLE_CAP = 0;
  } else {
    off();
    THROTTLE_CAP = 255;
  }

  if (thr_sent < 10) {
    motor_control(throttle);
    thr_sent++;
  }

  if (str_sent < 10) {
    steering_control(steering);
    str_sent++;
  }

  if ((millis() - last_control_rx) > CONTROL_RX_TIMEOUT && autonomy) {
    // A miscommunication has occured, stopping.
    DEG str;
    str.DEGREES = 90; //home

    PWM thr;
    thr.RPWMVAL = 0;
    thr.LPWMVAL = 0;
    thr.ENBL = false;

    autonomy = false;

    steering_control(str);
    motor_control(thr);
  }
}


void handle_commands(String command) {
  if (command == "HELP") {
    handle_command_help();
  } else if (command == "DEBUG") {
    handle_command_debug();
  } else if (command.startsWith("CONTROL")) {

    last_control_rx = millis();
    autonomy = true;
    
    int spacePos = command.indexOf(' ') + 1;
    int commaPos = command.indexOf(',');

    int throttle = command.substring(spacePos, commaPos).toInt();
    int steering = command.substring(commaPos + 1).toInt();

    if (DEBUG) {
      Serial.print("THROTTLE:"); Serial.print(throttle); Serial.print("\t STEERING:"); Serial.println(steering);
    }
    
    DEG str;
    PWM thr;

    str.DEGREES = steering;

    if (throttle < 0) {
      thr.RPWMVAL = abs(throttle);
      thr.LPWMVAL = 0;
      thr.ENBL = true;
    } else if (throttle > 0) {
      thr.LPWMVAL = throttle;
      thr.RPWMVAL = 0;
      thr.ENBL = true;
    } else {
      thr.RPWMVAL = 0;
      thr.LPWMVAL = 0;
      thr.ENBL = false;
    }

    steering_control(str);
    motor_control(thr);

    // return; // TODO: make this actually work (will require some hardware mods including servo monitoring...)
  } else if (command == "TEST") {
    test_motors();
  }
}


void handle_command_debug() {
  DEBUG = !DEBUG;
  if (DEBUG == true) {
    Serial.println("DEBUG ON");
  } else {
    Serial.println("DEBUG OFF");
  }
}


void handle_command_help() {
  Serial.println("COMMANDS:\n - HELP: DISPLAY THIS MESSAGE\n - DEBUG: SHOW DEBUGGING INFO\n - CONTROL <int throttle (-255-0, 0-255)>,<int servo (0-180)>: CONTROL THE CAR OVER SERIAL (BETA)\n - TEST: RUNS MOTORS IN PATTERN");
}


PWM map_throttle(unsigned long dur, unsigned long pulseWidth) {
  PWM throttle;
  
  if (pulseWidth < 1) {
    throttle.LPWMVAL = 0;
    throttle.RPWMVAL = 0;
    return throttle;
  }

  if (dur < (RX_PW_NEUTRAL - RX_PW_THRESH)) {  // reverse
    throttle.LPWMVAL = 0;
    throttle.RPWMVAL = min(map(dur, RX_PW_MIN, RX_PW_NEUTRAL, 255, 0), THROTTLE_CAP);
    throttle.ENBL = true;
    thr_sent = 0;
  } else if (dur > (RX_PW_NEUTRAL + RX_PW_THRESH)) {  // forward
    throttle.LPWMVAL = min(map(dur, RX_PW_NEUTRAL, RX_PW_MAX, 0, 255), THROTTLE_CAP);
    throttle.RPWMVAL = 0;
    throttle.ENBL = true;
    thr_sent = 0;
  } else {  // no control (within RX_PW_THRESH)
    throttle.LPWMVAL = 0;
    throttle.RPWMVAL = 0;
    throttle.ENBL = false;
  }

  if (DEBUG) {
    Serial.print("THROTTLE:  LPWM=");
    Serial.print(throttle.LPWMVAL);
    Serial.print("     RPWM=");
    Serial.print(throttle.RPWMVAL);
    Serial.print("\n");
  }

  return throttle;
}

DEG map_steering(unsigned long dur) {
  DEG steering;

  if ((RX_DEG_NEUTRAL + RX_DEG_THRESH) > dur && dur > (RX_DEG_NEUTRAL - RX_DEG_THRESH)) {
    steering.DEGREES = 90;  // stay at neutral
  } else {
    int servoAngle = map(dur, RX_DEG_MIN, RX_DEG_MAX, SERVO_MIN, SERVO_MAX);
    steering.DEGREES = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
    str_sent = 0;
  }
  
  if (DEBUG){
    Serial.print("STEERING: PULSEIN=");
    Serial.print(dur);
    Serial.print("\t DEG=");
    Serial.println(steering.DEGREES);
  }

  return steering;
}


void test_motors() {
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
  digitalWrite(ENABLE, HIGH);
  for (int i = 0; i < 255; i++) {
    analogWrite(LPWM, i); 
    delay(10);
  }
  analogWrite(LPWM, 0);
  delay(1000);
  for (int i = 0; i < 255; i++) {
    analogWrite(RPWM, i);
    delay(10);
  }
  analogWrite(RPWM, 0);
  digitalWrite(ENABLE, LOW);
}
