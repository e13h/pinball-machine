#include <Servo.h>

const int LEFT_FLIPPER_BUTTON_PIN = 8;
const int LEFT_FLIPPER_SOLENOID_PIN = 2;
const int MOTOR_PIN = 3;
const int MOTOR_PROX_SENSOR_PIN = 4;
const int SCOREBOARD_SER_PIN = 7;
const int SCOREBOARD_RCLK_PIN = 6;
const int SCOREBOARD_SRCLK_PIN = 5;
const int SCOREBOARD_LEFT_DIGIT_PIN = 13;
const int SCOREBOARD_RIGHT_DIGIT_PIN = 12;
const int START_BUTTON_PIN = 52;
const int RIGHT_PIEZO_BUMPER_PIN = A0;
const int BUZZER_PIN = 9;
const int DROPOUT_PROX_SENSOR_PIN = 50;
const int SERVO_PIN = 10;

enum GameplayState {
  NOT_STARTED,
  ROUND_1,
  ROUND_1_ENDED,
  ROUND_2,
  ROUND_2_ENDED,
  ROUND_3,
  ROUND_3_ENDED,
  GAME_OVER
};

struct Round {
  unsigned long timestamp_start = 0;
  unsigned int points = 0;
  unsigned int total_points = 0;
};

struct Scoreboard {
  Round round_1;
  Round round_2;
  Round round_3;
  unsigned int grand_total = 0;
  Round* current_round = &round_1;
};

struct MotorController {
  const unsigned int TIMEOUT = 3000;  // milliseconds
  const uint8_t SPEED = 200;
  bool enabled = false;
  unsigned long timestamp_enabled = 0;
};

struct ProximitySensorController {
  const unsigned int TIMEOUT = 250;  // milliseconds
  unsigned long timestamp_tripped = 0;
};

struct PiezoBumperController {
  const unsigned int TIMEOUT = 150;  // milliseconds
  const unsigned int POINTS = 3;
  const unsigned int SENSITIVITY = 75;
  unsigned long timestamp_tripped = 0;
};

struct BuzzerController {
  const unsigned int TIMEOUT = 75;  // milliseconds
  const unsigned int SPEED = 200;
  bool active = false;
  unsigned long timestamp_active = 0;
};

struct ServoController {
  const unsigned int UNLOCK_TIMEOUT = 3000;  // milliseconds
  const unsigned int WRITE_TIMEOUT = 15;  // milliseconds
  const unsigned int LOCKED_POS = 0;
  const unsigned int UNLOCKED_POS = 120;
  unsigned long timestamp_unlocked = 0;
  unsigned long timestamp_last_write = 0;
  bool unlocked = false;
  Servo device;
};

struct PinballMachine {
  bool scoreboardLeftDigitToggle = false;
  MotorController motor;
  Scoreboard score;
  GameplayState state = NOT_STARTED;
  ProximitySensorController motorProxSensor;
  PiezoBumperController rightBumper;
  BuzzerController buzzer;
  ServoController servo;
};

PinballMachine machine;

void displayDigit(byte);
void updateScoreboard(int, int);
void driveMotor();
void driveLeftSolenoid();
void driveRightSolenoid();
void updateScore();
void activateBuzzer();
void driveBuzzer();
void unlockServo();
void driveServo();
void checkForDropout();
void transitionToNextRound(GameplayState);
void printDebugStatements(unsigned long loopIterationTime);

void setup() {
  pinMode(LEFT_FLIPPER_BUTTON_PIN, INPUT);
  pinMode(LEFT_FLIPPER_SOLENOID_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SCOREBOARD_RCLK_PIN, OUTPUT);
  pinMode(SCOREBOARD_SER_PIN, OUTPUT);
  pinMode(SCOREBOARD_SRCLK_PIN, OUTPUT);
  pinMode(SCOREBOARD_LEFT_DIGIT_PIN, OUTPUT);
  pinMode(SCOREBOARD_RIGHT_DIGIT_PIN, OUTPUT);
  pinMode(MOTOR_PROX_SENSOR_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(RIGHT_PIEZO_BUMPER_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(DROPOUT_PROX_SENSOR_PIN, INPUT);
  machine.servo.device.attach(SERVO_PIN);
  Serial.begin(115200);
}

unsigned long last = millis();
void loop() {
  unsigned long now = millis();
  switch (machine.state) {
    case NOT_STARTED:
      updateScoreboard(10, 1);  // 'r' '1' for round 1
      if (digitalRead(START_BUTTON_PIN) == HIGH) {
        transitionToNextRound(ROUND_1);
      }
      break;
    case ROUND_1_ENDED:
      updateScoreboard(10, 2);  // 'r' '2' for round 2
      if (digitalRead(START_BUTTON_PIN) == HIGH) {
        transitionToNextRound(ROUND_2);
      }
      break;
    case ROUND_2_ENDED:
      updateScoreboard(10, 3);  // 'r' '3' for round 3
      if (digitalRead(START_BUTTON_PIN) == HIGH) {
        transitionToNextRound(ROUND_3);
      }
      break;
    case ROUND_1:
    case ROUND_2:
    case ROUND_3:
      driveLeftSolenoid();
      driveMotor();
      driveBuzzer();
      updateScore();
      updateScoreboard(machine.score.current_round->total_points / 10, machine.score.current_round->total_points % 10);
      checkForDropout();
      break;
    case GAME_OVER:
      updateScoreboard(machine.score.grand_total / 10, machine.score.grand_total % 10);
      break;
  }
  driveServo();
  // if (millis() % 100 == 0) {
  //   printDebugStatements(now - last);
  // }
  last = now;
}

void transitionToNextRound(GameplayState round) {
  machine.state = round;
  if (round == ROUND_1) {
    machine.score.current_round = &machine.score.round_1;
  } else if (round == ROUND_2) {
    machine.score.current_round = &machine.score.round_2;
  } else if (round == ROUND_3) {
    machine.score.current_round = &machine.score.round_3;
  } else {
    machine.state = GAME_OVER;
    return;
  }
  machine.score.current_round->timestamp_start = millis();
  unlockServo();
}

void updateScoreboard(int leftDigit, int rightDigit) {
  const byte INT_2_7SEG[] = {
    0b11111100,  // 0
    0b01100000,  // 1
    0b11011010,  // 2
    0b11110010,  // 3
    0b01100110,  // 4
    0b10110110,  // 5
    0b10111110,  // 6
    0b11100000,  // 7
    0b11111110,  // 8
    0b11110110,  // 9
    0b00001010,  // r
  };

  if (machine.scoreboardLeftDigitToggle) {
    digitalWrite(SCOREBOARD_RIGHT_DIGIT_PIN, LOW);
    // int leftDigit = machine.score.current_round->total_points / 10;
    displayDigit(INT_2_7SEG[leftDigit]);
    digitalWrite(SCOREBOARD_LEFT_DIGIT_PIN, HIGH);
  } else {
    digitalWrite(SCOREBOARD_LEFT_DIGIT_PIN, LOW);
    // int rightDigit = machine.score.current_round->total_points % 10;
    displayDigit(INT_2_7SEG[rightDigit]);
    digitalWrite(SCOREBOARD_RIGHT_DIGIT_PIN, HIGH);
  }
  machine.scoreboardLeftDigitToggle = !machine.scoreboardLeftDigitToggle;
}

void displayDigit(byte digit) {
  digitalWrite(SCOREBOARD_RCLK_PIN, LOW);
  shiftOut(SCOREBOARD_SER_PIN, SCOREBOARD_SRCLK_PIN, LSBFIRST, digit);
  digitalWrite(SCOREBOARD_RCLK_PIN, HIGH);
}

void driveMotor() {
  if (digitalRead(MOTOR_PROX_SENSOR_PIN) == LOW) {
    machine.motor.enabled = true;
    machine.motor.timestamp_enabled = millis();
  }
  if (machine.motor.enabled && millis() - machine.motor.timestamp_enabled > machine.motor.TIMEOUT) {
    // Turn off the motor after X seconds of activity
    machine.motor.enabled = false;
  }
  if (machine.motor.enabled) {
    analogWrite(MOTOR_PIN, machine.motor.SPEED);
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void driveLeftSolenoid() {
  if (digitalRead(LEFT_FLIPPER_BUTTON_PIN) == HIGH) {
    digitalWrite(LEFT_FLIPPER_SOLENOID_PIN, HIGH);
  } else {
    digitalWrite(LEFT_FLIPPER_SOLENOID_PIN, LOW);
  }
}

void driveRightSolenoid() {

}

void updateScore() {
  if (digitalRead(MOTOR_PROX_SENSOR_PIN) == LOW
      && millis() - machine.motorProxSensor.timestamp_tripped > machine.motorProxSensor.TIMEOUT) {
    (machine.score.current_round->points)++;
    activateBuzzer();
    machine.motorProxSensor.timestamp_tripped = millis();
  }
  if (analogRead(RIGHT_PIEZO_BUMPER_PIN) > machine.rightBumper.SENSITIVITY
      && millis() - machine.rightBumper.timestamp_tripped > machine.rightBumper.TIMEOUT) {
    machine.score.current_round->points += machine.rightBumper.POINTS;
    activateBuzzer();
    machine.rightBumper.timestamp_tripped = millis();
  }
  unsigned int time_score = (millis() - machine.score.current_round->timestamp_start) / 2000;
  machine.score.current_round->total_points = machine.score.current_round->points + time_score;

  machine.score.grand_total = machine.score.round_1.total_points + machine.score.round_2.total_points + machine.score.round_3.total_points;
}

void activateBuzzer() {
  machine.buzzer.active = true;
  machine.buzzer.timestamp_active = millis();
}

void driveBuzzer() {
  if (machine.buzzer.active && millis() - machine.buzzer.timestamp_active < machine.buzzer.TIMEOUT) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else if (machine.buzzer.active) {
    machine.buzzer.active = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void unlockServo() {
  machine.servo.unlocked = true;
  machine.servo.timestamp_unlocked = millis();
}

void driveServo() {
  if (!machine.servo.unlocked
      && millis() - machine.servo.timestamp_last_write > machine.servo.WRITE_TIMEOUT) {
    machine.servo.device.write(machine.servo.LOCKED_POS);
    machine.servo.timestamp_last_write = millis();
  } else if (machine.servo.unlocked
      && millis() - machine.servo.timestamp_unlocked < machine.servo.UNLOCK_TIMEOUT
      && millis() - machine.servo.timestamp_last_write > machine.servo.WRITE_TIMEOUT) {
    machine.servo.device.write(machine.servo.UNLOCKED_POS);
    machine.servo.timestamp_last_write = millis();
  } else if (machine.servo.unlocked
      && millis() - machine.servo.timestamp_unlocked > machine.servo.UNLOCK_TIMEOUT) {
    machine.servo.unlocked = false;
  }
}

void checkForDropout() {
  // if (digitalRead(DROPOUT_PROX_SENSOR_PIN) == LOW) {
  if (digitalRead(START_BUTTON_PIN) == HIGH && millis() - machine.score.current_round->timestamp_start > 1000) {  // TODO: Remove this once IR sensor is fixed
    if (machine.state == ROUND_1) {
      machine.state = ROUND_1_ENDED;
    } else if (machine.state == ROUND_2) {
      machine.state = ROUND_2_ENDED;
    } else if (machine.state == ROUND_3) {
      machine.state = GAME_OVER;
    }
    delay(1000);  // TODO: Remove this
  }
}

void printDebugStatements(unsigned long loopIterationTime) {
  Serial.print(loopIterationTime);
  Serial.print(" prox1:");
  Serial.print(digitalRead(MOTOR_PROX_SENSOR_PIN));
  Serial.print(" motor:");
  Serial.print(machine.motor.enabled);
  Serial.print(" lflip:");
  Serial.print(digitalRead(LEFT_FLIPPER_BUTTON_PIN));
  Serial.print(" start:");
  Serial.print(digitalRead(START_BUTTON_PIN));
  Serial.print(" rpiezo:");
  Serial.print(analogRead(RIGHT_PIEZO_BUMPER_PIN));
  Serial.print(" drop:");
  Serial.print(digitalRead(DROPOUT_PROX_SENSOR_PIN));
  Serial.print(" r1_score:");
  Serial.print(machine.score.round_1.total_points);
  Serial.print(" r2_score:");
  Serial.print(machine.score.round_2.total_points);
  Serial.print(" r3_score:");
  Serial.print(machine.score.round_3.total_points);
  Serial.print(" total:");
  Serial.println(machine.score.grand_total);
}
