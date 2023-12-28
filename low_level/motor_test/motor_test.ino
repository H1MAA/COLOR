// ================================================================
// ===                      Main Defines                        ===
// ================================================================

#include <Timer.h>
#include <PIDF.h>

// Motor and Encoder
#define ENC_A_M1 26
#define ENC_B_M1 27
#define ENC_A_M2 12
#define ENC_B_M2 13

#define M1_1 10
#define M1_2 9
#define M1_EN 5

#define M2_1 8
#define M2_2 7
#define M2_EN 6

// PID Parameters
#define Kp_1 7
#define Ki_1 1
#define Kd_1 0.02

#define Kp_2 0
#define Ki_2 0
#define Kd_2 0

// Sampling time and period in ms
#define Ts 0.01
#define period 10

#define radius 0.325  //in cm

MIA::PIDF velocity_pid[] = { MIA::PIDF(Kp_1, Ki_1, Kd_1, Ts),
                             MIA::PIDF(Kp_2, Ki_2, Kd_2, Ts) };

#define MIN_OUTPUT -255
#define MAX_OUTPUT 255

Timer myTimer;

double ppr = 2000;
long long counts[2] = { 0, 0 };      // Array to store counts for two motors
long long lastcounts[2] = { 0, 0 };  // Array to store last counts for two motors

double speed[2] = { 0, 0 };     // Array to store speed for two motors
double setpoint[2] = {0, 0 };  // Array to store setpoint for two motors

double pwm_value[2] = { 0, 0 };  // Array to store PWM value for two motors



void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  init_pid();


  pinMode(ENC_A_M1, INPUT_PULLUP);
  pinMode(ENC_B_M1, INPUT_PULLUP);
  pinMode(ENC_A_M2, INPUT_PULLUP);
  pinMode(ENC_B_M2, INPUT_PULLUP);

  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  digitalWrite(M1_1, LOW);
  digitalWrite(M1_2, LOW);
  digitalWrite(M2_1, LOW);
  digitalWrite(M2_2, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_A_M1), ISR_encoderPinA_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_M1), ISR_encoderPinB_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_M2), ISR_encoderPinA_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_M2), ISR_encoderPinB_M2, CHANGE);

  myTimer.every(period, pidRoutine);
}

long long debug_millis = 0;


void loop() {
  // put your main code here, to run repeatedly:
  myTimer.update();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the data until a newline character is received
    get_command(input);
  }

  if (millis() - debug_millis >= 1000) {
    debug_millis = millis();
    Serial.print("Speed M1: ");
    Serial.print(speed[0]);
    Serial.print(" - Speed M2: ");
    Serial.println(speed[1]);
  }
}

void init_pid() {
  for (int i = 0; i < 2; i++) {
    velocity_pid[i].limitOutput(MIN_OUTPUT, MAX_OUTPUT);
    velocity_pid[i].setpoint(setpoint[i]);
  }
}

void pidRoutine() {
  for (int i = 0; i < 2; i++) {

    speed[i] = (counts[i] - lastcounts[i]) * radius / (ppr * Ts);

    pwm_value[i] = velocity_pid[i].calculate(speed[i]);

    if (velocity_pid[i].setpoint() == 0) {
      velocity_pid[i].integralAccumulator = 0;
    }

    if (velocity_pid[i].setpoint() == 0 && abs(velocity_pid[i].setpoint() - speed[i]) <= 1)
      pwm_value[i] = 0;

    lastcounts[i] = counts[i];
  }

  analogWrite(M1_EN, pwm_value[0]);
  analogWrite(M2_EN, pwm_value[1]);
}


// speed = ((counts - lastcounts) / ppr) / (1.0 * (period) / 1000.0) * 60;

// // Serial.println(speed);

// pwm_value = velocity_pid[0].calculate(speed);

// if (velocity_pid[0].setpoint() == 0) {
//   velocity_pid[0].integralAccumulator = 0;
// }

// if (velocity_pid[0].setpoint() == 0 && abs(velocity_pid[0].setpoint() - speed) <= 1)
//   pwm_value = 0;
// else if (pwm_value > 0) {
//   digitalWrite(M1, HIGH);
//   digitalWrite(M2, LOW);
// } else if (pwm_value < 0) {
//   digitalWrite(M1, LOW);
//   digitalWrite(M2, HIGH);
// }
// analogWrite(en, pwm_value);

// lastcounts = counts;





void ISR_encoderPinA_M1() {
  if (digitalRead(ENC_A_M1) == digitalRead(ENC_B_M1)) {
    counts[0]++;
  } else {
    counts[0]--;
  }
}

void ISR_encoderPinB_M1() {
  if (digitalRead(ENC_A_M1) != digitalRead(ENC_B_M1)) {
    counts[0]++;
  } else {
    counts[0]--;
  }
}

void ISR_encoderPinA_M2() {
  if (digitalRead(ENC_A_M2) == digitalRead(ENC_B_M2)) {
    counts[1]++;
  } else {
    counts[1]--;
  }
}

void ISR_encoderPinB_M2() {
  if (digitalRead(ENC_A_M2) != digitalRead(ENC_B_M2)) {
    counts[1]++;
  } else {
    counts[1]--;
  }
}

int countCommas(String input) {
  int count = 0;
  for (int i = 0; i < input.length(); i++) {
    if (input.charAt(i) == ',') {
      count++;
    }
  }
  return count;
}

void get_command(String input) {

  int commaCount = countCommas(input);

  if (commaCount == 0) {
    // Assume a single integer as setpoint for both motors
    setpoint[0] = input.toDouble();
    setpoint[1] = input.toDouble();
    velocity_pid[0].setpoint(setpoint[0]);
    velocity_pid[1].setpoint(setpoint[1]);
    Serial.print("Received Setpoint: ");
  }

  else if (commaCount == 5) {
    // Four comma-separated values for P, I, D for both motors
    int firstCommaIndex = input.indexOf(',');                        // Find the first comma
    int secondCommaIndex = input.indexOf(',', firstCommaIndex + 1);  // Find the second comma
    int thirdCommaIndex = input.indexOf(',', secondCommaIndex + 1);  // Find the third comma
    int fourthCommaIndex = input.indexOf(',', thirdCommaIndex + 1);  // Find the fourth comma
    int fifthCommaIndex = input.indexOf(',', fourthCommaIndex + 1);

    if (firstCommaIndex != -1 && secondCommaIndex != -1 && thirdCommaIndex != -1 && fourthCommaIndex != -1 && fifthCommaIndex != -1) {
      String p1 = input.substring(0, firstCommaIndex);                         // Extract the first P value
      String i1 = input.substring(firstCommaIndex + 1, secondCommaIndex + 1);  // Extract the first I value
      String d1 = input.substring(secondCommaIndex + 1, thirdCommaIndex);      // Extract the first D value
      String p2 = input.substring(thirdCommaIndex + 1, fourthCommaIndex);      // Extract the second P value
      String i2 = input.substring(fourthCommaIndex + fifthCommaIndex);         // Extract the second I value
      String d2 = input.substring(fifthCommaIndex + 1);


      double kp1 = p1.toDouble();
      double ki1 = i1.toDouble();
      double kd1 = d1.toDouble();
      double kp2 = p2.toDouble();
      double ki2 = i2.toDouble();
      double kd2 = d2.toDouble();

      velocity_pid[0].tune(kp1, ki1, kd1);
      velocity_pid[1].tune(kp2, ki2, kd2);

      Serial.print("Received PID Values - Motor 1: ");
      Serial.print(kp1);
      Serial.print(", ");
      Serial.print(ki1);
      Serial.print(", ");
      Serial.print(kd1);
      Serial.print(" - Motor 2: ");
      Serial.print(kp2);
      Serial.print(", ");
      Serial.print(ki2);
      Serial.print(",");
      Serial.println(kd2);
    }
  }
}
