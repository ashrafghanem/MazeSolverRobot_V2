#include <SoftwareSerial.h>
SoftwareSerial hc06(2, 4);

// IR Sensors
int sensor1 = 12;      // Left most sensor
int sensor2 = 11;
int sensor3 = 10;
int sensor4 = 9;      // Right most sensor

// Initial Values of Sensors (Left to Right)
int sensor[4] = {0, 0, 0, 0};
char path[20];
char optPath[20];
char flippedOptPath[20];
int arrSize = 20;
int i, j = 0;

// Right Motor
int ENA = 3;
int motorInput1 = 13;
int motorInput2 = 7;

// Left Motor
int motorInput3 = 6;
int motorInput4 = 8;
int ENB = 5;

//Initial Speed of Motor
int initial_motor_speed = 85;

// Output Pins for Led
//int ledPin1 = A3;
//int ledPin2 = A4;

// PID Constants
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int flag = 0;

void flipOptimizedPath() {
  int k = 0;
  for (int i = arrSize - 1; i >= 0; i--) {
    if (optPath[i] == 'S')
      flippedOptPath[k++] = 'N';
    else if (optPath[i] == 'N')
      flippedOptPath[k++] = 'S';
    else if (optPath[i] == 'E')
      flippedOptPath[k++] = 'W';
    else if (optPath[i] == 'W')
      flippedOptPath[k++] = 'E';
  }
}

void optimizePath() {
  int i = 0;
  int index = 0;
  while (true) {
    if (arrSize == index)
      break;
    if (i == arrSize) {
      arrSize = index;
      i = index = 0;
      continue;
    }
    optPath[index] = path[i];

    if (path[i] == 'S' && path[i + 1] == 'N' || path[i] == 'N' && path[i + 1] == 'S') {
      i += 2;
      optPath[index] = path[i];
    }
    else if (path[i] == 'W' && path[i + 1] == 'E' || path[i] == 'E' && path[i + 1] == 'W') {
      i += 2;
      optPath[index] = path[i];
    }

    i++;
    index++;
  }
}

void updatePath(char prevDirection, String currState) {
//  hc06.write(prevDirection);
//  hc06.write('\n');

  if (prevDirection == 'N' && currState == "right") {
    path[i++] = 'E';
  }
  else if (prevDirection == 'N' && currState == "left") {
    path[i++] = 'W';
  }
  else if (prevDirection == 'N' && currState == "reverse") {
    path[i++] = 'S';
  }
  else if (prevDirection == 'N' && currState == "forward") {
    path[i++] = 'N';
  }

  else if (prevDirection == 'S' && currState == "right") {
    path[i++] = 'W';
  }
  else if (prevDirection == 'S' && currState == "left") {
    path[i++] = 'E';
  }
  else if (prevDirection == 'S' && currState == "reverse") {
    path[i++] = 'N';
  }
  else if (prevDirection == 'S' && currState == "forward") {
    path[i++] = 'S';
  }

  else if (prevDirection == 'E' && currState == "right") {
    path[i++] = 'S';
  }
  else if (prevDirection == 'E' && currState == "left") {
    path[i++] = 'N';
  }
  else if (prevDirection == 'E' && currState == "reverse") {
    path[i++] = 'W';
  }
  else if (prevDirection == 'E' && currState == "forward") {
    path[i++] = 'E';
  }

  else if (prevDirection == 'W' && currState == "right") {
    path[i++] = 'N';
  }
  else if (prevDirection == 'W' && currState == "left") {
    path[i++] = 'S';
  }
  else if (prevDirection == 'W' && currState == "reverse") {
    path[i++] = 'E';
  }
  else if (prevDirection == 'W' && currState == "forward") {
    path[i++] = 'W';
  }
}

void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  //pinMode(ledPin1, OUTPUT);
  //pinMode(ledPin2, OUTPUT);
  //digitalWrite(ledPin1, LOW);
  //digitalWrite(ledPin2, LOW);

  Serial.begin(9600);
  hc06.begin(38400);
  delay(500);
  Serial.println("Started !!");
  delay(1000);

  // initially the first path is considered North
  path[i++] = 'N';
}
void loop()
{
  read_sensor_values();
  for(int m=0;m<20;m++){
    hc06.write(path[m]);
  }
  hc06.println('\n');

  Serial.println(error);
  if (error == 100) {
    // Make left turn untill it detects straight path
    updatePath(path[i - 1], "left");
    do {
      read_sensor_values();
      analogWrite(ENA, 85);
      analogWrite(ENB, 85);
      sharpLeftTurn();
    } while (error != 0);

  } else if (error == 101) {
    // Make right turn in case it detects only right path (it will go into forward
    // direction in case of staright and right "|--") untill it detects straight path.
    analogWrite(ENA, 70);
    analogWrite(ENB, 90);
    forward();
    delay(200);
    stop_bot();
    read_sensor_values();
    if (error == 102) {
      updatePath(path[i - 1], "right");
      do {
        analogWrite(ENA, 85);
        analogWrite(ENB, 85);
        sharpRightTurn();
        read_sensor_values();
      } while (error != 0);
    } else {
      // straight
      updatePath(path[i - 1], "forward");
    }

  } else if (error == 102) {
    // Make left turn untill it detects straight path (U Turn)
    updatePath(path[i - 1], "reverse");
    do {
      analogWrite(ENA, 100);
      analogWrite(ENB, 100);
      sharpLeftTurn();
      read_sensor_values();
      if (error == 0) {
        stop_bot();
        delay(200);
      }
    } while (error != 0);

  } else if (error == 103) {
    // Make left turn untill it detects straight path or stop if dead end reached.
    if (flag == 0) {
      analogWrite(ENA, 70);
      analogWrite(ENB, 90);
      forward();
      delay(200);
      stop_bot();
      read_sensor_values();
      if (error == 103) {
        /**** Dead End Reached, Stop! ****/
        stop_bot();
        //digitalWrite(ledPin1, HIGH);
        //digitalWrite(ledPin2, HIGH);
        flag = 1;

      } else {
        /**** Move Left ****/
        updatePath(path[i - 1], "left");
        analogWrite(ENA, 85);
        analogWrite(ENB, 85);
        sharpLeftTurn();
        delay(200);
        do {
          read_sensor_values();
          analogWrite(ENA, 85);
          analogWrite(ENB, 85);
          sharpLeftTurn();
        } while (error != 0);
      }
    }

  } else {
    calculate_pid();
    motor_control();
  }
}

void read_sensor_values()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);

  // 1000
  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 3;

  // 1100
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 2;

  // 0100
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 1;

  // 0110
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0))
    error = 0;

  // 0010
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0))
    error = -1;

  // 0011
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1))
    error = -2;

  // 0001
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1))
    error = -3;

  // 1110
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // Turn robot left side
    error = 100;

  // 0111
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn robot right side
    error = 101;

  // 0000
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Make U turn
    error = 102;

  // 1111
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn left side or stop
    error = 103;
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(ENA, right_motor_speed - 20);
  analogWrite(ENB, left_motor_speed);

  //following lines of code are to make the bot move forward
  forward();
}

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
