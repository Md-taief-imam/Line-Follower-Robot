#include <QTRSensors.h>

#define Kp .1    // experment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 4     // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
#define Ki 0
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   30     // emitter is controlled by digital pin 2

#define rightMotor1 24
#define rightMotor2 25
#define rightMotorPWM 4
#define leftMotor1 26
#define leftMotor2 27
#define leftMotorPWM 6
#define motorPower 8
#define NUM_SAMPLES_PER_SENSOR 4
//QTRSensorsRC qtrrc((unsigned char[]) { 0,1,2,3,4,5,6,7} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void turn_right()
{
  digitalWrite(48, HIGH);
  digitalWrite(49, HIGH);
  digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 30);
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 30);
}

void turn_left() {
  digitalWrite(48, HIGH);
  digitalWrite(49, HIGH);
  digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 30);
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 30);
}


void turn_rightEX()
{
  digitalWrite(48, HIGH);
  digitalWrite(49, HIGH);
  digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 300);
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 300);
}

void turn_leftEX() {
  digitalWrite(48, HIGH);
  digitalWrite(49, HIGH);
  digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 300);
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 300);
}

void go_straightEX() {
   digitalWrite(48, HIGH);
    digitalWrite(49, HIGH);
    digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, 300);
    digitalWrite(motorPower, HIGH);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, 300);
}

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);

  pinMode(49, OUTPUT);
  pinMode(48, OUTPUT);

  int i;
  for ( i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  { qtra.calibrate();
    //comment this part out for automatic calibration
    if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
      turn_right();
    else
      turn_left();
  }
  digitalWrite(13, HIGH);    // turn on LED to indicate we are in calibration mode
  /*for (int i = 0; i < 50; i++)  // make the calibration take about 10 seconds
    {
     qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }*/
  digitalWrite(13, LOW);
  delay(20);
  wait();
  delay(2000); // wait for 2s to position the bot before entering the main loop

  //comment out for serial printing

  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

}

int lastError = 0;
int cumError = 0;

void loop()
{



  for (int i = 0; i < NUM_SENSORS; i++)
  {
    //Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();


  unsigned int sensors[8];
  int position = qtra.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;
  Serial.print(position);

  cumError += error;
  int motorSpeed = Kp * error + Kd * (error - lastError) + Ki * cumError;
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  if(position <1200)
  turn_leftEX();
  else if(position>5800&&position<7000)
  turn_rightEX();
  /*else if(position ==7000)
  go_straightEX();*/
  else
  {

    digitalWrite(48, HIGH);
    digitalWrite(49, HIGH);
    digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);
    digitalWrite(motorPower, HIGH);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  }
}

void wait() {
  digitalWrite(motorPower, LOW);
}
