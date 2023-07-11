#include <QTRSensors.h>
#define KP 0.0
#define KD 0.0
#define MAX_SPEED 240//MAX PWM SPEED MOTORS CAN GET
#define DEFAULT_SPEED 205//DEFAULT PWM TO CALCULATE MOTORS' SPEED

#define MIDDLE_SENSOR 4, 5
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define DEBUG 1 // set to 1 if serial debug output needed

QTRSensorsRC qtrrc((unsigned char[]) {54, 55, 56, 57, 58, 59, 60, 61} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// connect motor controller pins to Arduino digital pins
int enA = 3; int in1 = 4; int in2 = 5;   // left motor
int in3 = 6; int in4 = 7; int enB = 8;   // right motor

/*Parameters for the PID Controller*/
int lastError = 0;
int  last_proportional = 0;
int integral = 0;
int position, error;
int motorSpeed, rightMotorSpeed, leftMotorSpeed;

void setup()
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  manual_calibration();
}



digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);

digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);


void loop()
{

  unsigned int sensors[6];
  position = qtrrc.readLine(sensors);
  error = position - 3500;
  motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  rightMotorSpeed = DEFAULT_SPEED + motorSpeed;
  leftMotorSpeed = DEFAULT_SPEED - motorSpeed;


  if (rightMotorSpeed > MAX_SPEED ) rightMotorSpeed = MAX_SPEED; // limit to top speed
  if (leftMotorSpeed > MAX_SPEED ) leftMotorSpeed = MAX_SPEED;

  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  

  
  analogWrite(enA, rightMotorSpeed);
  
  analogWrite(enB, leftMotorSpeed);

}


void manual_calibration() {
  int i;
  for (i = 0; i < 250; i++)  // the calibration will take a few seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }
  if (DEBUG) { // if true, generate sensor dats via serial output
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    for (int i = 0; i < NUM_SENSORS; i++)
{
  Serial.print(qtrrc.calibratedMaximumOn[i]);
  Serial.print(' ');
}
Serial.println();
Serial.println();
  }
}