/********************************************************************
 *                                                                  *
 *          Upright Robot with Arduino                              *
 *                                                                  *
 *  MPU6050 accelerometer/gyroscope combo  *           *
 *  Uses Kalman filter for sensor fusion                           *
 *                                                                  *
 * ******************************************************************/
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu; // Create the MPU6050 instance

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

// Timers
uint32_t timer;

// Motor Pins --------------------------------------------------------------------------

// Motor A connections
int enA = 9;
int IN1 = 8;
int IN2 = 7;
// Motor B connections
int enB = 3;
int IN3 = 5;
int IN4 = 4;

// ------------------------------------------------------------------------------------
// Sensor Values ----------------------------------------------------------------------

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

// ------------------------------------------------------------------------------------
// Control Variables ------------------------------------------------------------------

double desiredAngle = 0.0; // Desired angle for the robot to maintain
double prevAngle = 0.0; // Previous angle for computing error
double prevError = 0.0; // Previous error for computing derivative

// ------------------------------------------------------------------------------------

/****************************************************************************************
                                       CONTROLLER
****************************************************************************************/

// Control Parameters
double kp = 1.0;
double kd = 1.0;

/****************************************************************************************
                                       SETUP LOOP
****************************************************************************************/
void setup(void) {
  Serial.begin(115200); // SEE NOTE ABOVE: this is set to 115200!

  while (!Serial)
    delay(10); // will pause everything until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // ----------------------------------------------------------------------------------
  // Trying to initialize the MPU6050 -------------------------------------------------
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!"); // This message indicated everything is connected correctly.

  // Setting up the MPU ranges:
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // options: 2G, 4G, 8G, 16G
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // options (+/-): 250, 500, 1000, 2000
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // options (Hz): 260, 184, 94, 44, 21, 10, 5

  // ----------------------------------------------------------------------------------
  // Sensor Setup ---------------------------------------------------------------------

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  // ----------------------------------------------------------------------------------
  // Motor Setup ----------------------------------------------------------------------

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // ----------------------------------------------------------------------------------

  timer = micros();
}

/****************************************************************************************
                                       MAIN LOOP

Steps:
    * read MPU6050 sensor, run Kalman filter
    * compute corrective action
    * scale & clamp corrective action, apply to motors
    * Delay a defined amount to pad the loop execution time to 5ms (bigT)
****************************************************************************************/
void loop() {
  float angle, velocity, turn_diff;
  float correction, prev_corr;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  
  // Compute the error (difference between desired angle and actual angle)
  double error = gyroY - prevAngle;
  
  // Compute derivative term
  double derivative = error - prevError;
  
  // Compute corrective action 
  correction = kp * error + kd * derivative;
  
  // Update previous error & angle for next iteration
  prevError = error;
  prevAngle = gyroY;

  if (correction>0){
    directionControl(1, correction);} // forward
  else {
    directionControl(0, correction);} // backward


  /* PRINT STATEMENTS 
  //Serial.print("Acc_X: ");
  Serial.print(a.acceleration.x);
  Serial.print("\t");
  
  //Serial.print("Acc_Y: ");
  Serial.print(a.acceleration.y);
  Serial.print("\t");
  
  //Serial.print("Acc_Z: ");
  Serial.print(a.acceleration.z);
  Serial.print("\t");
  
  //Serial.print("Gyro_X: ");
  Serial.print(g.gyro.x);
  Serial.print("\t");
  
  //Serial.print("Gyro_Y: ");
  Serial.print(g.gyro.y);
  Serial.print("\t");
  
  //Serial.print("Gyro_Z: ");
  Serial.println(g.gyro.z);
  /**/
  
  delay(2); // Add delay as needed
}

// ----------------------------------------------------------------------------------

/* MOTOR CONTROL FUNCTIONS *********************************************************
https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/ was a great resource
when figuring out how to build my motor driver circuit/get it working with the controller.

     IN1 |  IN2 |  Spinning Direction
    -----------------------------------
      0  |   0  |        OFF
      1  |   0  |      Forward
      0  |   1  |      Backward
      1  |   1  |        OFF

************************************************************************************/

// ----------------------------------------------------------------------------------
// Control of motor spinning direction ----------------------------------------------

void directionControl(int dir, double correction) {

  // Scale the correction to fit the motor control range
  int motorSpeed = int(correction * 255); // Scale to PWM range (0 - 255)

  // Clamp motor speed to avoid going beyond the valid range
  motorSpeed = constrain(motorSpeed, 0, 255);
  motorSpeed = motorSpeed*10; // scaling

	// Set motors to speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, motorSpeed);
	analogWrite(enB, motorSpeed);

  Serial.println(motorSpeed);

  if (dir == 1){ // FORWARD direction
	  // (1,0) logic to both A & B
	  digitalWrite(IN1, HIGH);
	  digitalWrite(IN2, LOW);
	  digitalWrite(IN3, HIGH);
	  digitalWrite(IN4, LOW);
	  delay(2); }
  else if (dir == 0){ // BACKWARD direction
    // (0,1) logic to both A & B
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2);}
}

// ----------------------------------------------------------------------------------
// Turns motors off when called -----------------------------------------------------

void motorsOff(){
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}

// ----------------------------------------------------------------------------------
