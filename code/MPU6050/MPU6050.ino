/********************************************************************
 * 																	*
 * 			Upright Robot with Arduino								*
 * 																	*
 * 	MPU6050 accelerometer/gyroscope combo	*			*
 * 	Uses Kalman filter for sensor fusion							*
 * 																	*
 * ******************************************************************/
// NOTES:

// BEFORE RUNNING: Make sure you chance 0x68 to 0x72 in the Adafruit_MPU6050.h file 
                // #define MPU6050_DEVICE_ID 0x72 -----> change it from 0x68 to 0x72

// VIEWING OUTPUT: Notice that the serial is set to 115200 baud. 
                // Make sure you set the monitor & plotter to this if you want to view!

// ------------------------------------------------------------------------------------
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu; // Create the MPU6050 instance

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

// Motor Pins --------------------------------------------------------------------------

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// ------------------------------------------------------------------------------------
// Sensor Values ----------------------------------------------------------------------

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

// ------------------------------------------------------------------------------------

uint32_t timer;

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

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  // ----------------------------------------------------------------------------------
  // Motor Setup ----------------------------------------------------------------------

  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);

  // ----------------------------------------------------------------------------------
  
  timer = micros();
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  while(true){
    directionControl(1); // forward
  }


   /* Print Data */
  #if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");

    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");

    Serial.print("\t");
  #endif

    Serial.print(roll); Serial.print("\t");
    Serial.print(gyroXangle); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");

    Serial.print("\t");

    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");

  /* PRINTING SENSOR VALUES TO SERIAL MONITOR */
  /* Uncomment for testing */
  /*
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
  */


  Serial.print("\r\n");
  delay(2);
}

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
void directionControl(int dir) {
	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, 255);
	analogWrite(enB, 255);

  if (dir == 1){ // FORWARD direction
	  // (1,0) logic to both A & B
	  digitalWrite(in1, HIGH);
	  digitalWrite(in2, LOW);
	  digitalWrite(in3, HIGH);
	  digitalWrite(in4, LOW);
	  delay(2000); }
  else if (dir == 0){ // BACKWARD direction
    // (0,1) logic to both A & B
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(2000);}
}
// ----------------------------------------------------------------------------------
// Control of motor speed 
void speedControl() {
	// Turn on motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
	
	// Accelerate from zero to maximum speed
	for (int i = 0; i < 256; i++) {
		analogWrite(enA, i);
		analogWrite(enB, i);
		delay(20);
	}
	
	// Decelerate from maximum speed to zero
	for (int i = 255; i >= 0; --i) {
		analogWrite(enA, i);
		analogWrite(enB, i);
		delay(20);
	}
	
	// Now turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

// ----------------------------------------------------------------------------------

// Turns motors off when called
void motorsOff(){
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

// ----------------------------------------------------------------------------------