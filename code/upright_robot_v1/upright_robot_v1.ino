
/********************************************************************
 * 																	*
 * 			Upright Robot with Arduino								*
 * 																	*
 * 	Simple model with LSM303/L3G4200 accelerometer/gyroscope combo	*
 * 	sensor (breakout board); now also with MPU6050					*
 * 	Uses Kalman filter for sensor fusion							*
 * 																	*
 * 	Some stability problems persist, they are likely caused by		*
 * 	mechanical imperfection (gearbox slack), and this nonlinearity	*
 * 	requires nonlinear approaches. Some ideas to follow up with:	*
 *		Anti-chatter												*
 * 		sliding mode control										*
 * 		nonlinear describing function								*
 * 																	*
 * ******************************************************************/


//--------------------------------------------------------------------------------
// EDITOR: Set to hard tabs, size 4 to view the code with correct indentation
//--------------------------------------------------------------------------------

/*
/	NB. Library manager is a piece of (...)
/ 	better install manually under /usr/share/arduino/libraries/....
/	The "regular" way is the main/default sketchbook. Check Preferences for
/	non-standard location (e.g., server). There, for example,
/	    /server/home/mhaidekk/projects/arduino/libraries/
/	the IDE looks for .h files.
/
/	Also note: Do not include the accelerometer .h file here.
/	This is done in the project file accel_gyro.c, which should reside
/	in the same directory as this main file. In this fashion,
/	we gain flexibility to include multiple sensors in accel_gyro.c
/
/	Moreover: Some sign-flipping took place with the new MPU6050.
/	These are not reflected in the L3G/LSM303 part. The present
/	coordinate system has the IMU z-axis pointing "forward".
/	The y-axis points down, and the x-axis points right.
/	Consequently, tilting forward makes the z-acceleration positive;
/	the derivative is the negative gyro x-axis.
/
*/


#include <Wire.h>		// I2C communication with IMU
#include <Kalman.h>		// theta, theta-dot Kalman filter
#include <math.h>


//-------------------------------------------------------------------------

/* Arduino Pin equates for various control and I/O pins */

#define AN_ZTRIM	0           // AN0: Zero trim potentiometer
#define AN_KPPOT	1           // AN1: kp
#define AN_KDPOT	2           // AN2: kD

#define BIN2		2			// Motor B direction control
#define BIN1		3			// Motor A direction control
#define AIN2		4			// Motor A direction control
#define AIN1		5			// Motor A direction control
#define PWMA		6			// Motor A PWM
#define CFG_SW2		7           // A DIP switch. The numbers refer to the labeling on the DIP sw unit
#define PWMB		9			// Motor B PWM
#define HBT_OUT		10			// Another LED... no LED installed, though.
#define CFG_SW1     11          // Another DIP switch (the lower one)
#define LEDOUT		13          // LED on the Arduino board


/* Some coefficients needed by the PD-controller. Make sure that
	KPSCALE, KDSCALE and TDSCALE are floating-point values. */

#define KPKD_FORMULA	1				/* Set to 0 for a k-tau_d formulation, 1 for a kp - kD formulation */
#define KPSCALE			60.0			/* Maximum value for kp (when ADC output is normalized to 0..1) */
#define KDSCALE			0.5				/* Maximum value for kD (when ADC output is normalized to 0..1) */
#define TDSCALE			0.01			/* tauD, alternative formulation. Roughly, TDSCALE*KPSCALE = KDSCALE */
#define DIFFGAIN		0.08			/* Wheel differential gain (keep straight path)	*/
#define KICKPWM  		0				/* Slack compensation when a motor sign change occurs */


/* Other constants used throughout */

#define	RAD_2_DEG		57.2957795131	/* Convert radians to degrees (180 / pi) */
#define ADC_MAX			1024.0			/* Highest ADC value (10-bit ADC) */


/* Weirdly enough, these don't seem to be defined by default. Arduino? What?? */

#ifndef TRUE
#define	TRUE		1
#endif
#ifndef FALSE
#define	FALSE		0
#endif


//-------------------------------------------------------------------------

// Flow control, debugging

unsigned char ena_serial;			// Set to 1 to dump messages to the serial I/F, 2 to use the serial plotter
unsigned long itercnt;				// Counter for the loop iterations
unsigned char enable;				// General enable flag for the motors


// Sensors and Measurement

int gyro_y;						// Intermediate raw sensor values
int accel_x;
int accel_z;

float angleCalc=0;				// Angle obtained from accelerometer
float kalAngle=0;				// Angle from Kalman filter
float gyroY;					// Angle rate (degrees per second), from gyro
float gyroW;					// Turn angle rate (right turn > 0)
float kalGyroY;					// Angle rate (degrees per second), from Kalman filter
float angleoffset;				// Zero-trim coefficient, set by potentiometer at AN0
float thetaz;					// Immediate past value for discrete D-component (not used)


// Controller and timing

unsigned long run_st, run_en;		// Runtime for one control action
unsigned long bigT;					// Controller sample rate T, compute time plus padding
unsigned char long_t_cycle;			// Flag decides whether to update the motor more or less frequently

float KD;							// D-gain, potentiometer-adjusted
float KP;							// P-gain, potentiometer-adjusted
float KI = 5e-7;					// Note: Must convert bigT to usec
float K = 20.0;						// controller gain in gain-zero formulation
float tau_d = 0.05;					// Those initial values get overwritten in the first loop run
int p_raw, d_raw;					// Raw ADC values from P and D potentiometers
float pwm_ramp;						// Allows slow ramping up of motor power at power-up or tripping
unsigned char prevsign = 0;			// to detect a motor direction change
unsigned char kick_pwm = 0;			// To sustain motor kick speed (to overcome slack) for more than T


// Smoothing filter cyclical value storage

float sensAng[4];					// Angle from accelerometer, unfiltered
float kalAng[4];					// Angle, Kalman-filtered
float sensVel[4];					// Angular vecloity from gyro, unfiltered
float kalVel[4];					// Angular vecloity, Kalman-filtered
float weight[4] = {0.50, 0.35, 0.10, 0.05};		// Ensure that the sum of the elements is 1.0!
float llc_gain;						// Gain coefficient to get unity DC gain
float intacc;						// Int component, anybody?

// Create a Kalman filter object

Kalman kalmanFilter;



/***********************************************************************

	The Arduino has two default entry points:
		setup()			called once on initialization
		loop()			the main loop, iterated forever

*/


void setup()
{
int i;

	Serial.begin(115200);			// Match this in IDE. 9600 Baud is not appropriate for 21th century.
	ena_serial = 0;					// Set to 1 to get debug messages to the console (text)
									// or set to 2 to use the serial plotter for angle, theta-dot, and corraction

	// Configure I/O pins

	pinMode (PWMB, OUTPUT);
	pinMode (PWMA, OUTPUT);
	pinMode (AIN1, OUTPUT);
	pinMode (AIN2, OUTPUT);
	pinMode (BIN1, OUTPUT);
	pinMode (BIN2, OUTPUT);

	pinMode (LEDOUT, OUTPUT);		// There is a LED attached to this. Use for monitoring
	pinMode (HBT_OUT, OUTPUT);		// Heartbeat output. Another LED & testpoint for oscilloscope
	pinMode (CFG_SW1, INPUT);		// A switch to change some mode in real-time
	pinMode (CFG_SW2, INPUT);		// Another switch

	digitalWrite (AIN1, LOW);
	digitalWrite (AIN2, LOW);
	digitalWrite (BIN1, LOW);
	digitalWrite (BIN2, LOW);
	digitalWrite (PWMA, LOW);
	digitalWrite (PWMB, LOW);

	// Initialize the I2C peripherals (= sensor IMU)

	I2C_Init();
	Accel_Init();
	Compass_Init();
	if (digitalRead(CFG_SW2))		// On = High = LPF on = low bandwidth
		Gyro_Init (0x40);
	else							// off = low = lpf off = high bandwidth
		Gyro_Init (0x50);			// LPF exists for both L3G gyro and MPU6050


	// Initialize or clear some variables

	run_st = micros();			// Number of usec since start of the program
	run_en = run_st;
	itercnt = 0;
	enable = FALSE;				// Keep the motors disabled at the start
	pwm_ramp = 0.0;
	intacc = 0.0;
	bigT = 5000;				// Sampling time in usec

	for (i=0; i<3; i++)
	{
		sensAng[i] = 0.0;
		kalAng[i] = 0.0;
		sensVel[i] = 0.0;
		kalVel[i] = 0.0;
	}

	init_potentiometers ();				// Populate coefficient variables at the start

	kalmanFilter.setAngle (0.0);		// The initial angle should be set
//	kalmanFilter.setQangle (0.001);		// Accelerometer noise variance (default is 0.001)
	kalmanFilter.setQbias (0.001);		// Gyro *bias* noise variance. Default is 0.003,
										// smaller values lead to more smoothing

	long_t_cycle = FALSE;				// If TRUE, update motor only every 4 cycles

}


// Main loop:
//
// Here, the control computation is performed in regular (!) intervals.
// The steps are:
//	* Read coefficient potentiometers (uncritical)
//	* Read sensors (accelerometer, gyroscope)
//	* Run Kalman filter to get best estimate of theta, theta-dot
//	* Compute corrective action
//	* Scale & clamp corrective action, apply to motors
//	* Delay a defined amount to pad the loop execution time to 5ms (bigT)
//
// L3G - LSM303:
// Computations take approximately 3 ms. Out of this, coefficient readout
// is approximately 0.5ms (quite variable), the Kalman filter is
// another 0.5ms (and fairly constant, well written!). The rest goes to
// several float operations, such as arc tan and corrective action.
//
// MPU6050:
// Computations take approximately 4 ms, longer if the lead-lag component
// is enabled. The longer time is due to the motor control, which now
// computes PWM separately for each motor and allows for turn angle correction.



void loop()
{
float angle, velocity, turn_diff;
float corraction;					// Corrective action from controller
float prev_corr;					// corrective action, previous iteration
unsigned char i, cycle;
unsigned long pad_dly;
unsigned long idle_dly;				// Idle delay (usec) to pad bigT


	run_st = micros();						// We need to know how long this takes
	digitalWrite (HBT_OUT, HIGH);			// LED/monitor bit -> high

	// Reads bottom potentiometer to calculate angle offset
	// Potentiometer can alter the offset from -3.5 to +1.5 degrees
	// (the actual zero angle is -1.25 degrees)
	//
	// The slow items (coefficient potentiometers) do not need to be read out
	// at every iteration, so we schedule them sequentially with
	// itercnt & 0x03, which cycles 0...1...2...3...0...1...

	cycle = itercnt & 0x03;

	if (cycle==1)				// Zero trim potentiometer (exact vertical; angle setpoint)
		angleoffset = (float( analogRead(AN_ZTRIM)*5) / ADC_MAX) - 3.5;
	else if (cycle==2)			// Potentiometer for kp coefficient
		p_raw = analogRead (AN_KPPOT);
	else if (cycle==3)			// potentiometer for kD coefficient
		d_raw = analogRead (AN_KDPOT);

	debug_dump_status (itercnt);

	// OTOH, sensors must be read out at the maximum sampling rate.
	// Use this place to store the sensor output in the past-data arrays.
	// Note: Code to perform smoothing via weighted average of past values
	// was tried and found inferior to Kalman filtering. Code removed.

	GetSensors();							// Read sensors, run Kalman filter
	sensAng[cycle] = angleCalc;
	angle = kalAng[cycle]  = kalAngle;		// Theta (tilt angle) from Kalman filter
	sensVel[cycle] = gyroY;
	velocity = kalVel[cycle]  = kalGyroY;	// Theta-dot from Kalman filter
	turn_diff = -(DIFFGAIN*gyroW);			// Calculate the wheel differential based on gyroW

	debug_dump_sensors();

	// We have two controller options:
	//
	// Normal:
	// Run the control action once per cycle. Sensor T and corrective action T
	// are the same. We need to ensure that T is not too short.
	//
	// Alternative:
	// Run the controller (i.e., compute corrective action and apply to motors)
	// at a lower frequency than reading the sensor.
	// The controller-T can be significantly slower than the sensor-T, and we cycle
	// the corrective action with itercnt. If it reaches 3, compute & apply.
	// The result is that we get 4 sensor readings (5ms apart) for one corrective action
	// (20ms apart). This allows for some smoothing.

	if ((long_t_cycle == FALSE)			// ==0 selects one update per cycle; ==1 sets one update / 4 cycles
		|| (cycle==0))					// and this part takes care of the update every 4th cycle
	{

		if (abs(angle)>45) 				// Cut off motors past 45 degrees...
		{
			corraction = 0.0;
			intacc = 0.0;
			itercnt = 0;				// Ensure some delay before the motors are back up
			pwm_ramp = 0.0;
			turn_diff = 0.0;
		}
		else
			corraction = controller(angle, velocity);	// ...or run the PD controller


		/* Output debugging information if enabled, then apply corrective action to motors. */

		debug_dump_corraction (angle, velocity, corraction);
		debug_plot_corraction (angle, velocity, gyroY);

		if ( (corraction*prev_corr) < 0.0)
			setMotors (0, turn_diff);	// Sign change: Skip one round; drop motor torque to zero
		else
			setMotors (corraction, turn_diff);	// Apply to motor PWM. setMotors() takes care of clamping

		prev_corr = corraction;
	}

	itercnt++;
	if (itercnt==20) enable=TRUE;		// Wait for 20 cycles before starting the motors
	if (pwm_ramp < 1.0)					// and even so, ramp up max power gradually
		pwm_ramp += 0.005;


	digitalWrite (HBT_OUT, LOW);		// LED/monitor bit -> low during idle delay
	run_en = micros ();					// Unless there is an overflow, we have the elapsed usecs
	if (run_en > run_st)
		idle_dly = run_en - run_st;		// Microseconds left to complete bigT
	else
		idle_dly = 1000;				// Rollover (very unlikely), assume some arbitrary runtime

	if (idle_dly < bigT)				// Let's hope this always takes less than bigT
	{									// Because if not, we skip time padding, and T becoomes irregular
		pad_dly = bigT - idle_dly;		// Need to explicitly assign to a long int to avoid rollover
		delayMicroseconds (pad_dly);	// Pad the run time to 5ms
	}


}



/************************************************************************************/

// Function to read the potentiometers for the first time.
// Should be called during initialization so that initial values
// for the controller are set.

void init_potentiometers ()
{
	angleoffset = (float( analogRead(AN_ZTRIM)*14) / ADC_MAX) - 4.0;
	p_raw = analogRead (AN_KPPOT);
	d_raw = analogRead (AN_KDPOT);
}



// Reads the sensors, calculates angle using accelerometer, feeds angle and gyroscope
// reading to Kalman filter object (angle in degrees, rate in degrees/second)
//
// L3G - LSM303 only:
// The accelerometer is set to 2g. It outputs 12 bits + sign
// and we have 1 mg / LSB. The z axis points forward, and the x axis down.
// Hence, the "active" axis is z, and it would be sufficient to scale accel_z
// with 1/1000 (because: 1 milli-gee per LSB). So a digital value Z of 1000 (or a scaled
// value of 1.000) is 1 gee. Taking the digital value Z/1000 is sin(theta) and therefore
// approximately theta in radians. The digital value of Z multiplied with 0.0573 yields
// theta in degrees for small angles.
//
// The students did it somewhat more elegantly by using the tangent, therefore
// obtaining the angle in radians irrespective of scaling. All that's left
// is to convert to degrees as required by the Kalman filter.
// Question: Does the division in atan2 increase sensor noise since it involves both axes?
// Answer: Hell yeah!
//
// Kalman filter: The getAngle method does the "heavy lifting". It requires the
// original sensor data (theta and theta-dot), as well as the time step bigT in seconds.
// Immediately returns the filtered angle; the rate theta-dot needs to be retrieved
// separately with getRate()
//
// The output of this function is either (gyroY, angleCalc) as calibrated, but
// unfiltered sensor output, or (kalGyroY, kalAngle) as Kalman-filtered output.
//
// In the original code, gyroY was used directly (without Kalman filtering) -- why?


void GetSensors()
{

	// Read the sensors. This is done in accel_gyro.c
	// The result is returned in gyroY and angleCalc

	Read_Accel();			// Accelerometer read into AN[3] (x-axis) and AN[5] (z-axis)
	Read_Gyro();			// Gyroscope y axis goes into AN[1] (15 bits + sign)
	Process_Accel_Gyro ();	// Prepare gyroY and angleCalc

	// Kalman filter. Requires Theta (degrees), theta-dot (degrees/s), and T in seconds
	// It returns the filtered angle theta (kalAngle), and the call to getAngle()
	// actually iterates the Kalman filter. The second result of the iteration,
	// the filtered rate theta-dot can be retrieved with getRate()

	kalAngle = kalmanFilter.getAngle(angleCalc, gyroY, (float)(bigT)*1.0e-6);
	kalGyroY = kalmanFilter.getRate();			// overwrite gyroY with the Kalman-filtered value

}


// Compute corrective action (scaled, signed, but not clamped)
// The corrective action is expressed as motor current in % of maximum current,
// and it will be expanded to 8 bits (0...255) in setMotors().
// If the robot is tilting forward, the corrective action also must point forward.
//
// NOTE: The IMU conveniently provides theta and its derivative. Hence, if the corrective action
// is produced by a PD controller,
//			A(s) = (kp + kD*s) epsilon(s)
// and we already have the derivative, then we can formulate
//			A(t) = kp * theta(t) + kD*omega(t)
// where theta is the angle deviation from vertical and omega is the angular (tilt) momentum.
// In cases where omega is not directly know, the derivative can be approximated by a finite
// difference under the knowledge that two successive calls of controller() are bigT apart.
// For this purpose, we need to store a past value, let's call it thetaz. Then we can use
//			A(t) = kp*theta(t) + kD*(theta(t) - thetaz(t))
//			thetaz(t) = theta(t)
// where the second line updates the immediate past value for the next iteration.
// Moreover, an integral component can be approximated in the same fashion, whereby we add up
// theta (or epsilon) multiplied by bigT, i.e., area under the measurement point.



float controller (float phi, float phidot)
{
float corraction, llc_out, epsilon;

	epsilon = phi-angleoffset;
	intacc += epsilon*bigT;

	KP = (float)(p_raw)*KPSCALE/ADC_MAX;
	KD = (float)(d_raw)* KDSCALE/ADC_MAX;
	corraction = KP*epsilon + KD*phidot;

	if (FALSE)							// The following code for illustration purposes only
	{
		corraction = 								// PD corrective action is the sum...
				KP*epsilon 							// of the P-component
				+ KD*(epsilon - thetaz)/bigT;		// and the finite-difference
													// approximation of the derivative
		thetaz = epsilon;							// Update past value for next iteration
	}


	if (digitalRead (CFG_SW1))	// Upper switch on (high), enables I-component
	{
		corraction += KI*intacc;
	}

	return corraction;
}




// Set motor PWM and direction given a corrective action value corr.
// corr reflects the motor power in percent (range w/o saturation: -100 to 100).
// corr can be signed. The PWM value is limited to 8 bits, and the sign
// of the corrective action translates into the driver's direction bits.
// DIRECTION SIGN: As defined here, corrective action with a positive sign
// moves in the positive z-direction of the MPU6050 sensor. This sign must
// match with the turn direction.
//
// Since the gearbox has some slack, we add some PWM-kicking whenever
// a direction change occurs. This is experimental, and the related lines
// are marked [!!] (in case they need to be removed)
//
// PWM values are split into left and right motor values early so that we can
// apply the turn ratio and retain the ability to clamp the values
// for each motor individually. This allows, in the extreme case, motors to have
// different directions.

void setMotors (float corr, float differential)
{
int pwm_right, pwm_left;
float corr_raw;
unsigned char sgn_l, sgn_r;		// FALSE (0 or positive); TRUE (negative)
unsigned char saturated, cfg;


	saturated = 0;

	if (!enable)				// Stop motors if they are disabled
	{
		digitalWrite (AIN1, LOW);
		digitalWrite (AIN2, LOW);
		digitalWrite (BIN1, LOW);
		digitalWrite (BIN2, LOW);
		analogWrite (PWMA, 0);
		analogWrite (PWMB, 0);
		digitalWrite (LEDOUT, LOW);
		return;
	}

	/* Step 1: Add the turn differential, and in the process
		separate out the sign (each motor independently), then
		pre-clamp to ensure we can fit the intermediate value in a
		16-bit integer
		NB: PWM A is left, PWM B is right.
	*/

	corr_raw = (corr - differential)*pwm_ramp;	// Right motor
	sgn_r = 0; if (corr_raw < 0) sgn_r = 1;
	corr_raw = fabs (2.56*corr_raw);			// Expand to byte; dispense with the sign, we extracted it
	if (corr_raw > 2000.0)
	{
		corr_raw = 2000.0;						// Pre-clamp (coarse clamp); ensure it fits an int
		saturated = 1;							// *any* clamping implies saturation
	}
	pwm_right = (int)( floor(corr_raw)+0.5);		// Round to integer
	if (pwm_right > 255)						// Final clamp
	{
		pwm_right =255;
		saturated = 1;
	}

	corr_raw = (corr + differential)*pwm_ramp;	// Left motor
	sgn_l = 0; if (corr_raw < 0) sgn_l = 1;
	corr_raw = fabs (2.56*corr_raw);			// Range & remove sign
	if (corr_raw > 2000.0)
	{
		corr_raw = 2000.0;						// Pre-clamp
		saturated = 1;
	}
	pwm_left = (int)( floor(corr_raw)+0.5);		// Round to integer
	if (pwm_left > 255)							// Final clamp
	{
		pwm_left =255;
		saturated = 1;
	}


	/* Step 2: Set direction bits and set PWM magnitude
		for each motor individually */

	if (sgn_r)								// negative direction, right
	{
		digitalWrite (BIN1, HIGH);
		digitalWrite (BIN2, LOW);
	}
	else
	{
		digitalWrite (BIN1, LOW);
		digitalWrite (BIN2, HIGH);
	}
	analogWrite (PWMB, pwm_right);			// Magnitude

	if (sgn_l)								// negative direction, right
	{
		digitalWrite (AIN1, HIGH);
		digitalWrite (AIN2, LOW);
	}
	else
	{
		digitalWrite (AIN1, LOW);
		digitalWrite (AIN2, HIGH);
	}
	analogWrite (PWMA, pwm_left);			// Magnitude

	if (saturated)							// Indicate clamping with LED
		digitalWrite (LEDOUT, HIGH);
	else
		digitalWrite (LEDOUT, LOW);

}


/************************************************************************************/

// Fault loop.
// If a fault is detected, we call this function.
// It disables the motors, then blinks the LED
// in a long - short - short ... pattern
// where the short blinks indicate the error number

void fault_stop (int errcode)
{
int i;

	digitalWrite (AIN1, LOW);
	digitalWrite (AIN2, LOW);
	digitalWrite (BIN1, LOW);
	digitalWrite (BIN2, LOW);
	analogWrite (PWMA, 0);
	analogWrite (PWMB, 0);

	while (1)
	{
		digitalWrite (LEDOUT, LOW);			// LED off for 1000ms
		delay (1000);
		digitalWrite (LEDOUT, HIGH);		// LED on for 500ms (long ---)
		delay (500);
		digitalWrite (LEDOUT, LOW);
        delay (500);

		for (i=0; i<errcode; i++)
		{
			delay (250);
			digitalWrite (LEDOUT, HIGH);		// LED on for 250ms
			delay (250);
			digitalWrite (LEDOUT, LOW);
		}
	}
}



/************************************************************************************/
//
// Device Testing & Software Debugging


// Debugging functions. These are executed only if ena_serial is TRUE.
// Otherwise we exit immediately.

void debug_dump_sensors ()
{

	if (ena_serial != 1)
		return;

	Serial.print("Sensors: unfiltered ang=");
	Serial.print (angleCalc);
	Serial.print("  vel=");
	Serial.print (gyroY);
	Serial.print ("      Kalman: ang=");
	Serial.print (kalAngle);
	Serial.print("  vel=");
	Serial.println (kalGyroY);

}

void debug_dump_status (long iter)
{

	if (ena_serial != 1)
		return;

	Serial.print("Iteration ");
	Serial.print (iter);
	if (enable)
		Serial.print("   Enabled ");
	else
		Serial.print("   Disabled ");

	Serial.print(" kp (raw) ");
	Serial.print (p_raw);
	Serial.print(" kd (raw) ");
	Serial.println (d_raw);
}


void debug_dump_corraction (float angle, float velocity, float corraction)
{

	if (ena_serial != 1)
		return;

	Serial.print("Controller angle ");
	Serial.print (angle);
	Serial.print("  vel ");
	Serial.print (velocity);
	Serial.print("  -> Corr.action ");
	Serial.println (corraction);
}



// Same, but formatted for the serial plotter

void debug_plot_corraction (float angle, float velocity, float corraction)
{

	if (ena_serial != 2)
		return;

	Serial.print (angle);
	Serial.print(" ");			// tab or blank space delimits
	Serial.print (velocity);
	Serial.print(" ");			// tab or blank space delimits
	Serial.println (corraction);	// End of line ends data set

}


// A small function to test the level of kicking PWM.
// This is an endless loop in which the PWMs are turned on
// for one cycle (5ms), then kept off for 100ms, repeated with
// alternating directions.

void test_slack_comp ()
{
int i;


	while (1)
	{
		if (i & 0x01)				// Negative direction
		{
			digitalWrite (AIN1, LOW);
			digitalWrite (AIN2, HIGH);
			digitalWrite (BIN1, LOW);
			digitalWrite (BIN2, HIGH);
		}
		else           			// positive direction
		{
			digitalWrite (AIN1, HIGH);
			digitalWrite (AIN2, LOW);
			digitalWrite (BIN1, HIGH);
			digitalWrite (BIN2, LOW);
		}

		analogWrite (PWMA, 64);
		analogWrite (PWMB, 64);

		delayMicroseconds (bigT);
		i++;

		analogWrite (PWMA, 0);
		analogWrite (PWMB, 0);

		delay (250);

	}

}





/**********************************************************************************/
