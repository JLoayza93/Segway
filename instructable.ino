// This program is an inverted pendulum on a two-wheel cart
// Uses an arduino Uno or bigger one if nescessary, MPU650 Gyro and L298 2A Controller for the motors.
//T.J.Moir
//Code is provided AS-IS - no responsibility is accepted for it. Use at your own risk.
// Code modified from various instructable projects. eg http://www.instructables.com/id/Balancing-Instructable-Robot/  or  http://www.instructables.com/id/Self-balancing-skateboardsegwy-project-Arduino-S/ 
// We use state-feedback  - pole-placement here.The four states are angular position, angular velocity,angular acceleration and a 4th state for an integrator.
// you will need the I2Cdev.h library
// you will need the MPU6050.h library
// other libraries you should have from basic arduino that comes with the device software
// No steering at present
// No encoder feedback from wheels
// The control system is state-feedback - not PID 9though thre are similarities if you do the maths). I can measure position and velocity from the sensor but have to create acceleration by diffeentiating velocity.
// should really be able to get velocity directly but at pesent I synthesise it from velocity by band-limited differentiation.

///////////////////////////////////////////////////////////////////////////////////
    // Wiring
       
    ///////////////////////////////////////////////////////////////////////////////
    // Wire Interupt on MPU6050 to pin 2 on Arduino
    // Connect SDL SDA pins on Arduino to MPU6050 - same lettering
    // Various power supplies - connect the 3.3V on MPU6050 Gyro toArduino 3.3V output.
    // Connect earths together and then to battery negative - important. You need three - one from each device.
    // MPU6050 - lie it flat near the centre of gravity of the robot ie near middle of the axle.
    //////////////////////////////////////////////////////////////////////////////
    
    //Arduino        L298 H Bridge
   // D10            7
   // D5             12
   // D9             IN1
    //D8             IN2
    //D7             IN3
   // D6             IN4
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
    // I also use two pots - you can use as many as you like but the two I use are for overall gain K and offset. The offset is for the gyro. Now and again I tim it so that the robot is vertical. Othrwise it leans over and uns away.
    // some others put pots for PID (which this project is not) - which is convenient I suppose. I did it in software by tial and error which takes longer.
    //So I use A0 and A1 on the Arduino which are A/D inputs. the wiper pat of the pots get wired to these pins A0 for gain and A1 for offset adjust. The other two leads on the pot are connected to regulated 5V from the L298 and 0 volts.
    // Do not connect the pots to the main battery power supply since it is unregulated and will drift.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container

VectorFloat gravity; // [x, y, z] gravity vector

////////////////////////////////////////////////////////////////////////////
// Overall Gain is set from Pot 1 on A0 A/D convertor
static int pot1Pin = A0;

//Offset Pot on pin A1 so that it doesn't drift in one direction and stays steady.
static int pot2Pin=A1;
/////////////////////////////////////////////////////////////////////////////

int STD_LOOP_TIME = 1; //10mS loop time (100Hz) // code that keeps loop time at 10ms per cycle of main program loop Won't run any faster! 
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
//float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector

/////////////////////////////////////////////////////////////////////////////////////////
// These gain values can be calculated rom the dynamic model of the system. If you don't have one then you need to guess! Educated guess that is. Start with Kp only and thn add Kd and Ka. Doesn't take much integral action.
// that's why Ki is small since we need good damping. Not good without Ki since it won't stay straight up and tends to flop back and forward due to stedy-state error. All control systems need integrators with very few exceptions.
// State 1 gain - popotional
float Kp = 38;  //15 normalmente   74N,50Eigentlich gut, 40 besser, 38 fast perfekt
//Integral gain for integral state 4
float Ki = 0; //0.3 normalmente
//Derivative gain for state 2
float Kd = 0; //12 normalmente   
// state gain for acceleration state 3
float Ka=10;  //10 normalmente 
//74,0.3,3,3
/////////////////////////////////////////////////////////////////////////////////////////
float currAnglep=0;
float currAngle = 0;
float angular_rate=0;
float angular_ratep=0;
float angular_acceln=0;
float angular_accelnp=0;
float setpoint = 0; // zero radians setpoint since it is vertical.
float control_out=0;
float error=0;
float last_error=0;
float integrated_error=0;
float integrated_errorp=0;
float est_angle;
float est_anglep;
float y_angle_comb;
float aa_const = 0.998; //this means 0.2% (1-0.998)*100 of the accelerometer reading is fed into angle of tilt calculation with every loop of program (to correct the gyro).
//accel is sensitive to vibration which is why we effectively average it over time in this manner. You can increase aa if you want to experiment. 
//too high though and the board may become too vibration sensitive.

// Overall gain is read from external pot going into A0 ie Analogue Input 0. Although not absolutely nescessary it servs as a soft-start. Start the pot at zero volts and increase as requied.
float K = 0;
//offset is read from second pot going into A1 ie Analogue Input 1
float offset=0; // 9 degrees offset from vertical for this gyro


// Sampling rate is 100Hz is set later in this code - checked with Scope
float fs=100;
float dt=1/fs; // sampling interval in seconds
boolean timeflag=LOW; // for debug with scope
////////////////////////////////////////////////////
// First-OrderLow pass filter cut-off 1/(2*pi*tau) Hz, unity gain. ie 1/(1+s.tau)
//Cut-of freq in Hz, sampling freq fs=100Hz. This is for a steep roll-off to prevent resonances and noise shaking the thing apart.
float fcut=14;
float tau=1/(2*3.14*fcut);
float gnum= (1-(2*tau/dt));
float gden = (1+(2*tau/dt));
float filt_par=gnum/gden;
// gain is 1/ gden 
float filt_gain=1/gden;
float filt=0;
float filtp=0;
float filt_out=0;

/////////////////////////////////////////
// Filter to filter the Gyro - not too much or phase shift will cause instability. We use an IIR filter and not an FIR Savitsky Golay filter.
// The reason is that the Savitsky Golay filter is non-minimum phase and hence has a lot of negative going phase which de-stabilises loop. If we make the cut-off
// of our IIR filter high enough it will do a small amount of filtering with less phase-shift. Never use FIR filters in a control loop. The unity gain bandwidth
// of this loop is about 1Hz, so 14 times higher at 14Hz cut-off (-45 degrees phase shift there)  gives us only -45/12 = -3.2 degrees phase shift at 1Hz - very roughly since it's not linear phase. You can move this up but best not to move it down in frequency.
float fcutg=fcut;
float taug=1/(2*3.14*fcutg);
float gnumg= (1-(2*taug/dt));
float gdeng = (1+(2*taug/dt));
float filt_parg=gnumg/gdeng;
// gain is 1/ gdeng 
float filt_gaing=1/gdeng;
float filtg=0;
float filtgp=0;
float filt_outg=0;
///////////////////////////////////////////
unsigned long loopStartTime = 0;
// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// === INITIAL SETUP ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
    while (!Serial); 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(5954);
    mpu.setYGyroOffset(265);
    mpu.setZGyroOffset(-59);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
//5954, 265, -59


    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
    
    // for dc motor 1
    pinMode(10, OUTPUT);
    pinMode(9, OUTPUT);
     pinMode(8, OUTPUT);
     
     // dc motor 2
    
    
    pinMode(7, OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(5,OUTPUT);
    ////////////////////////////////////////////
    // to find sampling time from a scope
    pinMode(12,OUTPUT);
   //////////////////////////////////////////////
   
   // external Pots
    //   A0  for Pot Gain adjust.
    pinMode(pot1Pin, INPUT);
    
    
    // This one is for Pot offset for gyro pot into A1- an adjust for vertical so it doesn't lean
    pinMode(pot2Pin,INPUT);
        
  }

// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================

void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetGyro(gyro, fifoBuffer);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
           // Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\n");
            //Serial.println(ypr[2] * 180/M_PI);
#endif


 //XXXXXXXXXXXXXXXXXXXXX Set Sampling rate to 100Hz:  XXXXXXXXXXXXXXX
    lastLoopUsefulTime = millis() - loopStartTime;
   // millis() returns in milliseconds - time since program started
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
   }
    
    lastLoopTime = millis() - loopStartTime;
   loopStartTime = millis();  
   
         //Serial.print(lastLoopUsefulTime); 
     //Serial.print(",");   
 
 //////////////////////////////////////////////////////
 // Toggle this flag to measue sampling interval from pin 12. Only for debug
   //timeflag=!timeflag;
   
    //digitalWrite(12,timeflag);
 /////////////////////////////////////////////////////////   
    
    
    //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX   
    
    // Overall gain control via external pot
        K = map(analogRead(pot1Pin), 0, 1024, 0, 255);
        
     //offset Pot
     
        offset = map(analogRead(pot2Pin), 0, 1024, 0, 255);
        
    // angle of tilt in degrees - print it out the serial port and check when the gain is zero (to trim offset) or when it's running poperly.     
       currAnglep=currAngle;
        currAngle = (double)((ypr[1] * 57.29)+9+0.02*offset-13); // 9 degrees is my offset but yours may be different. If you put a pot then the offset voltage from it can trim it out.
        //-13 ensima
        Serial.print("Angulo = ");
        Serial.print(currAngle);
        Serial.print("\n");
        // now filter this to get a steadier reading. Ye cannae re-write the laws o physics captain. Dinnae filter too much o it will cause too much phase shift.
        //Serial.print("\n");
        //Serial.print("ypr =");
        //Serial.print(ypr[0]); // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
        //Serial.print(ypr[1]); // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
        //Serial.print(ypr[2]); // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
        //Serial.print("gyro");
        //Serial.print(gyro[0]); // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
        //Serial.print(gyro[1]); // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
        //Serial.print(gyro[2]); // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
        Serial.print("\n");



      
  filtgp=filtg;
  filtg=-filtgp*filt_parg+ filt_gaing*(currAngle + currAnglep);
        
        
   angular_ratep=angular_rate;
   angular_rate = -((double)gyro[1]*0.007633); //  velocity of tilt - note minus sign due to gavity.
            
     // now integrate angular_rate to give a second estimate of angle from the Gyro. Use a simple Euler Integration but don't bother putting the sample interval since
     // it can be omitted and the output scaled later - it is only a gain term after all. of the form new_value=old_value+dt*input

     est_anglep=est_angle;
     est_angle=constrain(est_anglep+angular_rate,-255,255);
     
   ////////////////////////////////////////////////////////  
     //Complementary Filter - we combine both estimates to stop drift problems. Mostly gyro but a bit of Accelerometer.
     // first part is low pass and second is high-pass filter. Low pass filter the Gyro and high-pass filter the Accelerometer.dt=0.01second sampling interval
     
     filtg=constrain((float)(aa_const*(filtg + angular_rate*dt) +(1- aa_const)*est_angle),-255,255);
    ///////////////////////////////////////////////////////
   
   // differentiate velocity to give acceleration which is another state. Simple crude Euler differentiator (first difference) used here is not a great idea. You should never
   // use a pure differentiator since it amplifies noise and resonant frequencies. I low-pass filter the output therefore. first order only because 2nd ode gives too much phase-shift and de-stabilises the loop.
  angular_accelnp=angular_acceln;
  angular_acceln=angular_rate-angular_ratep;
  
   // now filter this with a low-pass filter
  filtp=filt;
  filt=-filtp*filt_par+angular_acceln + angular_accelnp;
    
   filt_out=filt_gain*filt;
  
  
  ////////////////////////////////////////////
  // Apply state-feedback + integral action control law to Motors
          
        Drive_Motor(updateSpeed());
        //Serial.print("speed: "); //normalmente comentado
        //Serial.println(updateSpeed()); //normalmente comentado
    }
}

float updateSpeed() {
    // K is overall gain potentiometer  for trial and error setting up. can also act as a soft-start - start it at zero, hold it vertical and wind up the gain.
    // at the same time you can tweek the offset pot so that it doesn't lean to on side and tries to run off.
    
    


// setpoint is zero - setpoint minus  IIR filtered Gyro value

 error=setpoint-filtg;
  
    
   // integrated error is the fourth state
   // now integrate angular_rate to give a second estimate of angle from the Gyro. Use a simple Euler Integration but don't bother putting the sample interval since
     // it can be omitted and the output scaled later - it is only a gain term after all. of the form new_value=old_value+dt*input
   integrated_errorp= integrated_error;
  integrated_error =  constrain(integrated_errorp + Ki*error,-64,64);
  
// filtg is the filtered Gyro sensor IIR output to give a steadier reading - kind of inertia.
  control_out=K*0.04*(integrated_error -(Kp*filtg  + Kd*angular_rate + Ka*filt_out));
   

  //Velocidad motores
  return constrain(control_out, -32,32);
  //  return constrain(control_out, -255,255);
}

float Drive_Motor(float torque)  {
  //Serial.print("torque: ");
 // Serial.println(torque);
  if (torque >= 0)  {                                        // drive motors forward
    digitalWrite(9, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(6, HIGH);
  }  else {                                                  // drive motors backward
    digitalWrite(9, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(6, LOW);
    torque = abs(torque);
  }
  // PWM for the dc motors - same drive for both at pesent since no steering.
  
  analogWrite(10,torque);
 analogWrite(5,torque); 
  
}

