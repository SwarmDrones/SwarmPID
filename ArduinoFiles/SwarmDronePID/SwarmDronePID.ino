#include <PID_v1.h>
#include <DroneMotor.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//////////////////////////////////////////Time related/////////////////////////////////////
double cTime, pTime, dTime; // current, previous, and delta time

///////////////////////////////////////motors stuff///////////////////////////////////////
DroneMotor m1(5, 16, 21);  // back left
DroneMotor m2(A7, 16, 21); // front left
DroneMotor m3(6, 16, 21);  // back right
DroneMotor m4(A6, 16, 21); // front right
double mspeed = 10.0;

///////////////////////////////////////PID stuff/////////////////////////////////////
double desiredRoll, desiredPitch, desiredYaw;// setpoint
double actualRoll, actualPitch, actualYaw;// angle read from imu
double msRoll, msPitch, msYaw; // motor speed output

// roll K's, aggressive and conservative
double aKpRoll=4, aKiRoll=0.2, aKdRoll=1;
double cKpRoll=1, cKiRoll=0.05, cKdRoll=0.25;

// pitch K's, aggressive and conservative
double aKpPitch=4, aKiPitch=0.2, aKdPitch=1;
double cKpPitch=1, cKiPitch=0.05, cKdPitch=0.25;

// yaw K's, aggressive and conservative
double aKpYaw=4, aKiYaw=0.2, aKdYaw=1;
double cKpYaw=1, cKiYaw=0.05, cKdYaw=0.25;

void pidSetup();
void updatePID();

// PID inits
PID rPID(&actualRoll, &msRoll, &desiredRoll, cKpRoll, cKiRoll, cKdRoll, DIRECT);
PID pPID(&actualPitch, &msPitch, &desiredPitch, cKpPitch, cKiPitch, cKdPitch, DIRECT);
PID yPID(&actualYaw, &msYaw, &desiredYaw, cKpYaw, cKiYaw, cKdYaw, DIRECT);

///////////////////////////////////////IMU stuff/////////////////////////////////////
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void imuSetup();
void updateIMU();
///////////////////////////////////////////////////////////////////////////////////////
/************************************SETUP********************************************/
///////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  m1.setupMotor();
  m2.setupMotor();
  m3.setupMotor();
  m4.setupMotor();
  mspeed = 0.00;
  pidSetup();
  Serial.println("PID setup finished");
  imuSetup();
  Serial.println("IMU setup finished");
  
  
}
///////////////////////////////////////////////////////////////////////////////////////
/************************************LOOP ********************************************/
///////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly:
  updateIMU();
  updatePID();
  //m1.setSpeed(mspeed-msRoll);
  //m2.setSpeed(mspeed-msRoll);
  //m3.setSpeed(mspeed+msRoll);
  //m4.setSpeed(mspeed+msRoll);
  //m1.setSpeed(mspeed-msPitch);
  //m2.setSpeed(mspeed+msPitch);
  //m3.setSpeed(mspeed-msPitch);
  //m4.setSpeed(mspeed+msPitch);

  m1.setSpeed(mspeed - msPitch - msRoll);
  m2.setSpeed(mspeed + msPitch - msRoll);
  m3.setSpeed(mspeed - msPitch + msRoll);
  m4.setSpeed(mspeed + msPitch + msRoll);
  
}

void pidSetup()
{
  // SETTING FOR UNINIT GLOBAL PID VARIABLES
  desiredRoll = 0.00;
  desiredPitch = 0.00;
  desiredYaw = 0.00;
  actualRoll = 0.00;
  actualPitch = 0.00;
  actualYaw = 0.00;
  msRoll = 0.00;
  msPitch = 0.00;
  msYaw = 0.00; 
  
  // SETTING FOR CONTROLLERS
  rPID.SetMode(AUTOMATIC);
  pPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  rPID.SetOutputLimits(-30.00+mspeed, 30.00-mspeed);
  pPID.SetOutputLimits(-30.00+mspeed, 30.00-mspeed);
  yPID.SetOutputLimits(-30.00+mspeed, 30.00-mspeed);
  
}
void updatePID()
{
  double rDiff = abs(desiredRoll - actualRoll);
  double pDiff = abs(desiredPitch - actualPitch);
  double yDiff = abs(desiredYaw - actualYaw);
  // UPDATING THE TUNNING VALUES;
  if(rDiff < 10.0) rPID.SetTunings(cKpRoll, cKiRoll, cKdRoll);
  else rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);
  
  if(rDiff < 10.0) rPID.SetTunings(cKpRoll, cKiRoll, cKdRoll);
  else rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);
  
  if(rDiff < 10.0) rPID.SetTunings(cKpRoll, cKiRoll, cKdRoll);
  else rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);
  // COMPUTING PID OUTPUT;
  rPID.Compute();
  pPID.Compute();
  yPID.Compute();
  
}
void imuSetup()
{
  bno.begin();
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
  sensors_event_t event;
  bno.getEvent(&event);
  delay(BNO055_SAMPLERATE_DELAY_MS);

  actualRoll = event.orientation.y;// will start at 0.00 and range is 90 to -90
  actualPitch = event.orientation.z;// will start at 180/-180 and range is -180 to 180
  actualYaw = event.orientation.x; //will be 0.00 when facing north and range is 0 to 360
}
void updateIMU()
{
  sensors_event_t event;
  bno.getEvent(&event);
  delay(BNO055_SAMPLERATE_DELAY_MS);

  actualRoll = event.orientation.y;
  actualPitch = event.orientation.z;
  actualYaw = event.orientation.x;
}


