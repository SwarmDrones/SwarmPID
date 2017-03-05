#include <PID_v1.h>
#include <DroneMotor.h>
#include <DroneCom.h>
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
void setTunningsFromSring(int axis, String str);
// PID inits
PID rPID(&actualRoll, &msRoll, &desiredRoll, cKpRoll, cKiRoll, cKdRoll, DIRECT);
PID pPID(&actualPitch, &msPitch, &desiredPitch, cKpPitch, cKiPitch, cKdPitch, DIRECT);
PID yPID(&actualYaw, &msYaw, &desiredYaw, cKpYaw, cKiYaw, cKdYaw, DIRECT);

///////////////////////////////////////IMU stuff/////////////////////////////////////
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void imuSetup();
void updateIMU();
///////////////////////////////////////COMM  stuff/////////////////////////////////////
DroneCom comm;
void check4Incoming();
void check4Outgoing();

///////////////////////////////////////////////////////////////////////////////////////
/************************************SETUP********************************************/
///////////////////////////////////////////////////////////////////////////////////////
void setup() {
  comm.init();
  //Serial.println("Communication setup finished");
  m1.setupMotor();
  m2.setupMotor();
  m3.setupMotor();
  m4.setupMotor();
  mspeed = 0.00;
  //Serial.println("Motor setup finished");
  pidSetup();
  //Serial.println("PID setup finished");
  imuSetup();
  //Serial.println("IMU setup finished");
}
///////////////////////////////////////////////////////////////////////////////////////
/************************************LOOP ********************************************/
///////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // UPDATE TIME
  pTime = cTime;
  cTime = millis();
  dTime = cTime-pTime;
  /////////////////////////////////////////////IMU/////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  updateIMU();
  /////////////////////////////////////////////PID/////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  updatePID();
  /////////////////////////////////////////////MOTORS/////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  m1.setSpeed(mspeed - msPitch - msRoll);
  m2.setSpeed(mspeed + msPitch - msRoll);
  m3.setSpeed(mspeed - msPitch + msRoll);
  m4.setSpeed(mspeed + msPitch + msRoll);
  //m1.setSpeed(mspeed-msRoll);
  //m2.setSpeed(mspeed-msRoll);
  //m3.setSpeed(mspeed+msRoll);
  //m4.setSpeed(mspeed+msRoll);
  //m1.setSpeed(mspeed-msPitch);
  //m2.setSpeed(mspeed+msPitch);
  //m3.setSpeed(mspeed-msPitch);
  //m4.setSpeed(mspeed+msPitch);
  /////////////////////////////////////////////COMMUNICATION//////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  check4Incoming();
  check4Outgoing();
}
///////////////////////////////////////////////////////////////////////////////////////
/************************************PID METHODS**************************************/
///////////////////////////////////////////////////////////////////////////////////////
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
  /*if(rDiff < 10.0) rPID.SetTunings(cKpRoll, cKiRoll, cKdRoll);
  else rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);
  
  if(rDiff < 10.0) rPID.SetTunings(cKpRoll, cKiRoll, cKdRoll);
  else rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);
  
  if(rDiff < 10.0) rPID.SetTunings(cKpRoll, cKiRoll, cKdRoll);
  else rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);*/
  // COMPUTING PID OUTPUT;
  rPID.Compute();
  pPID.Compute();
  yPID.Compute();
  
}
///////////////////////////////////////////////////////////////////////////////////////
/************************************IMU METHODS**************************************/
///////////////////////////////////////////////////////////////////////////////////////
void imuSetup()
{
  bno.begin();
  sensor_t sensor;
  bno.getSensor(&sensor);
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

///////////////////////////////////////////////////////////////////////////////////////
/************************************COMM METHODS**************************************/
///////////////////////////////////////////////////////////////////////////////////////
void check4Incoming()
{
  //Serial.println("checking incoming");
  comm.updateRXMsg();
  if(comm.checkInFlag() == true)
  {
      //Serial.println("incoming message is true");
      // checking if orientation or position PID
      if(comm.OriMsgIn == true)
      {
        // checking if Roll Pitch or Yaw is recieved
        if(comm.RollMsgIn == true)
        {
          // checking if P||I||D is recieved
          if(comm.PMsgIn == true)
          {
            //Set P value
            String str = comm.getPIDMsgVal();
            aKpRoll = str.toFloat();
          }
          else if(comm.IMsgIn == true)
          {
            //Set I value
            String str = comm.getPIDMsgVal();
            aKiRoll = str.toFloat();
          }
          else if(comm.DMsgIn == true)
          {
            //Set D value
            String str = comm.getPIDMsgVal();
            aKdRoll = str.toFloat();
          }
          // update the orientation PID vals
          rPID.SetTunings(aKpRoll, aKiRoll, aKdRoll);
          
        }
        else if(comm.PitchMsgIn == true)
        {
          // checking if P||I||D is recieved
          if(comm.PMsgIn == true)
          {
            //Set P value
            String str = comm.getPIDMsgVal();
            aKpPitch = str.toFloat();
          }
          else if(comm.IMsgIn == true)
          {
            //Set I value
            String str = comm.getPIDMsgVal();
            aKiPitch = str.toFloat();
          }
          else if(comm.DMsgIn == true)
          {
            //Set D value
            String str = comm.getPIDMsgVal();
            aKdPitch = str.toFloat();
          }
          // update the orientation PID vals
          pPID.SetTunings(aKpPitch, aKiPitch, aKdPitch);
        }
        else if(comm.YawMsgIn == true)
        {
          // checking if P||I||D is recieved
          if(comm.PMsgIn == true)
          {
            //Set P value
            String str = comm.getPIDMsgVal();
            aKpYaw = str.toFloat();
          }
          else if(comm.IMsgIn == true)
          {
            //Set I value
            String str = comm.getPIDMsgVal();
            aKiYaw = str.toFloat();
          }
          else if(comm.DMsgIn == true)
          {
            //Set D value
            String str = comm.getPIDMsgVal();
            aKdYaw = str.toFloat();
          }
          // update the orientation PID vals
          yPID.SetTunings(aKpYaw, aKiYaw, aKdYaw);
        }
      }

      //Serial.println("checked for ori input");
      if(comm.PosMsgIn == true)
      {
        //Serial.println("orientation PID value in");
        comm.resetFlags();
      }
      //Serial.println("checked for Pos input");
      if(comm.DestMsgIn == true)
      {
        //Serial.println("destination values in");
        comm.resetFlags();
      }
      //Serial.println("done checking incoming");
  }
      
}
void check4Outgoing()
{

  //TODO: OUTPUT ORIENTATION AND POSITION MESSAGES OUT TO COORDINATE
  //Serial.println("output Orientation");
  comm.sendOrientation(actualRoll, actualPitch, actualYaw);
  //comm.transmit2Coor("hello");
}


