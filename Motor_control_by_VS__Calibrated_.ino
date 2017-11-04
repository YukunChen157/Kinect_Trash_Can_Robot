#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 //LED to show bluetooth status
/***************************/
MPU6050 mpu;
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorFloat gravity;
Quaternion q;
float ypr[3]; 
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
/****Till this line is all gyroscope setting***/

float bearingOffset;   //bearing is the heading angle, so when bearing offset is the direction that the robot facing to at the beggining
Servo motor1, motor2, motor3;
int val1, val2, val3;   //speed parameter for the motor, which is range from 0 to 180 and has middle at 90
int val1p=90, val2p=90,val3p=90;    //val-previous record the motor parameter from the last run
int headingAngle, distance;         //the angle is the direction the robot heading to (couting clockwise from heading offset, range 0~360), distance is in millimeter, both variable from bluetooth signal 
char Char;
String message;       //message from the bluetooth signal
float runningTime;    //running time of the motors to get to certain distance
double t1,t2, t3, t4, t5;
bool ending = false, moving = false;
int motorParameter[12][3] = {{90,66,110}, {73,73,110}, {64,90,112}, {64,103,104},
                              {66,108,90}, {72,108,73}, {90,107,66}, {102,103,65}, 
                              {108,90,64}, {109,72,72}, {108,65,90}, {104,65,103}};
                              //the preset parameter for motor1, 2 and 3 when the robot is heading to 0, 30, 60,...,330 degree clockwise from the center
float motorRunningTime[12][8] = {{8,16,26,40,50,65,87,92},{3,7,12,18,25,30,35,40},
                                 {11,16,25,36,50,61,67,73},{10,14,22,32,41,50,61,75},
                                 {10,17,27,39,47,61,72,87},{4,7,11,19,23,33,45,50},
                                 {4,10,17,21,33,38,42,50},{3,7,11,18,25,37,49,57},
                                 {8,17,27,38,50,60,75,88},{6,12,20,30,36,43,52,62},
                                 {10,14,22,35,42,57,62,67},{6,12,20,30,41,47,55,66}};
                                 //the preset time parameter, with columns showing situations when the robot heading to  0, 30, 60,...,330 degree, and rows showing how far the robot can go in centimeter in 0.3, 0.4,...,1.0 sec)
float GyroOutput();         //The output from the gyroscope, in degree, range from 0 to 360

void dmpDataReady() {
    mpuInterrupt = true;
}   


void motorSetup(){          //set all motors in middle (90) when the gyroscope is calibrating, it is used to initiate and activate the motor
  Serial.println("Motor Ready");
  val1 = 90; 
  val2 = 90;
  val3 = 90;
  motor1.write(val1);                  // sets the motors' initial speed to zero 
  motor2.write(val2);
  motor3.write(val3);
}
 
void setup() {          //initiate the gyroscope, code copied online
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kH z if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 //  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
 // while (!Serial.available());                 // wait for data
 // while (Serial.available() && Serial.read()); // empty buffer again
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  motor1.attach(12,1000,2000);  // attaches the motor on pin 13 to the servo object 
  motor2.attach(11,1000,2000);  // attaches the motor on pin 12 to the servo object 
  motor3.attach(10,1000,2000);  // attaches the motor on pin 11 to the servo object
  motorSetup();
  t1 = millis();
  t4 = millis();
  t3 = millis();
  while((t3-t1)<20000){   //calibrate the gyroscope for 20 seconds, wait for the output to get stable
    GyroOutput();
    t3=millis();
  } 
  bearingOffset = -1.0*GyroOutput();    //set the initial angle the robot facing to
  Serial.print("Callibrated\nBearing Offset: ");Serial.println(bearingOffset);
}

float GyroOutput(){     //get the output from gyroscope, code copied online, the result is float ranging from 0 to 360
  float yaw = 0;
  mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    yaw = ypr[0] * 180/M_PI;
    //Serial.print("ypr\t");Serial.println(yaw);
    return yaw;
}

void getRunningTime(){      //method to get the running time of three motors in order to get to certain angle and distance
                            //basically the idea is to assume the motors' runtime is linear between two angles and two adjacent time
  int i = headingAngle/30;
  float range[2][8];
  float timeA, timeB;
  for(int j=0; j<8; j++){
    range[0][j]=motorRunningTime[i][j];
  }
  if(i==11){
    for(int j=0; j<8; j++){
      range[1][j]=motorRunningTime[0][j];
    }
  }
  else{
    for(int j=0; j<8; j++){
      range[1][j]=motorRunningTime[1][j];
    }
  }
  int A=-1, B=-1;
  float lowerBoundA=0, lowerBoundB=0, upperBoundA, upperBoundB;
  for(int j=0; j<8; j++){
    if(distance>range[0][j]*100){
      A=j;
    }
    if(distance>range[1][j]*100){
      B=j;
    }
  }
  if(A==-1){
    upperBoundA = range[0][0];
    timeA = distance/(upperBoundA*100)*0.3;
  }
  else if(A<7){
    lowerBoundA = range[0][A];
    upperBoundA = range[0][A+1];
    timeA = 0.3+0.1*A+0.1*(distance-lowerBoundA*100)/((upperBoundA-lowerBoundA)*100);
  }
  else{
    timeA = 1;
  }
  if(B==-1){
    upperBoundB = range[1][0];
    timeB = distance/(upperBoundB*100)*0.3;
  }
  else if(B<7){
    lowerBoundB = range[1][B];
    upperBoundB = range[1][B+1];
    timeB = 0.3+0.1*B+0.1*(distance-lowerBoundB*100)/((upperBoundB-lowerBoundB)*100);
  }
  else{
    timeB = 1;
  }
  runningTime = timeA+(timeB-timeA)*(headingAngle-i*30)/30;
  Serial.print("Running Time is: ");Serial.println(runningTime);
}

/*************MAIN LOOP**************/
void loop(){
  /***********PROCEED THE BLUETOOTH DATA, SET ANGLE AND DISTANCE*********************************/
  
  
  /************************************ bluetooth input with space*****************************
   if(Serial.available()){    //while there is a char data available from bluetooth port, store the data in message
    Char = Serial.read();
    if(Char==' '){       //' ' (space) is the keyword to show the end of message
      moving = false;
      ending = true;
    }
    else{
      message += Char;
      t1 = millis();
      Serial.println(message);
    }
  }
  t2 = millis();
  
  if(ending){       //if there is a space
    if(message.length()==7){      //check whether the message has 7 digits
      long temp = message.toInt();
      headingAngle = temp/10000;
      distance = temp-headingAngle*10000;         //the first 3 digits is angle, ranging from 0 to 360, the last 4 digits is distance in millimeter
      Serial.print("Heading: ");Serial.print(headingAngle);Serial.print("  Distance : ");Serial.println(distance);
      moving = true;
      getRunningTime();
      t4 = millis();
    }
    ending = false;
    message = "";
  }
  **************************************************************************************************/

  /***************************bluetooth input without space*****************************************/
  if(Serial.available()){    //while there is a char data available from bluetooth port, store the data in message
    Char = Serial.read();
    if(message.length()==7){       //' ' (space) is the keyword to show the end of message
      long temp = message.toInt();
      headingAngle = temp/10000;
      distance = (temp-headingAngle*10000)*10;         //the first 3 digits is angle, ranging from 0 to 360, the last 4 digits is distance in millimeter
      Serial.print("Heading: ");Serial.print(headingAngle);Serial.print("  Distance : ");Serial.println(distance);
      moving = true;
      getRunningTime();
      t4 = millis();
      message = "";
    }
    else{
      message += Char;
      t1 = millis();
      Serial.println(message);
    }
  }
  /************SET MOTOR PARAMETER FOR EACH MOTOR***************************************************/
  int range[2][3];
  if(moving){
    int i=headingAngle/30;
    for(int j=0; j<3; j++){
      range[0][j]=motorParameter[i][j];
    }
    if(i==11){
      for(int k=0; k<3; k++){
        range[1][k]=motorParameter[0][k];
      }
    }
    else{
      for(int k=0; k<3; k++){
        range[1][k]=motorParameter[i+1][k];
      }
    }
    val1 = range[0][0]+((range[1][0]-range[0][0])*(headingAngle-i*30)+15)/30;
    val2 = range[0][1]+((range[1][1]-range[0][1])*(headingAngle-i*30)+15)/30;   
    val3 = range[0][2]+((range[1][2]-range[0][2])*(headingAngle-i*30)+15)/30;     
  }
  else{
    val1 = 90;
    val2 = 90;
    val3 = 90;
  }
  /*************************************************************************************************/

  /**************THIS STEP IS TO ENSURE THE ROBOT IS ALWAYS FACING TO THE SAME DIRECTION*****************/
  float bearing = GyroOutput()+bearingOffset;      //compute the heading angle
  if(bearing<-180){
    bearing+=360;
  }
  else if(bearing>180){
    bearing-=360;
  }
  int angleCorrectSpeed = bearing*0.2;      //if the heading angle is not in th middle, there is a correction parameter for the motor, which is 0.2 for 1 degree
  val1+=angleCorrectSpeed;
  val2+=angleCorrectSpeed;
  val3+=angleCorrectSpeed;        //add the correction speed to the motor
  /******************************************************************************************************/
  
  t5=millis();
  if((t5-t4)>(runningTime*1000)){     //turn all motors back to 90 when exceeding the runtime
    val1=90;
    val2=90;
    val3=90;
  }
  if(val1!=val1p || val2!=val2p || val3!=val3p){          //print out the motors' parameters when they are different from the previous loop
    Serial.print(val1);Serial.print(" ");Serial.print(val2);Serial.print(" ");Serial.println(val3);
  }
  val1p=val1;
  val2p=val2;
  val3p=val3;
  motor1.write(val1);           //write the motors' parameters to the motors
  motor2.write(val2);
  motor3.write(val3);
}
