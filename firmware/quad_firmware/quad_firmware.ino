#include <radio.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LEFT_DOWN_MOTOR 8
#define RIGHT_DOWN_MOTOR 5
#define RIGHT_UP_MOTOR 4
#define LEFT_UP_MOTOR 3
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
float yaw = 0;
float throttle = 0;
float roll = 0;
float pitch = 0;
float yaw_set = 0;
float throttle_set = 0;
float roll_set = 0;
float pitch_set = 0;

float motor[4] = {0};
long f_prev = 0;
float const PI_F = 3.14159265F;

void ComplementaryFilter(sensors_event_t a, sensors_event_t g, float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
    float dt = (millis() - f_prev)/ 1000.0; f_prev = millis();
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)g.gyro.x) * dt; // Angle around the X-axis
    *roll -= ((float)g.gyro.y) * dt;    // Angle around the Y-axis
    int forceMagnitudeApprox = abs(a.acceleration.x) + abs(a.acceleration.y) + abs(a.acceleration.z);
    if (forceMagnitudeApprox > 5 && forceMagnitudeApprox < 20 )
    {
    // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)a.acceleration.y, (float)a.acceleration.z) * 180 / PI_F;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
    // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)a.acceleration.x, (float)a.acceleration.z) * 180 / PI_F;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} 

unsigned char buff[255];
int val1 = -1;
int val2 = -1;
char cmd = 0;
void readVals(){
  int count = 0;
  if(rfAvailable()){
    count = rfRead(buff, 255);
    for(int i = 0; i < count; i++){
      if(cmd == 0){
        char p = buff[i];
        if(p == 'y' || p == 't' || p == 'r' || p == 'p') cmd = p;
      }else if(val1 == -1){
        val1 = buff[i];
      }else if(val2 == -1){
        val2 = buff[i];
        unsigned int val = ((unsigned int)val1 << 8) + val2;
        char p = cmd;
        val1 = -1;
        val2 = -1;
        cmd = 0;
        if(val > 1500||val < 0) continue;
        else{
          //Serial.println(val);
          if(p == 'y'){
            yaw_set = (float)val/1500.0*90.0-45.0;
          }
          if(p == 't'){
            throttle_set = val/1500*255;
          }
          if(p == 'r'){
            roll_set = (float)val/1500.0*90.0-45.0;
          }
          if(p == 'p'){
            pitch_set = (float)val/1500.0*90.0-45.0;
          }
        }
      }
    }
  }
}

class PIDCtrl
{
  public:
    float Kp, Ki, Kd;
    float lastError, integError;
    float compP, compI, compD;
    long prevTime;
    
    PIDCtrl(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        prevTime = millis();
        ResetCtrl();
    }
    
    void ResetCtrl()
    {
        lastError = 0.0;
        integError = 0.0;
    }
    
    float Update(float error)
    {
        float dt = millis() - prevTime;
        prevTime = millis();
        integError /= 2;
        integError += error * dt/100;
        compP = error * Kp;
        compI = integError * Ki;
        compD = ((error - lastError) / dt*1000) * Kd;
        lastError = error;
        return compP + compI + compD;
    }
    
    float ComponentP() {
        return compP;
    }
    float ComponentI() {
        return compI;
    }
    float ComponentD() {
        return compD;
    }
    
};

PIDCtrl PID_roll_1(3,0,1.3);
PIDCtrl PID_roll_2(3,0,1.3);
//PIDCtrl PID_yaw(2,0.01,0.001);

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}
float rolls[10];
int r_index = 0;
float pitches[10];
int p_index = 0;
bool has_input = 0;

void calRollAndPitch(sensors_event_t a, sensors_event_t g){
  rolls[r_index] = (float)atan2(a.acceleration.x, a.acceleration.z)/PI_F * 180.0;
  pitches[p_index]= (float)atan2(a.acceleration.y, a.acceleration.z)/PI_F * 180.0;
  //ComplementaryFilter(a,g,&pitches[p_index],&rolls[r_index]);
  r_index = (r_index + 1)%10;
  p_index = (p_index + 1)%10;
  
  float tmp[10];
  for(int i = 0; i < 10;i++){
    float tmp_r = rolls[i];
    int count = 0;
    for(int j = 0; j < 10; j++){
      if(tmp_r > rolls[j]) count++;
    }
    tmp[count] = tmp_r;
  }
  roll = (tmp[4]+tmp[5])/2.0;

  for(int i = 0; i < 10;i++){
    float tmp_p = pitches[i];
    int count = 0;
    for(int j = 0; j < 10; j++){
      if(tmp_p > pitches[j]) count++;
    }
    tmp[count] = tmp_p;
  }
  pitch = (tmp[4]+tmp[5])/2.0;
}

void setup() {
   pinMode(3, OUTPUT); 
   pinMode(4, OUTPUT); 
   pinMode(5, OUTPUT); 
   pinMode(8, OUTPUT); 
   Serial.begin(115200);  // Start up serial
   Serial1.begin(230400);
   rfBegin(21);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
   
   lsm.begin();
   setupSensor();
   lsm.read();
   sensors_event_t a, m, g, temp;
   lsm.getEvent(&a, &m, &g, &temp); 
   pitch = 0;
   roll = 0;
   f_prev = millis();
   motor[0] = 150;
   motor[1] = 150;
   motor[2] = 90;
   motor[3] = 130;
}

int clamp(int a){
  if (a < 0) return 0;
  if (a > 255) return 255;
  return a;
}

void loop() {
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  calRollAndPitch(a, g);
  //ComplementaryFilter(a,g,&pitch,&roll);
  float roll_error = (roll - roll_set);
  float drive_1 = PID_roll_1.Update(roll_error);
  float drive_2 = PID_roll_2.Update(roll_error);

  Serial.print("roll: ");Serial.println(roll);

  analogWrite(3, clamp(motor[2]));
  analogWrite(4, clamp(motor[3]));
  analogWrite(5, clamp(motor[1]+drive_2));
  analogWrite(8, clamp(motor[0]+drive_1));

  readVals();
  //delay(5);
}

