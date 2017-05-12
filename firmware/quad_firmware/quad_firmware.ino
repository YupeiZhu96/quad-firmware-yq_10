#include <radio.h>
#include <Adafruit_LSM9DS1.h>
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

float yaw = 0;
float roll = 0;
float pitch = 0;
float yaw_set = 0;
float roll_set = 0;
float pitch_set = 0;
bool start = false;

unsigned char buff[255];
int val1 = -1;
int val2 = -1;
char cmd = 0;

float motor[4] = {0};
long f_prev = 0;
float const PI_F = 3.14159265F;

void ComplementaryFilter(sensors_event_t a, sensors_event_t g, float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
    float dt = (millis() - f_prev)/ 1000.0; f_prev = millis();
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch -= ((float)g.gyro.x) * dt; // Angle around the X-axis
    *roll += ((float)g.gyro.y) * dt;    // Angle around the Y-axis
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


void readVals(){
  int count = 0;
  if(rfAvailable()){
    count = rfRead(buff, 255);
    
    for(int i = 0; i < count; i++){
      if(cmd == 0){
        char p = buff[i];
        if(p == 'y' || p == 't' || p == 'r' || p == 'p' || p == 's') cmd = p;
      }else if(val1 == -1){
        val1 = buff[i];
      }else if(val2 == -1){
        val2 = buff[i];
        unsigned int val = ((unsigned int)val1 << 8) + val2;
        char p = cmd;
        char v1 = val1;
        char v2 = val2;
        val1 = -1;
        val2 = -1;
        cmd = 0;
        if(p != 's' && (val > 1500||val < 0)) continue;
        else{
          //Serial.println(val);
          if(p == 'y'){
            yaw_set = (float)val/1500.0*90.0-45.0;
          }
          if(p == 't'){
            float throttle_set = (float)val/1500.0*90.0;
            motor[0] = 100 + throttle_set;
            motor[1] = 100 + throttle_set;
            motor[2] = 50 + throttle_set * 0.7;
            motor[3] = 80 + throttle_set * 0.8;
          }
          if(p == 'r'){
            roll_set = (float)val/1500.0*90.0-45.0;
          }
          if(p == 'p'){
            pitch_set = (float)val/1500.0*90.0-45.0;
          }
          if(p == 's' && v1 == v2 && v1 == 's'){
            start = !start;
            analogWrite(3, 0);
            analogWrite(4, 0);
            analogWrite(5, 0);
            analogWrite(8, 0);
            //Serial.println(count);
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
    float lastError = 0, integError = 0;
    float compP, compI, compD;
    long prevTime;
    
    PIDCtrl(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        prevTime = millis();
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
    
};

PIDCtrl PID_roll_1(3,0.8,1.3);
PIDCtrl PID_roll_2(3,0.8,1.3);
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
   f_prev = millis();
   motor[0] = 100;
   motor[1] = 100;
   motor[2] = 50;
   motor[3] = 80;
   pitch = atan2f((float)a.acceleration.y, (float)a.acceleration.z) * 180 / PI_F;
   roll = atan2f((float)a.acceleration.x, (float)a.acceleration.z) * 180 / PI_F;
}

int clamp(int a){
  if (a < 0) return 0;
  if (a > 255) return 255;
  return a;
}

void loop() {
  lsm.read();  /* ask it to read in the data */ 

  readVals();
  if(!start) return;
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  ComplementaryFilter(a,g,&pitch,&roll);
  float roll_error = (roll - roll_set);
  float drive_1 = PID_roll_1.Update(roll_error);
  float drive_2 = PID_roll_2.Update(roll_error);

  //Serial.print("roll: ");Serial.println(roll);

  analogWrite(3, clamp(motor[2]));
  analogWrite(4, clamp(motor[3]));
  analogWrite(5, clamp(motor[1]+drive_2));
  analogWrite(8, clamp(motor[0]+drive_1));

  
  //delay(5);
}

