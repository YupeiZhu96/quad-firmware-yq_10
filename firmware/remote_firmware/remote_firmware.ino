#include <radio.h>
#include <serLCD.h>
const int y = 509;
const int t = 531;
const int p = 501;
const int r = 504;
serLCD lcd;
void setup() {
  lcd.clear();
  lcd.display();
  lcd.setBrightness(5);
  Serial.begin(115200);
  rfBegin(21);
  pinMode(16, INPUT_PULLUP); 
}
int clamp(int c, int threhold){
  if(c > threhold) return threhold;
  if(c < 0) return 0;
  return c;
}
void setVals(int val, char type, char* msg){
  char val1 = (0xFF00 & val)>>8;
  char val2 = 0x00FF & val;
  msg[0] = type;
  msg[1] = val1;
  msg[2] = val2;
}
void loop() {
  int button1Value = digitalRead(16); 
  int yaw = analogRead(0) - y;
  int throttle = analogRead(1) - t;
  int roll = analogRead(2) - r;
  int pitch = analogRead(3) - p;
  Serial.println(button1Value);
  char str[] = "sss";
  if(button1Value == 0) rfWrite(str,3);
  
  yaw = yaw < 0 ? map(yaw, 137-y, 0, 0, 749) : map(yaw, 0, 817-y, 750, 1500);
  throttle = throttle < 0 ? map(throttle, 123-t, 0, 0, 749) : map(throttle, 0, 817-t, 750, 1500);
  roll = roll < 0 ? map(roll, 129-r, 0, 0, 749) : map(roll, 0, 817-r, 750, 1500);
  pitch = pitch < 0 ? map(pitch, 124-p, 0, 0, 749) : map(pitch, 0, 817-p, 750, 1500);


  yaw = clamp(yaw, 1500);
  throttle = clamp(throttle, 1500);
  roll = clamp(roll,1500);
  pitch = clamp(pitch,1500);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T: ");
  lcd.print(throttle);
  lcd.setCursor(0,8);
  lcd.print("P: ");
  lcd.print(pitch);
  lcd.setCursor(1,0);
  lcd.print("Y: ");
  lcd.print(yaw);
  lcd.setCursor(1,8);
  lcd.print("R: ");
  lcd.print(roll);
  

  char toSend[12];
  setVals(yaw, 'y', toSend);
  setVals(throttle, 't', &toSend[3]);
  setVals(roll, 'r', &toSend[6]);
  setVals(pitch, 'p', &toSend[9]);
  rfWrite(toSend, 12);
  delay(100);
}
