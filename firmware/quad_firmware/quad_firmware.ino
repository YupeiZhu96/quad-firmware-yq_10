#include <radio.h>
#include <serLCD.h>

serLCD lcd;
void setup() {
  lcd.clear();
  lcd.display();
  lcd.setBrightness(5);
  Serial.begin(115200);
  rfBegin(21);
}
int clamp(int c, int threhold){
  if(c > threhold) return threhold;
  if(c < 0) return 0;
  return c;
}
void loop() {
  int yaw = analogRead(0);
  int throttle = analogRead(1);
  int roll = analogRead(2);
  int pitch = analogRead(3);
  
  
  Serial.print("Yaw:");
  Serial.println(yaw);
  Serial.print("Throttle:");
  Serial.println(throttle);
  Serial.print("Roll:");
  Serial.println(roll);
  Serial.print("Pitch:");
  Serial.println(pitch);


  yaw = clamp(map(yaw, 137, 817, 0, 1500), 1500);
  throttle = clamp(map(throttle, 123, 817, 0, 1500), 1500);
  roll = clamp(map(roll, 130, 817, 0, 1500),1500);
  pitch = clamp(map(pitch, 124, 817, 0, 1500),1500);
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
  Serial.print("Yaw:");
  Serial.println(yaw);
  Serial.print("Throttle:");
  Serial.println(throttle);
  Serial.print("Roll:");
  Serial.println(roll);
  Serial.print("Pitch:");
  Serial.println(pitch);
  

  String toSend= "";
  yaw = clamp(map(yaw, 0, 1500, 1, 255),255);
  throttle = clamp(map(throttle, 0, 1500, 1, 255),255);
  roll = clamp(map(roll, 0, 1500, 1, 255),255);
  pitch = clamp(map(pitch, 0, 1500, 1, 255),255);
  toSend+='y';
  toSend+=(char)yaw;
  toSend+='t';
  toSend+=(char)throttle;
  toSend+='r';
  toSend+=(char)roll;
  toSend+='p';
  toSend+=(char)pitch;
  rfPrint(toSend);
  delay(100);
}
