#include <SoftwareSerial.h>
 
SoftwareSerial BTSerial(10, 11); // RX, TX
 
void setup()
{pinMode(3,OUTPUT);
 pinMode(5,OUTPUT);
 pinMode(6,OUTPUT);
 pinMode(9,OUTPUT);
 pinMode(8,OUTPUT);
 digitalWrite(8,HIGH);
 
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
 
  Serial.println("Ready!");
 
  // set the data rate for the SoftwareSerial port
 
  // for HC-05 use 38400 when poerwing with KEY/STATE set to HIGH on power on
  BTSerial.begin(38400);
}
int spd;
String input;
int a;
void loop() // run over and over
{
  if (BTSerial.available()>0)
  {
    input = BTSerial.readString() ;
   // Serial.write(input);
     a =  input.toInt();

    sspd=(a*50/350) ;
    analogWrite(5,spd);
    analogWrite(3,0);
    analogWrite(6,spd);
    analogWrite(9,0);
    delay(10);
    if(spd < 20 && spd > 0){
    analogWrite(3,0);
    analogWrite(5,0);
    analogWrite(6,0);
    analogWrite(9,0);
    }
   
    Serial.print(a);
    Serial.print("    ");
    Serial.println(input); 
  }
    
  if (Serial.available())
    BTSerial.write(Serial.read());
}
