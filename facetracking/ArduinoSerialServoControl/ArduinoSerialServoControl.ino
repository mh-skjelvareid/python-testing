#include <Servo.h>
Servo panServo;
Servo tiltServo;
String inByte;
int pos;

/*
   The code receives serial data and adjusts the servo positions. 
   A command to change position comes in parts:
    - First byte: 'P' or 'T' (indicates pan or tilt)
    - Remaining bytes: Interpreted as ASCII numbers, converted to desired angle
*/

// Connect servos
void setup() {
  panServo.attach(9);     // Pan servo on pin 9
  tiltServo.attach(10);   // Tilt servo on pin 10
  Serial.begin(115200);     // Baud rate 9600
}


void loop()
{
  int val =  0;           /* byte value read from serial */

  if (Serial.available() >0) {
    // Read first byte (indicates which servo to adjust)
    val = Serial.read();

    // Adjust position if command 'P' or 'T'
    if (val=='P' || val=='T') 
    {
      inByte = Serial.readStringUntil('\n'); // read data until newline
      pos = inByte.toInt();   // change datatype from string to integer
      
      if(val=='P') 
      {
        panServo.write(pos);
        Serial.print("Pan:  ");
        Serial.println(inByte);
      } // end if(val=='P') 
      
      else 
      {
        tiltServo.write(pos);
        Serial.print("Tilt: ");
        Serial.println(inByte);
      }
       
    }     
  }  
} // end loop
