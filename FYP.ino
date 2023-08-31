#include<Servo.h>
#define numOfValsRec 7
#define digitsPerValRec 3
int valsRec[numOfValsRec];
int stringlength = numOfValsRec*digitsPerValRec;
Servo Thumb;
Servo Index;
Servo Middle;
Servo Ring;
Servo Pinky;
Servo Elbow;
Servo Shoulder;
String Received;
void setup() 
{
  Serial.begin(115200);
  Thumb.attach(3);
  Index.attach(4);
  Middle.attach(5);
  Ring.attach(6);
  Pinky.attach(7);
  Elbow.attach(8);
  Shoulder.attach(9);
  Zero();
}
void Zero()
{
  Thumb.write(0);
  Index.write(0);
  Middle.write(0);
  Ring.write(0);
  Pinky.write(0); 
  Elbow.write(0);
  Shoulder.write(0); 
}
void Receiving()
{
  if(Serial.available())
  {
    Received=Serial.readString();
    for(int i=0;i<numOfValsRec;i++)
      {
        int num =(i*digitsPerValRec);
        valsRec[i]= Received.substring(num,num+digitsPerValRec).toInt();         
      }  
  }

 Servo_Control();
 Received="";
}

void Servo_Control()
{
Thumb.write(valsRec[0]);
Index.write(valsRec[1]);
Middle.write(valsRec[2]);
Ring.write(valsRec[3]);
Pinky.write(valsRec[4]);
Pinky.write(valsRec[5]);
Pinky.write(valsRec[6]);  
}

void loop() 
{
Receiving();
}

