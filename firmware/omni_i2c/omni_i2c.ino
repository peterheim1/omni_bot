
#include <Wire.h>
#include <I2C_Anything.h>
#include <Messenger.h>
// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();
//steering variables
int fr_s, fl_s,rl_s,rr_s;
//drive variables
int fr_d, fl_d,rl_d,rr_d;
byte a1 = 0;
byte b1 = 0;
int period = 100; //10 hz
unsigned long time_now = 0;

// request position from slave
int RequestData(int address)
{
  int val;
  Wire.requestFrom(address, 2);    // request 2 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
  val = Wire.read ();
  val <<= 8;
  val |= Wire.read ();
  int result = val;
  return result;
  }
}


void setup() 
{
  Wire.begin ();
  Serial.begin (115200);
  _Messenger.attach(OnMssageCompleted);
 // Wire.onReceive (receiveEvent);
  
} 


void loop() 
{
  ReadSerial();
  //Timing loop
  if(millis() > time_now + period){
  fr_s = RequestData(9);
  fl_s = RequestData(8);
  rr_s = RequestData(5);
  rl_s = RequestData(2);
 time_now = millis();
  } 
Ser_print();
}

 void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
}

// Define messenger function
void OnMssageCompleted()
{

  if (_Messenger.checkString("s"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   int b = _Messenger.readInt();
   int c = _Messenger.readInt();
   int d = _Messenger.readInt();
   SetTarget(9, a);
   SetTarget(8, b);
   SetTarget(5, c);
   SetTarget(2, d);
    return;

  }

  if (_Messenger.checkString("d"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   int b = _Messenger.readInt();
   int c = _Messenger.readInt();
   int d = _Messenger.readInt();
   SetTarget(7, a);// fl
   SetTarget(3, b);// fr
   SetTarget(4, c);// rl
   SetTarget(6, d);// rr
    return;

  }

  if (_Messenger.checkString("v"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   int b = _Messenger.readInt();
   int c = _Messenger.readInt();
   int d = _Messenger.readInt();
   int e = _Messenger.readInt();
   int f = _Messenger.readInt();
   int g = _Messenger.readInt();
   int h = _Messenger.readInt();
   SetTarget(8, a);// fl
   SetTarget(9, b);// fr
   SetTarget(2, c);// rl
   SetTarget(5, d);// rr
   SetTarget(7, e);// fl
   SetTarget(3, f);// fr
   SetTarget(4, g);// rl
   SetTarget(6, h);// rr
    return;

  }

  if (_Messenger.checkString("a"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   
   SetTarget(9, a);
   
    return;

  }

  if (_Messenger.checkString("b"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   
   SetTarget(8, a);
   
    return;

  }

  if (_Messenger.checkString("c"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   
   SetTarget(5, a);
   
    return;

  }

  if (_Messenger.checkString("d"))
  {
    //Set wheel angle
   int a = _Messenger.readInt();
   
   SetTarget(2, a);
   
    return;

  }
}




// set servo target on slave servo
void SetTarget(int address, long target)
{
    //double foo = target;
    b1 = lowByte(target);
    a1 = highByte(target);
    
    Wire.beginTransmission (address);
    Wire.write(b1); // respond with message of 2 bytes
    Wire.write(a1);
    Wire.endTransmission ();


       
}
