
#include <Wire.h>
#include <I2C_Anything.h>
#include <Messenger.h>
#include <NewPing.h>

#define TRIGGER_PIN  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();
//steering variables
int fr_s, fl_s,rl_s,rr_s;
//drive variables
int fr_d, fl_d,rl_d,rr_d;
//float rear_dist = 0;
//int ir_l, ir_r, rear_bump;
int x = 0;
int y = 0;
int z = 0;
int ir_r, ir_l;
int RevD = 1;
bool dock = 0;
 int ir_state = 0;
bool left_ir_state, right_ir_state;
double rear_dist;
int battery;

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
 pinMode(2, INPUT);
 pinMode(6, INPUT);
 pinMode(8, INPUT_PULLUP);
  
} 


void loop() 
{
  ReadSerial();
  //Timing loop
  if(millis() > time_now + period){
   rear_dist =  sonar.ping_cm();
   battery = analogRead(A0);
   ir_r = digitalRead(6);
   ir_l = digitalRead(2);
   //rear_bump = digitalRead(8);
   if (dock > 0 ){AutoDock();}
   fr_s = RequestData(9);
   fl_s = RequestData(8);
   rr_s = RequestData(5);
   rl_s = RequestData(2);
   Ser_print();
  time_now = millis();
  } 

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

  if (_Messenger.checkString("s"))
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
    dock = 1;
    Serial.println("going to auto dock");
    return;

  
  }
//}

}

void GoLeft(){
  // angle
   SetTarget(8, 170);// fl
   SetTarget(9, 170);// fr
   SetTarget(2, 170);// rl
   SetTarget(5, 170);// rr
   // speed
   SetTarget(7, -80);// fl
   SetTarget(3, -80);// fr
   SetTarget(4, -80);// rl
   SetTarget(6, -80);// rr

}

void GoRight(){
  // angle
   SetTarget(8, 190);// fl
   SetTarget(9, 190);// fr
   SetTarget(2, 190);// rl
   SetTarget(5, 190);// rr
   // speed
   SetTarget(7, -80);// fl
   SetTarget(3, -80);// fr
   SetTarget(4, -80);// rl
   SetTarget(6, -80);// rr

}

void GoBack(){
  // angle
   SetTarget(8, 180);// fl
   SetTarget(9, 180);// fr
   SetTarget(2, 180);// rl
   SetTarget(5, 180);// rr
   // speed
   SetTarget(7, -80);// fl
   SetTarget(3, -80);// fr
   SetTarget(4, -80);// rl
   SetTarget(6, -80);// rr

}

void GoStop(){
  // angle
   SetTarget(8, 180);// fl
   SetTarget(9, 180);// fr
   SetTarget(2, 180);// rl
   SetTarget(5, 180);// rr
   // speed
   SetTarget(7, 0);// fl
   SetTarget(3, 0);// fr
   SetTarget(4, 0);// rl
   SetTarget(6, 0);// rr

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

void AutoDock(){
  
  Serial.println("we are docking ");
  while (dock == 1){
    dock = digitalRead(8);
    left_ir_state = digitalRead(2);
    right_ir_state = digitalRead(6);
    rear_dist =  sonar.ping_cm();
    //ir_state = left_ir_state + right_ir_state;
    if (left_ir_state == 0 && right_ir_state == 1){Serial.println("go left"); GoLeft();}
    if (right_ir_state == 0 && left_ir_state == 1){Serial.println("go right ");GoRight();}
    if (right_ir_state == 0 && left_ir_state == 0){Serial.println("straight back"); GoBack();}
    if (right_ir_state == 1 && left_ir_state == 1){Serial.println("no signal ");GoStop();}
    if (dock == 0){Serial.println("we have docked ");GoStop();}
   //Serial.println(ir_state);
    delay(10);
  }
  }
