/*
 * pwm_enc reading
 eeprom variables
 id
max
min
offset
start
invert
out limit
mag_enc
kp
ki
kd
  20.4487804878 ticks per mm

 */

#define BAUDRATE     115200

#include "Arduino.h"
#include <Wire.h>
//#include <AS5045.h>
#include <PID_v1.h>
#include <EEPROM.h>

 // user data to be put in eeprom
int MinVal;// min angle in degrees
int MaxVal;//max angle in degrees
int invert;
boolean mag_enc = false;
int out_limit;
int offset;// correction to read zero at 180 degrees
int my_id;
int start_pos;

int mod_angle;
double Kp = 10;
double Ki = 5;
double Kd = 0;
int Ko = 50;//loop pid time



/* Serial port baud rate */
// CHANGE THESE AS APPROPRIATE magnetic encoder
//#define CSpin   5//11// pins are reverser for some reason
//#define CLKpin  6//10// pins are reverser for some reason
//#define DOpin   4//12

//AS5045 enc (CLKpin, CSpin, DOpin) ;
long ticks;
long Prevticks;
int distance;
String EncString;
long cmds;
#define enc_pin 6


/* Include definition of serial commands */
#include "commands.h"



int tmp;
long angle =0;
byte a1 = 0;
byte b1 = 0;
byte a2 = 0;
byte b2 = 0;
volatile byte buf [32];


#define SERVO_DIRECTION_PIN_1 8 //Direction pin servo motor
#define SERVO_DIRECTION_PIN_2 7 //9Direction pin servo motor
#define SERVO_PWM_PIN 9       //PWM pin servo motor changed from 7 to pwm pin
#define SERVO_SENSOR A0       //Analog input servo sensor 
int period = 100; //10 hz
unsigned long time_now = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,0,0, DIRECT);
/* Variable initialization */
int right_power = 0;
int moving =0;



// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index1 = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;



/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index1 = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[3];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case POSITION://a
    // Reset the auto stop timer 
    //lastMotorCommand = millis();
    //if (arg1 == 0 ) {
      //Setpoint = 1800;
      //Prevticks;
      //moving = 0;
    //}
    //else moving = 1;

    Setpoint = arg1;
    Serial.println(arg1);
    Serial.println("OK");
    
    break;
  case ANGLE://j  
    Serial.print(ticks);
    Serial.print("  ");
    //Serial.print(framerate);
    //Serial.print("  ");
    //Serial.print(Input);
    //Serial.print("  ");
    Serial.println("OK");
    
    break;

 
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
   
    myPID.SetTunings(pid_args[0], pid_args[1],pid_args[2]);
    Serial.println("OK");
    break;
    
    case LOOK_PID://l
    Kp = (EEPROM.read(14)* 256) + EEPROM.read(13);
    Ki = (EEPROM.read(16)* 256) + EEPROM.read(15);
    Kd = (EEPROM.read(18)* 256) + EEPROM.read(17);
      Serial.println(Kp);
      Serial.println(Ki);
      Serial.println(Kd);
    
    break;

    case DIAGNOSTICS://l
      Serial.println(" ");
      Serial.println(" ");
      Serial.print("angle   ");
      Serial.println(angle);
      Serial.print("motor power  ");
      Serial.println(Output);
      Serial.print("setpoint  ");
      Serial.println(Setpoint);
      Serial.print("corrected angle angle  ");
      mod_angle = angle + offset;
      Serial.println(mod_angle);
      Serial.print("out put limit  ");
      Serial.println(out_limit);
     
    
    break;

    case LOOK_ID://m
      //MinVal = (EEPROM.read(4)* 256) + EEPROM.read(3);
      //MaxVal = (EEPROM.read(2)* 256) + EEPROM.read(1);
     // offset = (EEPROM.read(6)* 256) + EEPROM.read(5);
     // start_pos = (EEPROM.read(8)* 256) + EEPROM.read(7);
      //invert = (EEPROM.read(10)* 256) + EEPROM.read(9);
      //out_limit = EEPROM.read(11);
      //mag_enc = EEPROM.read(12);
      //start_pos
      Serial.print("Device id is       ");
      Serial.print(my_id);
      Serial.println(" ");
      Serial.print("Min rotation  is   ");
      Serial.print(MinVal);
      Serial.println(" ");
      Serial.print("Max rotation  is   ");
      Serial.print(MaxVal);
      Serial.println(" ");
      Serial.print("offset  is         ");
      Serial.print(offset);
      Serial.println(" ");
      Serial.print("start position is  ");
      Serial.print(start_pos);
      Serial.println(" ");
      Serial.print("invert motor direction  ");
      Serial.print(invert);
      Serial.println(" ");
      Serial.print("output limit is  ");
      Serial.print(out_limit);
      Serial.println(" ");
      Serial.print("Magnetic encoder invert  ");
      Serial.print(mag_enc);
      Serial.println(" ");
      
    
    break;
    case WRITE_ID://l
      Serial.println(arg1);
      if( EEPROM.read(0) != arg1 ){
      EEPROM.write(0, arg1);} 
      my_id = EEPROM.read(0);
    break;

    case WRITE_MAX://0
      a1 = lowByte(arg1);// mapped_Right_Lift change to *10
      b1 = highByte(arg1);
    
      Serial.println(arg1);
      if( EEPROM.read(1) != a1 ){
      EEPROM.write(1, a1);} 
      if( EEPROM.read(2) != b1 ){
      EEPROM.write(2, b1);} 
      //buf [3] * 256 + buf [2];
    break;

    case WRITE_MIN://0
      a1 = lowByte(arg1);// mapped_Right_Lift change to *10
      b1 = highByte(arg1);
    
      Serial.println(arg1);
      if( EEPROM.read(3) != a1 ){
      EEPROM.write(3, a1);} 
      if( EEPROM.read(4) != b1 ){
      EEPROM.write(4, b1);} 
      //buf [3] * 256 + buf [2];
    break;

    case WRITE_OFF://p
      Serial.println(arg1);
      a1 = lowByte(arg1);// mapped_Right_Lift change to *10
      b1 = highByte(arg1);
    
      Serial.println(arg1);
      if( EEPROM.read(5) != a1 ){
      EEPROM.write(5, a1);} 
      if( EEPROM.read(6) != b1 ){
      EEPROM.write(6, b1);} 
    break;

    case WRITE_START://s
      Serial.println(arg1);
      a1 = lowByte(arg1);// mapped_Right_Lift change to *10
      b1 = highByte(arg1);
    
      Serial.println(arg1);
      if( EEPROM.read(7) != a1 ){
      EEPROM.write(7, a1);} 
      if( EEPROM.read(8) != b1 ){
      EEPROM.write(8, b1);} 
    break;

    case WRITE_INVERT://i
      Serial.println(arg1);
      a1 = lowByte(arg1);// mapped_Right_Lift change to *10
      b1 = highByte(arg1);
    
      Serial.println(arg1);
      if( EEPROM.read(9) != a1 ){
      EEPROM.write(9, a1);} 
      if( EEPROM.read(10) != b1 ){
      EEPROM.write(10, b1);} 
    break;

    case WRITE_OUTLIMIT://r
      Serial.println(arg1);
      EEPROM.write(11, arg1);
      out_limit = EEPROM.read(11);
    break;

    case WRITE_PID://r
      Serial.println("PID");
      Serial.println(Kp);
      Serial.println(Ki);
      Serial.println(Kd);
      
    break;

    case WRITE_ENC://l
      Serial.println(arg1);
      EEPROM.write(11, arg1);
      mag_enc = EEPROM.read(12);
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  my_id = EEPROM.read(0);
  Wire.begin(my_id);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  TCCR1B = TCCR1B & 0b11111000 | 0x01 ;// for pin 9
  pinMode(SERVO_DIRECTION_PIN_1, OUTPUT);  
  pinMode(SERVO_DIRECTION_PIN_2, OUTPUT);   
  pinMode(SERVO_PWM_PIN, OUTPUT);     
  myPID.SetMode(AUTOMATIC);
  //myPID.SetSampleTime(Ko);
  pinMode(SERVO_SENSOR, INPUT); 
  pinMode(enc_pin, INPUT_PULLUP);    
 
  start_pos = (EEPROM.read(8)* 256) + EEPROM.read(7);
  Setpoint = start_pos; // start pos
  
  MinVal = (EEPROM.read(4)* 256) + EEPROM.read(3);
  MaxVal = (EEPROM.read(2)* 256) + EEPROM.read(1);
  offset = (EEPROM.read(6)* 256) + EEPROM.read(5);
  out_limit = EEPROM.read(11);
  mag_enc = EEPROM.read(12);
  invert = (EEPROM.read(10)* 256) + EEPROM.read(9);
  pinMode(LED_BUILTIN, OUTPUT);
  
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  if(millis() > time_now + period){
        float deltaT = (millis() - time_now)/10;
        time_now = millis(); 
        int rev = pulseIn(enc_pin, HIGH); //map(enc.read (), 0, 4095 , 4095, 0);// change this to invert function
        angle = map(rev, 0, 4090, 0, 360);
        Input = angle;
  b1 = lowByte(angle);// mapped_Right_Lift change to *10
  a1 = highByte(angle);
  //Setpoint = ((buf[2] * 256) + buf[1]);
  //double w = ((buf[2]*256) +buf[1]) *0.1;
  myPID.Compute();
  myPID.SetOutputLimits(-250 , 250);
  right_power = Output;
  //stepper.move(Output * 16);
  Serial.print(my_id);
  Serial.print("   ");
 Serial.print(Setpoint);
  Serial.print("   ");
 Serial.print(Input);
  Serial.print("   ");
  Serial.println(Output);
  //Serial.print("   ");
  //Serial.println(rev);
  
  
  
  motor();
time_now = millis();
  }

  //Ser_print();
 // Serial.write(mapped_Right_Lift);
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index1] = NULL;
      else if (arg == 2) argv2[index1] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index1] = NULL;
        arg = 2;
        index1 = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index1] = chr;
        index1++;
      }
      else if (arg == 2) {
        argv2[index1] = chr;
        index1++;
      }
    }
  }
  
    // if (millis() > nextPID) {
    //updatePID();
    //nextPID += PID_INTERVAL;
  //}

  //if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
   // right_power = 0;
   // Setpoint = 0;
   // moving = 0;
  //}
}

void updatePID() {
  // change to velocity
  ticks = 0;
  distance = ticks *0.0489026718;
  EncString = String(ticks);
  Input = ticks - Prevticks;
  myPID.Compute();
  myPID.SetOutputLimits(-255 ,255);
  right_power = Output;
  //Serial.print(distance);
  //Serial.print("  ");
  //Serial.println(ticks);
  Prevticks = ticks;
}


void motor(){
  //Output = Output * -1;
  if (right_power > 1) {
    digitalWrite(SERVO_DIRECTION_PIN_1, HIGH);
    digitalWrite(SERVO_DIRECTION_PIN_2, LOW);
    analogWrite(SERVO_PWM_PIN, abs(right_power));
  }

  if (right_power < -1) {
    digitalWrite(SERVO_DIRECTION_PIN_1, LOW);
    digitalWrite(SERVO_DIRECTION_PIN_2, HIGH);
    analogWrite(SERVO_PWM_PIN, abs(right_power));
  }

  if (right_power == 0) {
    digitalWrite(SERVO_DIRECTION_PIN_1, LOW);
    digitalWrite(SERVO_DIRECTION_PIN_2, LOW);
    analogWrite(SERVO_PWM_PIN, abs(right_power));
  }
}

void requestEvent() {
 // Wire.write(8);
  Wire.write(a1); // respond with message of 6 bytes
  Wire.write(b1);
 
  // as expected by master
}

void receiveEvent(int bytes)
{
  if(Wire.available() != 0)
  {
    for(int i = 0; i< bytes; i++)
    {
      buf [i]= Wire.read();
      int x = ((buf[1]*256) +buf[0]);
      //Serial.print("Received: ");
        Serial.print(x);
     // Serial.print("\n");
      Setpoint = x;
    }
  }
}

///feedback
int getFeedback(int a){
int j;
int mean;
int result;
int test;
int reading[20];
boolean done;

for (j=0; j<20; j++){
reading[j] = analogRead(a); //get raw data from servo potentiometer
delay(3);
} // sort the readings low to high in array
done = false; // clear sorting flag
while(done != true){ // simple swap sort, sorts numbers from lowest to highest
done = true;
for (j=0; j<20; j++){
if (reading[j] > reading[j + 1]){ // sorting numbers here
test = reading[j + 1];
reading [j+1] = reading[j] ;
reading[j] = test;
done = false;
}
}
}
mean = 0;
for (int k=6; k<14; k++){ //discard the 6 highest and 6 lowest readings
mean += reading[k];
}
result = mean/8; //average useful readings
return(result);
}    // END GET FEEDBACK
