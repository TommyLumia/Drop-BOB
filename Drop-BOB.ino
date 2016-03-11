/* This sketch is created for use of a cold drip coffeemaker with a THING DEV from Sparkfun using the ESP8266 wifi chip. 
Copyright: Bobby Lumia, may be used and modified for personal use only. No resale.
Please reference my project blog: http://bobbobblogs.blogspot.ca/ if useing this sketch in your work.
*/

#include <stdlib.h> // Include Standard Library
#include <Servo.h> // for the servo
#include <SimpleTimer.h>

#include <Wire.h>

// for the ESP wifi - Blynk App =====\/
//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <SPI.h>
#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp8266.h> 
const char* ssid     = "ASUS-BOB-BOB-Jelly-Jube";
const char* password = "fuck off get your own";
char auth[] = "58620a59ec64485aa3d5bfd00edae573"; //=====/\

SimpleTimer timer;

//============================= \/ DEFINE PINS \/ ==============

int ServoPIN = 2; // attaches the servo on pin 2 to the servo object ==================== PIN 2
int photo_interuptor_PIN = 14; // attaches the photo-interuptor to the 14 pin ============ PIN 14
int LED_PIN = 5; // attaches the LED to pin 5 (may be other pins on other boards ========== PIN 5

//================================\/ PID \/=============

unsigned long lastTime;
double errSum, lastErr, error, dErr;
double kp = 1.5, ki = 0.4/60000.0, kd = 0; //kp tunes to 1.3 alone

//======================================================

Servo myservo;  // create servo object to control a servo 
int servo_time_to_detatch = 20; // time in (ms) to let the servo force to the correct position

uint32_t count = 0;
uint32_t delta = 0;
int state = HIGH; // HIGH is high signal, not broken light beam.
uint32_t lastDrop = millis();
float DPM = 0;
int first_drop = 1; // Avoid the first drop in servo settings
int creep = 0;
int voltage = 0;
int button = 0;
int restart = 0;
int pause = 0;
long slide_time = 0;
bool led_gp5;
float A = 0;
float B = 0;
char LCD[15] = "              ";

// SETUP VALUES ======================================================
double set_DPM = 6.0;
float Servo_Val = 140; // starting servo value (0 is full open 180 is full close)
float servo_min = 15;
float servo_max = 195;

// RUNNING AVERAGE VARIABLES =========================================
const int numReadings = 5;
double readings[numReadings];      // the readings from the analog input
double total = 0;                  // the running total
double average = 0;                // the average
int index_n = 0;                  // the index_n of the current reading
int start_avg = 0;
//====================================================================

//=========================================================================BLYNK functions & Widgets=====



void Servo_Value_update(){ //V0 is Servo position 0 to 180 being reported back to Blynk app - OUTPUT
  Blynk.virtualWrite(V0, Servo_Val);
}

BLYNK_WRITE(V1){ //V1 pushbutton in Blynk app that simulates a drop of water (for testing) - INPUT
  if (param.asInt() == 1){
    voltage = 5.0;
    button = 1;
  }
}

void setpoint_DPM_update(){ //V2 is "setDPM" - actual value being reported back to Blynk app - OUTPUT
  Blynk.virtualWrite(V2, set_DPM);
}

BLYNK_WRITE(V3){ //V3 is "Blynk slider for DPM setting" - INPUT
  pause = 1;
  slide_time = millis();
  set_DPM = param[0].asInt();
}

void count_drops_update(){ //V4 is "actual number of drops" being reported back to Blynk app - OUTPUT
  Blynk.virtualWrite(V4, count);
}

void sendUptime(){ //V5 is "uptime" being reported back to Blynk app - OUTPUT
  Blynk.virtualWrite(V5, millis() / 1000);
}

BLYNK_WRITE(V6){ //V6 is "Blynk restart button" - INPUT
  restart = param.asInt();
}

void actual_DPM_update(){//V7 is current actual "DPM" - OUTPUT
  Blynk.virtualWrite(V7, DPM);
}

BLYNK_WRITE(V8){ //V8 is "Blynk Pause button" - INPUT
  pause = param.asInt();
}

BLYNK_WRITE(V9){ //V9 is "Blynk Manual servo control" - INPUT
  pause = 1;
  slide_time = millis();
  Servo_Val = param.asInt();
}

void actual_DPM_history(){//V10 is current actual "DPM - HISTORY" - OUTPUT
  Blynk.virtualWrite(V10, DPM);
}

void open_up(){
  if ((millis()-lastDrop) > 1.05*(60000.0/set_DPM)){ //----------------------------medium loop 2
      char LCD[15] = "[ open servo ]"; //15 char only
      uint32_t temp_delta = millis() - lastDrop;
      float temp_DPM = 60000.0/temp_delta;
      
      Servo_Val = Servo_Val - (set_DPM - temp_DPM); // if the servo closed and no drops are comming for too long open it up a little.
      
      if (Servo_Val < servo_min)
        Servo_Val = servo_min;
        
      //myservo.attach(ServoPIN);  // attaches the servo on pin 2 to the servo object ==================== 2
      //delay(15);
      myservo.write(Servo_Val);
      //delay(servo_time_to_detatch);
      //myservo.detach();
                
      creep = 0;
  
      Serial.print("   ");Serial.print("\t");Serial.print("ms");
      Serial.print("\t");
      Serial.print(count);
      Serial.print("\t");
      Serial.print("~");Serial.print(temp_DPM);Serial.print("\t");Serial.print("DPM");
      Serial.print("\t");
      Serial.print(set_DPM);Serial.print("\t");Serial.print("Setpoint");
      Serial.print("\t");
      Serial.print(Servo_Val);Serial.print("\t");Serial.print("Servo_val");//*
      Serial.print("\t");
      Serial.print("   ");Serial.print("\t");Serial.print("error");
      Serial.print("\t");
      Serial.print("   ");Serial.print("\t");Serial.print("errSum");
      Serial.print("\t");
      Serial.print("   ");Serial.print("\t");Serial.println("dErr");//*/
    }//----------------------------------------------------------------------------medium loop 2
  }

//=====================================================================START SETUP===========================

void setup()
{    
  char LCD[15] = "...setting up."; //15 char only
  
  Serial.begin(9600);
  delay(10);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  myservo.attach(ServoPIN);  // attaches the servo on pin A0 to the servo object ==================== A0
  delay(15);
  myservo.write(Servo_Val);
  //delay(servo_time_to_detatch);
  //myservo.detach();
  
  Blynk.begin(auth, "ASUS-BOB-BOB-Jelly-Jube", "fuck off get your own");
  
  Serial.println("Dropcounter 0.1");
  Serial.println();
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; // Initialize the array
    
  pinMode(photo_interuptor_PIN, INPUT);
  pinMode(ServoPIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  while (!Blynk.connect()) {/*Wait until connected*/}

  // Blink the LED pin during setup (for fun)
  digitalWrite(LED_PIN, LOW);  led_gp5 = LOW;
  delay(500);
  digitalWrite(LED_PIN, HIGH); led_gp5 = HIGH;
  delay(500);
  digitalWrite(LED_PIN, LOW); led_gp5 = LOW;
  delay(500);
  digitalWrite(LED_PIN, HIGH); led_gp5 = HIGH;
  delay(500);

  timer.setInterval(500L, sendUptime); 
  timer.setInterval(300L, Servo_Value_update); 
  timer.setInterval(1500L, count_drops_update);
  timer.setInterval(1500L, actual_DPM_update);
  timer.setInterval(1500L, actual_DPM_history);
  timer.setInterval(2000L, setpoint_DPM_update);
  
  Blynk.virtualWrite(V4, count);
  Blynk.virtualWrite(V7, DPM);
  Blynk.virtualWrite(V2, set_DPM);
  Blynk.virtualWrite(V0, Servo_Val);

  Blynk.tweet("Brewing a fresh pot of Cold Drip Coffee with my Drop-BOB v1.0: Check it out at www.bobbobblogs.blogspot.com");
  
  timer.setInterval(2000L, open_up); // open up the servo every 2 seconds that no drops come ... not a Blynk update
}//================================================================================END SETUP========================

void loop(){
  char LCD[15] = "counting drops"; //15 char only

  while(pause == 1){
     char LCD[15] = "..PAUSED!!!!.."; //15 char only
     Blynk.run(); 
     int lapse = millis() - slide_time;
     if ( lapse > 1000 && slide_time > 0){
        pause = 0;
        slide_time = 0;
     }
  }
 
 
  Blynk.run(); //Constant Blynk connection
  timer.run(); // Initiates SimpleTimer

  digitalWrite(LED_PIN, HIGH); // for some odd reason ... LED PIN to "HIGH" means "off"
    
  int raw = analogRead(photo_interuptor_PIN); // read the drop sensor
  if(button == 0){
    voltage = 5.0 * raw / 1023; // convert it to voltage
  }
  
  if (voltage > 1.0 && state == HIGH) {//-----------------------------------------------------------------------big loop 1
    state = LOW;
    button = 0;

    digitalWrite(LED_PIN, LOW); // for some odd reason ... LED PIN to "LOW" means "on"
    
    count = count + 1;   // or count++;
    delta = millis() - lastDrop; //get the difference in time between each drop
    lastDrop = millis(); // remember time of last drop to prevent bouncing.
    DPM = 60000.0 / delta; // get drop per min
    
    total= total - readings[index_n];  // subtract the last reading:       
    readings[index_n] = DPM; 
    total= total + readings[index_n];   // add the reading to the total:    
    index_n = index_n + 1;                    
    
    if (index_n >= numReadings) {  
      start_avg = 1;      
      index_n = 0;}      
    if (start_avg == 1) {
      DPM = total / numReadings;}   // drop the instantaneou DPM and use the running ave after all initial readings taken
    
    /*Compute all the working error variables*/
    error = set_DPM - DPM;
    errSum += (error * delta);
    dErr = (error - lastErr) / delta;    
   
    if ( DPM < set_DPM * 2.0 && first_drop == 0) { ////THIS GIVES SEPERATE CONTROL OF DPM GREATER THAN SETPOINT (default 6)
      Servo_Val = Servo_Val - kp * error - ki * errSum - kd * dErr; // Set servo change depending on how far away from set_DPM you are at
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;
      
      //myservo.attach(ServoPIN);  // attaches the servo on pin 2 to the servo object ==================== 2
      //delay(15);
      myservo.write(Servo_Val);
      //delay(servo_time_to_detatch);
      //myservo.detach();
    }
    else if ( DPM > set_DPM * 2.0 && first_drop == 0) { ////THIS GIVES SEPERATE CONTROL OF DPM GREATER THAN SETPOINT BY ALOT!!!! (default 6)
      Servo_Val = Servo_Val - kp*0.4 * error - ki * errSum - kd * dErr; // Set servo change depending on how far away from set_DPM you are at
      if (Servo_Val < servo_min) Servo_Val = servo_min;
      if (Servo_Val > servo_max) Servo_Val = servo_max;
      
      //myservo.attach(ServoPIN);  // attaches the servo on pin 2 to the servo object ==================== 2
      //delay(15);
      myservo.write(Servo_Val);
      //delay(servo_time_to_detatch);
      //myservo.detach();
    }
    
    Serial.print(delta);Serial.print("\t");Serial.print("ms");
    Serial.print("\t");
    Serial.print(count);
    Serial.print("\t");
    Serial.print(DPM);Serial.print("\t");Serial.print("DPM");
    Serial.print("\t");
    Serial.print(set_DPM);Serial.print("\t");Serial.print("Setpoint");
    Serial.print("\t");
    Serial.print(Servo_Val);Serial.print("\t");Serial.print("Servo_val");//*
    Serial.print("\t");
    Serial.print(kp * error);Serial.print("\t");Serial.print("error");
    Serial.print("\t");
    Serial.print(ki * errSum);Serial.print("\t");Serial.print("errSum");
    Serial.print("\t");
    Serial.print(kd * dErr);Serial.print("\t");Serial.println("dErr");//*/
     
    first_drop = 0;
  }//----------------------------------------------------------------------------------------------------big loop 1

  if (voltage < 1.0 && state == LOW ){  // end of pulse, now we may expect a new one, 
    if (millis() - lastDrop > 100) state = HIGH; // only go back to state high if some time has passed.
  }

  lastErr = error;
  
  if( (millis()-lastDrop) > 60000 && Servo_Val < (servo_min + 2)){
    Serial.println();Serial.println("FINISHED!!!");
    char LCD[15] = "FINISHED!!!!!!"; //15 char only
    Blynk.tweet("Brew DONE!!: www.bobbobblogs.blogspot.com");
    while(restart == 0) {Blynk.run();} // when finished do nothing but listen for Blynk app
    restart = 0;
    lastDrop = millis();
    Servo_Val = 140;
    Serial.println();Serial.print("Restarting");delay(700);Serial.print(" .");delay(700);Serial.print(".");delay(700);Serial.print(".");delay(10);Serial.println(" waiting for drops");Serial.println();
  }
}
