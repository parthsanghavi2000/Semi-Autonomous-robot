/*
   @file Final.ino
   @author Dhruv Parikh (dhruvkp@seas.upenn.edu), Yug Ajmera (yug@xyz), Parth Sanghavi (didntwrite@gmail.com)
   @brief Main File for the GTA2022 Competition offerred in MEAM 5100 Fall 22 at University of Pennsylvania.
          The code has 4 functionalities: Pushing the police car, Beacon Tracking, Wall following and Communication with Staff ESP
   @version 2.1
   @date 2022-12-16

   @copyright Copyright (c) 2022

*/



//-- INCLUDE FILES| --//
#include "html510.h"
#include "sliderinterface.h"
#include <WiFi.h>
#include <esp_now.h>
#include "vive510.h"
#include <WiFiUdp.h>


//-- |VIVE CIRCUIT| --//
#define SIGNALPIN1 25 // pin receiving signal from Vive circuit //25-26
#define SNDVIVE 1
#define teamNumber 22
#define FREQ 1 // in Hz
uint16_t xold1, xold2, yold1, yold2;  //Variables for median filter
uint16_t x, y;                        //Orignal Variables
Vive510 vive1(SIGNALPIN1);            //Vive 1 Object
float realX = 4000;                   //realX is Police Car Location
float realY = 4000;                   //realY is Police Car Location
float gotdata = 0;                    //Got-data flag for Police Car

//UDP details for Police car data
#define UDPPORT 2510 // port for game obj transmission
WiFiUDP UDPServer;
IPAddress myIPaddress(192, 168, 1, 58); // change to your IP
//Police Car data
int TholaX, TholaY;

//PWM setup
#define LEDC_RESOLUTION_BITS 14 // 2^13 = 4096
#define LEDC_RESOLUTION  ((1<<LEDC_RESOLUTION_BITS)-1) //4095
#define LEDC_FREQ_HZ     500

#define RGBLED 18

#define MAC_RECV  {0x84,0xF7,0x03,0xA8,0xBE,0x30} // receiver MAC address (last digit should be even for STA)
float t2 = millis();

// --|HTML|-- //
void handleRoot(void);
void handleSlider(void);
HTML510Server h(80);


// --|Motor Pins|-- //
// Motor 1 pins - naming dictates the pins on driver
#define one_A              22
#define two_A              19
// Motor 2 pins
#define three_A            18
#define four_A             23
// PWM pins
#define pwm1        5
#define pwm2        21
#define pwm1_channel 0
#define pwm2_channel 1

// --|Beacon Tracking|-- //
#define pulse_r 36
#define pulse_l 39

// --|Wall Following|-- //
// Right ultrasonic
#define r_echoPin 37
#define r_trigPin 27

// Forward ultrasonic
#define f_echoPin 34
#define f_trigPin 33


//Wifi Details
const char* ssid     = "TP-Link_05AF";
const char* password = "47543454";




esp_now_peer_info_t staffcomm = {
  .peer_addr = {0x84, 0xF7, 0x03, 0xA9, 0x04, 0x78},
  .channel = 1,             // channel can be 1 to 14, channel 0 means current channel.
  .encrypt = false,
};

void pingstaff() {
  uint8_t teamNum = 5;
  esp_now_send(staffcomm.peer_addr, &teamNum, 1);
}


//Extra Variables
int state = 0;
long duration;
int distance_r;
int distance_f;
bool leftIsSeen;           // phototransistor 1 is seeing the beacon
bool rightIsSeen;          // phototransistor 2 is seeing the beacon
int ontime, offtime, duty;  //For frequency detection
float freq1, period, freq2;
float lastseen = 0;         //Variable for what is seen last
float target = 15.0;        //Wall following variables
float error = 0, preverr = 0; //PD controller
float out, dt, t1 = 0;        //PD Controller
float Kp = 30, Kd = 5;
float speed = 60;           //Feedforward speed

//Median Filter variables
float old1, old2;
float tmp2 = 20;

//Function Defs
//HTML Page Setup
void handleRoot() {
  /*
     Sending the body defined in inteface file
       @module: HTML Webpage
  */
  h.sendhtml(body);
}
void handleStates() {
  // * @module: HTML Webpage
  state = h.getVal(); //Get the value from button GET
}

//Analog Write for motors
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // * @module: General Car Functionality
  uint32_t duty =  LEDC_RESOLUTION * min(value, valueMax) / valueMax;
  ledcWrite(channel, duty);  // write duty to LEDC
}

void fwd() {
  /*
    Calling this command will configure the motors to go in forward mode
    @module: General Car Functionality
  */
  digitalWrite(one_A, HIGH);
  digitalWrite(two_A, LOW);
  digitalWrite(three_A, HIGH);
  digitalWrite(four_A, LOW);
}

void right() {
  /*
    Calling this command will configure the motors to go in turn right mode
    @module: General Car Functionality
  */
  digitalWrite(one_A, LOW);
  digitalWrite(two_A, HIGH);
  digitalWrite(three_A, HIGH);
  digitalWrite(four_A, LOW);
}

void left() {
  /*
    Calling this command will configure the motors to go in turn left mode
    @module: General Car Functionality
  */
  digitalWrite(one_A, HIGH);
  digitalWrite(two_A, LOW);
  digitalWrite(three_A, LOW);
  digitalWrite(four_A, HIGH);
}

void back() {
  /*
    Calling this command will configure the motors to go in reverse mode
    @module: General Car Functionality
  */
  digitalWrite(one_A, LOW);
  digitalWrite(two_A, HIGH);
  digitalWrite(three_A, LOW);
  digitalWrite(four_A, HIGH);
}

void STOP() {
  /*
    Calling this command will configure the motors to go in Stop mode
    @module: General Car Functionality
  */
  digitalWrite(one_A, LOW);
  digitalWrite(two_A, LOW);
  digitalWrite(three_A, LOW);
  digitalWrite(four_A, LOW);
}


void get_dist_r() {
  /*
    Gets the ultrasonic reading from sensor mounted on right hand side
    @module: Wall Following and Police Car
    @update: distance_r (float) storing the readings in cm
  */

  //Stop the pulse - resetting
  digitalWrite(r_trigPin, LOW);
  delayMicroseconds(2);

  //Start the pulse
  digitalWrite(r_trigPin, HIGH);
  //Wait for some time
  delayMicroseconds(10);
  //Stop the pulse
  digitalWrite(r_trigPin, LOW);
  //Measure the time to get it back
  duration = pulseIn(r_echoPin, HIGH);
  //Calculate distance using Speed of sound
  distance_r = duration * 0.017;
}

void get_dist_f() {
  /*
    Gets the ultrasonic reading from sensor mounted on forward side
    @module: Wall Following and Police Car
    @update: distance_f (float) storing the readings in cm
  */
  digitalWrite(f_trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(f_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(f_trigPin, LOW);

  duration = pulseIn(f_echoPin, HIGH);

  distance_f = distance_f * 0.8 + 0.2 * (duration * 0.017);

}

void read_right_ir() {
  /*
    Gets the reading from right phototransistor
    @module: Beacon Tracking
    @update: freq1 (float) storing the frequency in hz
           : rightIsSeen (bool) storing if right side was seen first or left side
  */

  //Measure the HIGH time for the PWM
  ontime = pulseIn(pulse_r, HIGH);
  //Measure the LOW time for the PWM
  offtime = pulseIn(pulse_r, LOW);
  //Time period = ontime + offtime
  period = ontime + offtime;
  //If period is zero then avoid dividing by zero and simply set freq as not detected
  if (period == 0) {
    rightIsSeen = false;
    freq1 = 0;
    Serial.print(0);
  }

  else {
    //Low pass filter on frequency obtained
    freq1 = 0.8 * freq1 + 0.2 * (1000000.0 / period);
    //If the frequency is between 500-900 Hz, then we have detected 200 Hz
    if (abs(freq1 - 700) < 200) {
      rightIsSeen = true;
      Serial.print(1);
    }
    else {
      rightIsSeen = false;
      Serial.print(0);
    }
  }

}

void read_left_ir() {
  /*
    Gets the reading from left phototransistor
    @module: Beacon Tracking
    @update: freq1 (float) storing the frequency in hz
           : leftIsSeen (bool) storing if left side was seen first or left side
  */

  //Measure the HIGH time for the PWM
  ontime = pulseIn(pulse_l, HIGH);
  //Measure the LOW time for the PWM
  offtime = pulseIn(pulse_l, LOW);
  //Time period = ontime + offtime
  period = ontime + offtime;
  //If period is zero then avoid dividing by zero and simply set freq as not detected
  if (period == 0) {
    leftIsSeen = false;
    Serial.print(0);
    freq2 = 0;
  }
  else {
    //Low pass filter on frequency obtained
    freq2 = 0.8 * freq2 + 0.2 * (1000000.0 / period);
    //If the frequency is between 500-900 Hz, then we have detected 200 Hz
    if (abs(freq2 - 700) < 200) {
      leftIsSeen = true;
      Serial.print(1);
    }
    else {
      leftIsSeen = false;
      Serial.print(0);
    }
  }
}

void readBeaconSensor() {
  /*
    Wrapper to read both the beacon sensors and print
    @module: Beacon Tracking
  */
  read_left_ir();
  Serial.print(",");
  read_right_ir();
  Serial.println("");
}

void beacon_track() {
  /*
    Beacon Tracking main call
    @module: Beacon Tracking
    @runtime: 10 ms
    @NOTE: call readBeaconSensor before calling this function
  */

  //If both sensors see the readings - then go forward
  if (rightIsSeen && leftIsSeen) {
    ledcAnalogWrite(pwm1_channel,  180, 255);
    ledcAnalogWrite(pwm2_channel,  180, 255);
    fwd();
    delay(10);
  }

  //If left sensor sees the reading then turn left and then slightly forward
  else if (leftIsSeen) {
    ledcAnalogWrite(pwm1_channel,  100, 255);
    ledcAnalogWrite(pwm2_channel,  100, 255);
    left();
    delay(5);
    fwd();
    delay(2);
    lastseen = 1;
  }

  //If right sensor sees the reading then turn right and then slightly forward
  else if (rightIsSeen) {
    ledcAnalogWrite(pwm1_channel,  100, 255);
    ledcAnalogWrite(pwm2_channel,  100, 255);
    right();
    lastseen = 0;
    delay(5);
    fwd();
    delay(2);
  }

  //Nothing is seen - rotate in the direction where the latest reading was detected
  else {
    ledcAnalogWrite(pwm1_channel,  100, 255);
    ledcAnalogWrite(pwm2_channel,  100, 255);
    if (lastseen) {
      right();
    }
    else {
      left();
    }

  }

}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  /*
    Function to handle when the UDP package is detected
    @module: Police Car
  */
  Serial.printf(" Recv from: %02x:%02x:%02x:%02x:%02x:%02x ", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(" Data: ");
  if (isascii(data[0])) Serial.println( (char *)data);
  else {
    for (int i = 0; i < data_len; i++ ) {
      Serial.printf("%x", data[i]); if (i % 3 == 0) Serial.print(" ");
    }
  }
}

void wallfollow() {
  /*
    Wall Following main call
    @module: Wall follow
    @runtime: 600 ms worst case
    @update: distance_f, distance_r
    @NOTE: set the target before calling this function
  */

  //Get the front sensor data
  get_dist_f();

  //If we have not encountered a wall in front of us
  if (distance_f >= 15) {

    //Calculate Loop speed
    dt = 0.001 * (millis() - t1);
    t1 = millis();

    //Get right ultrasonic distance
    get_dist_r();

    //Error
    error = (target - distance_r);
    //PD Controller
    out = Kp * error + Kd * (error - preverr) / dt;
    preverr = error;

    //Right and Left motor control
    float right = out + speed;
    float left = speed - out;

    //Saturators
    if (right > 255) {
      right = 255;
    }
    else if (right < 20) {
      right = 20;
    }

    if (left > 255) {
      left = 255;
    }
    else if (left < 20) {
      left = 20;
    }

    //Write to the motors and drive forward
    ledcAnalogWrite(pwm1_channel,  left, 255);
    ledcAnalogWrite(pwm2_channel,  right, 255);
    fwd();


  }
  //If front wall is detected
  else {
    //Go back for 200 ms
    ledcAnalogWrite(pwm1_channel,  255, 255);
    ledcAnalogWrite(pwm2_channel,  255, 255);
    back();
    delay(200);
    //Stop for 100 ms
    STOP();
    delay(100);
    //Go left for 300 ms
    ledcAnalogWrite(pwm1_channel,  90, 255);
    ledcAnalogWrite(pwm2_channel,  90, 255);
    left();
    delay(300);

    //Reset the controller variables
    preverr = 0;
    t1 = 0;
  }

}


float med3Filt(float a, float b, float c) {
  /*
     General 3 window median filter
     @module: General Sensor filtering functionality
  */
  int middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else    middle = (a <= b) ? a : b;
  return middle;
}



void handleUDPServer() {
  /*
    Handles UDP packages that are recieved
    @update: TholaX and TholaY
    @module: Police Car
  */
  const int UDP_PACKET_SIZE = 14; // can be up to 65535          
  uint8_t packetBuffer[UDP_PACKET_SIZE];

  int cb = UDPServer.parsePacket(); // if there is no message cb=0
  if (cb) {
    int x, y;
    packetBuffer[13] = 0; // null terminate string

    UDPServer.read(packetBuffer, UDP_PACKET_SIZE); 
    x = atoi((char *)packetBuffer + 3); // ##,####,#### 2nd indexed char
    y = atoi((char *)packetBuffer + 8); // ##,####,#### 7th indexed char
    TholaX = x;
    TholaY = y;
  }
}




void getrobustPoliceData() {
  /*
     Gets police car data for first time. We need a very clean signal on police car hence we measure it for 10 times when initialising
     @module: Police Car
     @updates: realX and realY
  */
  for (int i = 0; i < 10; i++) {
    realX = 0.8 * realX + 0.2 * TholaX;
    realY = 0.8 * realY + 0.2 * TholaY;
  }
}

void sendVive(void) {
  //Sends the vive message to the staff comms
  //@module: communications
  uint8_t message[13]; // Max ESPnow packet is 250 byte data
  sprintf((char *) message, "%02d:%4d,%4d", teamNumber, x, y);
  esp_now_send(staffcomm.peer_addr, message, sizeof(message));
}

void policeCar() {
  /*
    Push the police car functionality
    @module: Police Car
    Note: Strongly Needs vive updates!
  */

  //Get front sensor data
  get_dist_f();

  //If we are detecting a wall, read again for ten samples and make sure it is a wall
  if (distance_f <= 10) {
    for (int i = 0; i < 10; i++) {
      get_dist_f();
    }
  }

  //If wall is not there
  if (distance_f >= 12) {
    //update dt
    dt = 0.001 * (millis() - t1);
    t1 = millis();
    //Get right sensor distance
    get_dist_r();
    //Apply a median and low pass filter
    float tmp = distance_r;
    distance_r = 0.6 * old1 + 0.4 * (med3Filt(distance_r, old1 , old2 ));
    old2 = old1;
    old1 = tmp;

    //Saturate to avoid noisy readings
    if (distance_r > 60) {
      distance_r = 60;
    }

    //PD Controller with updated Gains
    error = (target - distance_r);
    out = Kp * 1.1 * error + Kd * 0.7 * (error - preverr) / dt;

    //Decrease the output
    out = out * 0.5;
    float right = out + speed;
    float left = speed - out;

    //Saturators
    if (right > 255) {
      right = 255;
    }
    else if (right < 20) {
      right = 20;
    }

    if (left > 255) {
      left = 255;
    }
    else if (left < 20) {
      left = 20;
    }

    //Write to the motors and ram forward
    ledcAnalogWrite(pwm1_channel,  left, 255);
    ledcAnalogWrite(pwm2_channel,  right, 255);
    fwd();
    preverr = error;
  }
  //Vive location based ramming
  else if (abs(x - realX) < 700 && abs(y - realY) < 700)
  {
    //We are getting a consensus on readings that we should hit the car
    ledcAnalogWrite(pwm1_channel,  255, 255);
    ledcAnalogWrite(pwm2_channel,  255, 255);
    //Ram forward
    fwd();
    //For 1 second keep on going forward
    delay(1000);
    //Reset the controller variables
    preverr = 0;
    t1 = 0;
  }
  else {
    //We are probably getting bad data
    STOP();
  }
}



//--------------------------------------------------------------------------//
// MAIN CODE OF GTA2022
//--------------------------------------------------------------------------//

void setup()
{
  //Station+AP mode
  WiFi.mode(WIFI_AP_STA);
  //Setup the wifi
  WiFi.begin(ssid, password);
  WiFi.config( myIPaddress,        // Device IP address
               IPAddress(192, 168, 1, 1),   // gateway (not important for 5100)
               IPAddress(255, 255, 255, 0)); // net mask
  UDPServer.begin(UDPPORT);  // 2510 forgame  arbitrary UDP port# need to use same one
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }

  //HTML STUFF
  ssid = "DhruvP";
  WiFi.softAP(ssid, ""); // no password
  WiFi.config(IPAddress(192, 168, 1, 151),  // local IP
              IPAddress(192, 168, 1, 1),       // Gateway
              IPAddress(255, 255, 255, 0));    // subnet mask
  //ESP NOW
  esp_now_init();
  esp_now_add_peer(&staffcomm);


  //HTML
  h.begin();
  h.attachHandler("/ ", handleRoot); //index page
  h.attachHandler("/mode?val=", handleStates); //handling slider

  //Initialize the frequencies for low pass filter
  freq1 = 700;
  freq2 = 700;

  //---------------------PinModes---------------------//
  //Beacon Tracking
  pinMode(pulse_r, INPUT);
  pinMode(pulse_l, INPUT);
  //Ultrasonic
  pinMode(r_trigPin, OUTPUT);
  pinMode(r_echoPin, INPUT);
  pinMode(f_trigPin, OUTPUT);
  pinMode(f_echoPin, INPUT);
  //Motors
  pinMode(one_A, OUTPUT); pinMode(two_A, OUTPUT); pinMode(three_A, OUTPUT); pinMode(four_A, OUTPUT);
  ledcSetup(pwm1_channel, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(pwm1, pwm1_channel);
  ledcSetup(pwm2_channel, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(pwm2, pwm2_channel);
  //Vive
  pinMode(SIGNALPIN1, INPUT);

  //Callback for espnow
  esp_now_register_recv_cb(OnDataRecv);

  //Serial Port
  Serial.begin(115200);
  //Vive object begin
  vive1.begin();
  Serial.println("  Vive trackers started");
  //Delay for 1 second
  delay(1000);
}

void loop()
{

  h.serve(); //Start the server for HTML
  //Handle UDP for Police Car data
  handleUDPServer();
  //Take Vive reading 2 times
  for (int i = 0; i < 2; i++) {
    if (vive1.status() == VIVE_RECEIVING) {
      //Low pass filter
      float xtemp = x * 0.9 + 0.1 * vive1.xCoord();
      float ytemp = y * 0.9 + 0.1 * vive1.yCoord();
      //If out of range vive data, then don't update!
      if (ytemp > 10000 || ytemp < 200) {
        ytemp = y;
      }
      if (xtemp > 10000 || xtemp < 200) {
        xtemp = x;
      }

      // 3 Window Median Filter
      x = med3Filt(xtemp, xold1 , xold2 );
      y = med3Filt(ytemp, yold1 , yold2 );
      xold2 = xold1;
      xold1 = xtemp;
      yold2 = yold1;
      yold1 = ytemp;
    }

    else {
      //If we are not getting vive data then set it to 0
      x = 0;
      y = 0;
      switch (vive1.sync(5)) {
          break;
        case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
          Serial.println("Weak Signal");
          break;
        default:
        case VIVE_NO_SIGNAL: // nothing detected
          Serial.println("No Signal!");
      }
    }
    delay(20); //If we uncomment this delay it wont work :p

  }

  //Send Vive only if 100 ms passed -> can be set to 1 Hz for competition
  if (SNDVIVE) {
    if (millis() - t2 > 100) {
      t2 = millis();
      sendVive();
    }
  }


  //---------------------------------------//
  // FSM Based Model - states obtained from webpage
  //---------------------------------------//

  //Stale mode
  if (state == 0) {
    STOP();
  }

  //Beacon Tracking
  if (state == 2) {
    readBeaconSensor();
    beacon_track();
  }

  //Wall Following
  if (state == 1) {
    speed = 60;
    wallfollow();
  }

  //Police Car
  if (state == 3) {
    //reset the target and speed
    target = 55;
    speed = 80;
    //Get front sensor data
    for (int i = 0; i < 10; i++) {
      get_dist_f();
    }

    //Vive data
    //Initial measure multiple times to get reliable data
    if (gotdata == 0) {
      getrobustPoliceData(); //PS: not robust
      gotdata = 1;
    }
    //Update Data single shot
    if (TholaX == 0 || TholaY == 0) {
    }
    else {
      realX = 0.7 * realX + 0.3 * TholaX; realY = 0.7 * realY + 0.3 * TholaY;
    }

    //Run Police Car for some time before breaking the loop
    for (int i = 0; i < 100; i++) {
      policeCar();
    }
  }
}
