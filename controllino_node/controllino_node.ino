/*
Copyright 2024 Tiam Dominique González Alvarado

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <Ethernet.h>

//Use controllino library for aliases
#include <Controllino.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string.h>

//Controlino output definitions
/*Pumps*/
#define PUMP_LEFT CONTROLLINO_R0
#define PUMP_RIGHT CONTROLLINO_R1
/*Light*/
#define SIGNAL_GREEN CONTROLLINO_R2
#define SIGNAL_RED_CONTINOUS CONTROLLINO_R3
#define SIGNAL_RED_DIS CONTROLLINO_R4
#define BUZZER CONTROLLINO_R5

//Define Analog Sensor inputs
#define FLOW_SENSOR_LEFT CONTROLLINO_A0
#define FLOW_SENSOR_RIGHT CONTROLLINO_A1
#define DISTANCE_SENSOR CONTROLLINO_A2
#define TEMP_SENSOR CONTROLLINO_A3

void controllino_setup(){
  pinMode(PUMP_LEFT, OUTPUT);
  pinMode(PUMP_RIGHT, OUTPUT);
}

//Server information
byte mac[] = { 0x50, 0xD7, 0x53, 0x00, 0xEE, 0xD4 };
IPAddress ip(192, 168, 0, 10);
IPAddress server(192, 168, 0, 1);
const uint16_t serverPort = 11411;

/*
Create process class with all info needed
*/

//Input

class Input {
public:
  bool start = 0;
  String side = "";
  bool cancel = 0;
};
//ROS Output
class Output {
public:
  std_msgs::String current_state;
  bool pump_left = 0;
  bool pump_right = 0;
  bool green_light = 1;
  bool red_light_continous = 0;
  bool red_light_dis = 0;
  bool buzzer = 0;
  std_msgs::Float32 flow_left;
};
//Controlino Output
//Process class
class Process {
public:
  Input input;
  Output output;
  int current_step = 10;
  float flow_right = 0;
  float flow_left = 0;
  long timer;
  long pump_timeout = 5000; //Zeit in milisekunden
  float water_level = 100; // [cm]
  float water_treshhold = 10; // [cm]
  float water_treshhold_restart = 50;
  float pt_widerstand;
  float temperature;
};

Process process;

bool toggle_variable = 0;
bool step_change = 0;
/*
  ROS variables and funcs
*/
ros::NodeHandle nh;

/*
Subscribers
*/
//Subscribe to cancel
void cancelCb(const std_msgs::Empty& msg) {
  Serial.println("Starting...");
  process.input.cancel = 1;
}
ros::Subscriber<std_msgs::Empty> cancel_message("cancel_process", &cancelCb);

//Subscrube to side
//When message incoming --> Start the process
void sideCb(const std_msgs::String& msg) {
  Serial.print("Starting on the: ");
  Serial.println(msg.data);
  process.input.start = 1;
  process.input.side = msg.data;
}
ros::Subscriber<std_msgs::String> side_message("side", &sideCb);

/*
Publishers
*/
ros::Publisher state("current_state", &process.output.current_state);
ros::Publisher flow_left("flow_left", &process.output.flow_left);
void setup() {
  controllino_setup();
  Serial.begin(9600);
  delay(1000);
  //Connect to ethernet
  Ethernet.begin(mac, ip);
  Serial.println("");
  Serial.println("Ethernet connected");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  //Subscribers
  nh.subscribe(side_message);
  nh.subscribe(cancel_message);

  //Publishers
  nh.advertise(state);
  nh.advertise(flow_left);
}

void loop() {
  /* 
    Inputs
  */
  //ROS Inputs
  nh.spinOnce();
  
  //Left flow
  process.flow_left = 5.313*((float(analogRead(FLOW_SENSOR_LEFT))*0.03/1.180)-4); // Wasser [4...20 mA]: Q [l/min] = 5,313 x (I - 4 mA)
  process.output.flow_left.data = process.flow_left;
  
  //Right flow
  process.flow_left = 5.313*((float(analogRead(FLOW_SENSOR_RIGHT))*0.03/1.180)-4); // Wasser [4...20 mA]: Q [l/min] = 5,313 x (I - 4 mA)
  process.output.flow_left.data = process.flow_left;
  //Water level
  process.water_level = 96 - 7 * ((float(analogRead(DISTANCE_SENSOR)) * 0.03/1.192)-4) + 8;

  //Reset all variables
  process.output.green_light = 0;
  process.output.red_light_continous = 0;
  process.output.red_light_dis = 0;
  process.output.buzzer = 0;
  
  //Temperature
  process.pt_widerstand = 1180*24/(float(analogRead(TEMP_SENSOR))*0.025)-1180;
  process.temperature = (process.pt_widerstand - 100)/(100 * 0.00385);
  Serial.println(process.pt_widerstand);
  /*
    Verarbeitung
  */
  step_change = 0; //Reset step change
  switch (process.current_step) {
    case 10:
    process.output.green_light = 1;
      //Wait for process to start
      if (process.input.start && process.input.side == "Rechts") {
        process.input.start = 0;
        process.current_step = 20;
        step_change = 1;
      } else if (process.input.start && process.input.side == "Links") {
        process.input.start = 0;
        process.current_step = 30;
        step_change = 1;
      }else if(process.water_level < process.water_treshhold){
        process.current_step = 15;
        step_change = 1;
      }
      break;
    case 15:
      /*
        Water level is to low
      */
      process.output.red_light_continous = 1;
      if(process.water_level > process.water_treshhold_restart){
        process.current_step = 10;
        step_change = 1;
      }
      break;
    case 20:
    /*
      Start the right pump
    */
      //Start pump timer
      if(!process.output.pump_right){
        process.timer = millis();
        Serial.println(process.timer);
      }
      //Turn pump and right valve on
      process.output.pump_right = 1;
      Serial.println(process.flow_left);
      if (process.input.cancel || millis() - process.timer > process.pump_timeout) {
        process.input.cancel = 0;
        process.current_step = 40;
        step_change = 1;
      } else if(millis()-process.timer> 1000 && process.flow_right < 1){
        process.current_step = 25;
        step_change = 0;
      }
      break;
    case 25:
      /*
        No flow on right side
      */
      process.output.pump_left = 0;
      process.output.buzzer =1;
      process.output.red_light_continous = 1;

      break;
    case 30:
    /*
      Start the left pump
    */
      //Start pump timer
      if(!process.output.pump_left){
        process.timer = millis();
        Serial.println(process.timer);
      }
      //Turn left pump on
      process.output.pump_left = 1;
      Serial.println(process.flow_left);
      flow_left.publish(&process.output.flow_left);
      if (process.input.cancel || millis() - process.timer > process.pump_timeout) {
        process.input.cancel = 0;
        process.current_step = 40;
        step_change = 1;
      } else if(millis()-process.timer> 1000 && process.flow_left < 1){
        process.current_step = 35;
        step_change = 0;
      }
      break;
    case 35:
    /*
      No flow on left side
    */
      process.output.pump_left = 0;
      process.output.buzzer =1;
      process.output.red_light_dis = 1;

      break;
    case 40:
      //Turn pump and valves off
      process.output.pump_left = 0;
      process.output.pump_right = 0;
      
      process.current_step = 10;
      step_change = 1;
      break;
    default:
      break;
  }

  /*
  Ausgabe
  */
  if (step_change) {
    itoa(process.current_step, process.output.current_state.data, 10);
    state.publish(&process.output.current_state);
  }
  
  //Pumps
  digitalWrite(PUMP_LEFT, process.output.pump_left);
  digitalWrite(PUMP_RIGHT, process.output.pump_right);

  //Signals
  digitalWrite(SIGNAL_GREEN, process.output.green_light);
  digitalWrite(SIGNAL_RED_CONTINOUS, process.output.red_light_continous);
  digitalWrite(SIGNAL_RED_DIS, process.output.red_light_dis);
  digitalWrite(BUZZER, process.output.buzzer);
}
