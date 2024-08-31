#include <Ethernet.h>

//Use controllino library for aliases
#include <Controllino.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <string.h>

//Controlino output definitions
#define PUMP CONTROLLINO_D0
#define RIGHT_VALVE CONTROLLINO_D1
#define LEFT_VALVE CONTROLLINO_D2

void controllino_setup(){
  pinMode(PUMP, OUTPUT);
  pinMode(RIGHT_VALVE, OUTPUT);
  pinMode(LEFT_VALVE, OUTPUT);  
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
  bool pump = 0;
  bool right_valve = 0;
  bool left_valve = 0;
};
//Controlino Output
//Process class
class Process {
public:
  Input input;
  Output output;
  int current_step = 10;
  int water_quantity = 0;
};

Process process;

bool toggle_variable = 0;

/*
  ROS variables and funcs
*/
ros::NodeHandle nh;

//Subscribers
//Subscribe to start
void startCb(const std_msgs::Empty& toogle_msg) {
  Serial.println("Starting...");
  process.input.start = 1;
}
ros::Subscriber<std_msgs::Empty> start_message("start_process", &startCb);

//Subscribe to cancel
void cancelCb(const std_msgs::Empty& msg) {
  Serial.println("Starting...");
  process.input.cancel = 1;
}
ros::Subscriber<std_msgs::Empty> cancel_message("cancel_process", &cancelCb);

//Subscrube to side
void sideCb(const std_msgs::String& msg) {
  Serial.print("Changin side to: ");
  Serial.print(msg.data);
  process.input.side = msg.data;
}
ros::Subscriber<std_msgs::String> side_message("side", &sideCb);

//Publishers
ros::Publisher state("current_state", &process.output.current_state);

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
  nh.subscribe(start_message);
  nh.subscribe(side_message);
  nh.subscribe(cancel_message);

  //Publishers
  nh.advertise(state);
}

void loop() {
  // EVA Prinzip
  //Get all inputs
  nh.spinOnce();
  bool step_change = 0;
  /*
    Verarbeitung
  */
  switch (process.current_step) {
    case 10:
      //Wait for process to start
      if (process.input.start && process.input.side == "Rechts") {
        process.input.start = 0;
        process.current_step = 20;
        step_change = 1;
      } else if (process.input.start && process.input.side == "Links") {
        process.input.start = 0;
        process.current_step = 30;
        step_change = 1;
      }
      break;

    case 20:
      //Turn pump and right valve on
      process.output.pump = 1;
      process.output.right_valve = 1;

      if (process.input.cancel) {
        process.input.cancel = 0;
        process.current_step = 40;
        step_change = 1;
      }
      break;
    case 30:
      //Turn pump and right valve on
      process.output.pump = 1;
      process.output.left_valve = 1;

      if (process.input.cancel) {
        process.input.cancel = 0;
        process.current_step = 40;
        step_change = 1;
      }
      break;
    case 40:
      //Turn pump and valves off
      process.output.pump = 0;
      process.output.left_valve = 0;
      process.output.right_valve = 0;
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
  digitalWrite(PUMP, process.output.pump);
  digitalWrite(LEFT_VALVE, process.output.left_valve);
  digitalWrite(RIGHT_VALVE, process.output.right_valve);
}
