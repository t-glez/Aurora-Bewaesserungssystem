#include <Ethernet.h>

//Use controllino library for aliases
#include <Controllino.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Empty.h>

#include <string.h>


//Server information
byte mac[] = { 0x50, 0xD7, 0x53, 0x00, 0xEE, 0xD4 };
IPAddress ip(192,168,0,10);
IPAddress server(192,168,0,1);
const uint16_t serverPort = 11411;

/*
Create process class with all info needed
*/

//Input

class Input{
  public:
    bool start = 0;
    String side = "";
    bool cancel = 0;
};
//ROS Output

//Controlino Output
//Process class
class Process{
  public:
    Input input;
    int current_step = 10;
    int water_quantity = 0;
};

Process process;

bool toggle_variable = 0;

//ROS variables and funcs
void messageCb(const std_msgs::Empty& toogle_msg){
  Serial.println("Starting..."); 
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Empty> sub("toggle", &messageCb);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  //Connect to ethernet
  Ethernet.begin(mac, ip);
  Serial.println("");
  Serial.println("Ethernet connected");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());
  
  nh.getHardware()->setConnection(server,serverPort);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // EVA Prinzip
  //Get all inputs
  nh.spinOnce();
  
  /*
    Verarbeitung
  */
  switch(process.current_step){
    case 10:
      //Wait
      break;
      
    case 20:
      break;
    case 30:
      break;
    case 40:
      break;
  }
  
  /*
    Ausgabe
  */
}
