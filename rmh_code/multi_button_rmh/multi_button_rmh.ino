
/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg1;
std_msgs::Bool pushed_msg2;
std_msgs::Bool pushed_msg3;
std_msgs::Bool pushed_msg4;

ros::Publisher pub_button1("pushed1", &pushed_msg1);
ros::Publisher pub_button2("pushed2", &pushed_msg2);
ros::Publisher pub_button3("pushed3", &pushed_msg3);
ros::Publisher pub_button4("pushed4", &pushed_msg4);

const int buttonPin1 = 2;  
const int buttonPin2 = 3; 
const int buttonPin3 = 4; 
const int buttonPin4 = 5;

bool last_reading1;
bool last_reading2;
bool last_reading3;
bool last_reading4;
long last_debounce_time1=0;
long last_debounce_time2=0;
long last_debounce_time3=0;
long last_debounce_time4=0;
long debounce_delay=50;
bool published1 = true;
bool published2 = true;
bool published3 = true;
bool published4 = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button1);
  nh.advertise(pub_button2);
  nh.advertise(pub_button3);
  nh.advertise(pub_button4);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  pinMode(buttonPin4, INPUT);
  

  
  //The button is a normally button
  last_reading1 = ! digitalRead(buttonPin1);
  last_reading2 = ! digitalRead(buttonPin2);
  last_reading3 = ! digitalRead(buttonPin3);
  last_reading4 = ! digitalRead(buttonPin4);
 
}

void loop()
{
  
  bool reading1 = ! digitalRead(buttonPin1);
  bool reading2 = ! digitalRead(buttonPin2);
  bool reading3 = ! digitalRead(buttonPin3);
  bool reading4 = ! digitalRead(buttonPin4);
  
  if (last_reading1!= reading1){
      last_debounce_time1 = millis();
      published1 = false;
      }
  if (last_reading2!= reading2){
    last_debounce_time2 = millis();
    published2 = false;
    }
      
   if (last_reading3!= reading3){
    last_debounce_time3 = millis();
    published3 = false;
    }
  if (last_reading4!= reading4){
    last_debounce_time4 = millis();
    published4 = false;
    }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published1 && (millis() - last_debounce_time1)  > debounce_delay) {
    
    pushed_msg1.data = reading1;
    pub_button1.publish(&pushed_msg1);
    published1 = true;
    }
    
  if ( !published2 && (millis() - last_debounce_time2)  > debounce_delay) {
  
  pushed_msg2.data = reading2;
  pub_button2.publish(&pushed_msg2);
  published2 = true;
  }
  
   if ( !published3 && (millis() - last_debounce_time3)  > debounce_delay) {
  
  pushed_msg3.data = reading3;
  pub_button3.publish(&pushed_msg3);
  published3 = true;
  }
  
    if ( !published4 && (millis() - last_debounce_time4)  > debounce_delay) {
  
  pushed_msg4.data = reading4;
  pub_button4.publish(&pushed_msg4);
  published4 = true;
  }

  last_reading1 = reading1;
  last_reading2 = reading2;
  last_reading3 = reading3;
  last_reading4 = reading4;
  
  nh.spinOnce();
}
