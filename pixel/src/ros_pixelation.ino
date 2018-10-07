#include <ros.h>
#include <std_msgs/Char.h>




#define lf 2  
#define lb 3
#define le 5
#define rf 7
#define rb 6
#define re 4


#include <ros.h>
#include <std_msgs/Char.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Char& msg){
  
 char c = msg.data;
 if(c == 'W')
 {
   digitalWrite(lf, LOW);
   digitalWrite(lb, HIGH);
   analogWrite(le, 255);
   digitalWrite(rf, LOW);
   digitalWrite(rb, HIGH);
   analogWrite(re, 235);
  //ros::ROS_INFO("W");
   delay(100);
 }
 else if(c == 'A')
 {
   digitalWrite(lf, LOW);
   digitalWrite(lb, HIGH);
   analogWrite(le, 255);
   digitalWrite(rf, HIGH);
   digitalWrite(rb, LOW);
   analogWrite(re, 255);
   // ros::ROS_INFO("A");
   delay(50);
 }
 else if(c == 'D')
 {
  digitalWrite(lf, HIGH);
   digitalWrite(lb, LOW);
   analogWrite(le, 255);
   digitalWrite(rf, LOW);
   digitalWrite(rb, HIGH);
   analogWrite(re, 255);
   //ros::ROS_INFO("D");
   delay(50);
 }
   digitalWrite(lf, LOW);
   digitalWrite(lb, LOW);
   analogWrite(le, 255);
   digitalWrite(rf, LOW);
   digitalWrite(rb, LOW);
   analogWrite(re, 255);
 
 }
 ros::Subscriber<std_msgs::Char> sub("arduino", &messageCb ,1);

void setup() 
{
  pinMode(lf, 1);
  pinMode(lb, 1);
  pinMode(le, 1);
  pinMode(rf, 1);
  pinMode(rb, 1);
  pinMode(re, 1);
  pinMode(13, 1);
  
  Serial.begin(9600);
  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
  digitalWrite(le, LOW);
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
  digitalWrite(re, LOW);
  nh.initNode();
  nh.subscribe(sub);
}
void loop()
{
  nh.spinOnce();
  delay(1);
}
