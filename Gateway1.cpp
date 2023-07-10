#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <thread>
#include <mutex>
#include <queue>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

 

// Global Vars
int file_id1;

 


int n1;
double allow_cad;
int vmin = 32;
double allow_ca = 666;

 

std::mutex q_mutex;

 

// Prototypes
void write_data(void);

 


// Queues:

 

// CA
std::queue <double> q1;
std::queue <double> q2;

 

// PP
std::queue <double> q1_velocity_PP;
std::queue <double> q2_steeringAngle_PP;
std::queue <double> q3_brake_PP;

 


// CA
typedef struct {
double flag;
double data1;
double data2;
}the_data;

 

the_data received;

 

// PP
typedef struct {
double flag;
double velocity;
double steeringAngle;
double brake;
}PP_data;

 

PP_data received_PP;

 

void file_init(int vmin) {
    file_id1 = open("/dev/ttyUSB9", O_RDWR | O_NOCTTY);
    if (file_id1 == -1) {
        std
        ::cerr << "Error opening UART device\n";
        return;
    }

 

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(file_id1, &tty) != 0) {
        std
        ::cerr << "Error getting UART attributes\n";
        return;
    }
    cfsetospeed(&tty, B115200); // Set baud rate
    tty.c_cflag &= ~PARENB; // Disable parity bit
    tty.c_cflag &= ~CSTOPB; // Set stop bit to 1
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // Set data bits to 8
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver and ignore modem control lines
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Set raw input mode
    tty.c_cc[VMIN] = vmin; // Wait for at least 16 characters
    tty.c_cc[VTIME] = 0; // Wait indefinitely

 

    if (tcsetattr(file_id1, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting UART attributes\n";
        return;
    }
}
void sub1_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    q_mutex.lock();
    q1.push(msg->data);
    q_mutex.unlock();
    ROS_INFO("Subscribed value angleCA: %lf", q1.front());

 

}

void sub2_Callback(const std_msgs::Float64::ConstPtr& msg2)
{
    q_mutex.lock();
    q2.push(msg2->data);
    q_mutex.unlock();
    ROS_INFO("Subscribed value brake CA: %lf", q2.front());
}

 

 

void sub3_Callback(const std_msgs::Float64::ConstPtr& msg1)
{
    q_mutex.lock();
    q1_velocity_PP.push(msg1->data);
    q_mutex.unlock();
    ROS_INFO("Subscribed value throttle PP: %lf", q1_velocity_PP.front());

 

}

void sub4_Callback(const std_msgs::Float64::ConstPtr& msg2)
{
    q_mutex.lock();
    q2_steeringAngle_PP.push(msg2->data);
    q_mutex.unlock();
    ROS_INFO("Subscribed value anglePP: %lf", q2_steeringAngle_PP.front());
}

 

void sub5_Callback(const std_msgs::Float64::ConstPtr& msg5)
{
    q_mutex.lock();
    q3_brake_PP.push(msg5->data);
    q_mutex.unlock();
    ROS_INFO("Subscribed value brakePP: %lf", q3_brake_PP.front());
}

 

void sub6_Callback(const std_msgs::Float64::ConstPtr& msg3)
{
    q_mutex.lock();
    if(msg3->data == 1)
    {
    allow_ca = 555;
    allow_cad = (double)allow_ca;
    vmin = 24;
    file_init(vmin);
    }
    else
    {
    allow_ca = 666;
    allow_cad = (double)allow_ca;
    vmin = 32;
    file_init(vmin);
    }

    q_mutex.unlock();
}

 

 

void sub_thread(void) {

 

    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("steering_topic_CA",1000, sub1_Callback);
    ros::Subscriber sub2 = n.subscribe("brake_topic_CA",1000, sub2_Callback);
    ros::Subscriber sub3 = n.subscribe("throttle_topic",1000, sub3_Callback);
    ros::Subscriber sub4 = n.subscribe("steering_topic",1000, sub4_Callback);
    ros::Subscriber sub5 = n.subscribe("brake_topic",1000, sub5_Callback);
    ros::Subscriber sub6 = n.subscribe("Alarm",1000, sub6_Callback);
    ros::spin();
}

 

void write_data(void) {

 

double value1;
double value2;
double velocityPP;
double steering_angle;
double brakepp;

 

while (true) {
    q_mutex.lock();
    if (!q1.empty() && !q2.empty() && !q1_velocity_PP.empty() && !q2_steeringAngle_PP.empty() && !q3_brake_PP.empty()) {

 

        value1 = q1.front();
        received.data1 = value1;
        q1.pop();

 

        value2 = q2.front();
        received.data2=value2;
        q2.pop();

 

            velocityPP = q1_velocity_PP.front();
        received_PP.velocity = velocityPP;
        q1_velocity_PP.pop();

 

        steering_angle = q2_steeringAngle_PP.front();
        received_PP.steeringAngle=steering_angle;
        q2_steeringAngle_PP.pop();

 

        brakepp = q3_brake_PP.front();
        received_PP.brake = brakepp;
        q3_brake_PP.pop();

 

        received.flag = (double)allow_ca;
        received_PP.flag = (double)allow_ca;

 

        q_mutex.unlock();


 

        if(allow_ca == 555){
        n1 = write(file_id1, &received, sizeof(received));
        if (n1 < 0) {
            std::cerr << "Error writing to UART device\n";

 

        } else {
        printf("Written value alarm ca: %lf\n" ,allow_cad );
        std::cout << "Written angle ca: " << value1 << std::endl;
        std::cout << "Written brake ca: " << value2 << std::endl;
               } 
        }

        else if(allow_ca == 666){
        n1 = write(file_id1, &received_PP, sizeof(received_PP));
        if (n1 < 0) {
            std::cerr << "Error writing to UART device\n";

 

        } else {
        printf("Written value alarm pp: %lf\n" ,allow_cad );
               std::cout << "Written velocity pp: " << velocityPP << std::endl;
        std::cout << "Written steering pp: " << steering_angle << std::endl;
        std::cout << "Written brake pp: " << brakepp << std::endl;

 

               }        
        }
    } else {
    q_mutex.unlock();
    }
    }
}

 

int main(int argc, char **argv){
file_init(vmin);
ros::init(argc, argv, "gateway1");
std::thread subscriber_thread(sub_thread);
std::thread thread2(write_data);
subscriber_thread.join();
thread2.join();
return 0;
}
