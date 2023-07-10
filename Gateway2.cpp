#include <queue>
#include <thread>
#include <ros/ros.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <mutex>

int fd, n;

char flag = 0;

double value;
std::mutex q_mutex;

typedef struct {
	double value_pp[3];
	double value_ca[3];
} data_uart;

typedef struct{
	char flag;
	data_uart structure;
}data;

data received;

std::queue<double> q1_velocity_pp;
std::queue<double> q2_angle_pp;
std::queue<double> q3_brake_pp;
std::queue<double> q1_angle_ca;
std::queue<double> q2_brake_ca;

void file_init(void) {

	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (fd == -1) {
		std
		::cerr << "Error opening UART device\n";
		return;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0) {
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
	tty.c_cc[VMIN] = 8; // Wait for at least 8 characters
	tty.c_cc[VTIME] = 0; // Wait indefinitely

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		std
		::cerr << "Error setting UART attributes\n";
		return;
	}
}

void UARTThread(void) {

	
	double pp_throttle, pp_angle, pp_brake, ca_brake, ca_angle;

	while (true) {

		n = read(fd, &received.flag, sizeof(received.flag));

		while(received.flag != (char)0x66 && received.flag != (char)0x55)
		{
			read(fd, &received.flag, sizeof(received.flag));
		}
		printf("flag: %x\n", received.flag);
		
		
		flag = received.flag;


		if(flag == (char)0x66){
		n=read(fd,&received.structure.value_pp[0],8);
		char *ptr = (char *)&received.structure.value_pp[0];
		char *new_ptr = (char *)&pp_throttle;

		for(int i = 0;i < 8;i++)
		{
			new_ptr[i] = ptr[7-i];
		}
		q1_velocity_pp.push(pp_throttle);
		printf("throttle received :%lf\n",pp_throttle);
		usleep(20);
		n=read(fd,&received.structure.value_pp[1],8);
		char *ptr1 = (char *)&received.structure.value_pp[1];
		
		char *new_ptr1 = (char *)&pp_angle;

		for(int i = 0;i < 8;i++)
		{
			new_ptr1[i] = ptr1[7-i];
		}
		q2_angle_pp.push(pp_angle);
                printf("angle pp received :%lf\n",pp_angle);
		usleep(20);
		n=read(fd,&received.structure.value_pp[2],8);
		

		char *ptr2 = (char *)&received.structure.value_pp[2];
		char *new_ptr2 = (char *)&pp_brake;

		for(int i = 0;i < 8;i++)
		{
			new_ptr2[i] = ptr2[7-i];
		}

		q3_brake_pp.push(pp_brake);
                printf("brake pp received :%lf\n",pp_brake);
		
		
		}
		else if(flag == (char)0x55){
		
		n = read(fd,&received.structure.value_ca[0],8);
		char *ptr = (char *)&received.structure.value_ca[0];
		char *new_ptr = (char *)&ca_angle;
		for(int i = 0;i < 8;i++)
		{
			new_ptr[i] = ptr[7-i];
		}
		q1_angle_ca.push(ca_angle);
		printf("ca angle received:%lf\n",ca_angle);
		usleep(20);
			
		n=read(fd,&received.structure.value_ca[1],8);
		char *ptr1 = (char *)&received.structure.value_ca[1];
		
		char *new_ptr1 = (char *)&ca_brake;

		for(int i = 0;i < 8;i++)
		{
			new_ptr1[i] = ptr1[7-i];
		}
		q2_brake_ca.push(ca_brake);
		printf("ca brake received:%lf\n",ca_brake);
		n=read(fd,&received.structure.value_ca[2],8);
		}
				

		if (n < 0) {
			std
			::cerr << "Error reading UART device\n";
			break;
		} else if (n == 0) {
			continue; // No data available yet, try again
		}

	}
}

void ROSPublisherThread(void) {

	ros
	::NodeHandle nh;
        ros
	::Publisher pub1 = nh.advertise<std_msgs::Float64>("throttle_topic_control", 1000);
	ros
	::Publisher pub2 = nh.advertise<std_msgs::Float64>("steering_topic_control", 1000);
	ros
	::Publisher pub3 = nh.advertise<std_msgs::Float64>("brake_topic_control", 1000);
	
	
	while (ros::ok()) {

		if(flag == (char)0x66){
		if (!q1_velocity_pp.empty()&& !q2_angle_pp.empty() && !q3_brake_pp.empty()) {
			std_msgs
			::Float64 msg1;
			std_msgs
			::Float64 msg2;
			std_msgs
			::Float64 msg3;
			
			msg1.data = q1_velocity_pp.front();
			q1_velocity_pp.pop();

			msg2.data = q2_angle_pp.front();
			q2_angle_pp.pop();
			
			msg3.data = q3_brake_pp.front();
			q3_brake_pp.pop();

		

			pub1.publish(msg1); // Publish value at top of queue
			pub2.publish(msg2); // Publish value at top of queue
			pub3.publish(msg3); 
		
			ROS_INFO("published throttle pp: %lf", msg1.data);
			ROS_INFO("published angle pp: %lf", msg2.data);
			ROS_INFO("published brake pp: %lf", msg3.data);
			
		}
		}
		else if(flag == (char)0x55){
		if (!q1_angle_ca.empty()&& !q2_brake_ca.empty() ) {
			std_msgs
			::Float64 msg4;
			std_msgs
			::Float64 msg5;
			
			
			msg4.data = q1_angle_ca.front();
			q1_angle_ca.pop();

			msg5.data = q2_brake_ca.front();
			q2_brake_ca.pop();
			
		
			pub2.publish(msg4); // Publish value at top of queue
			pub3.publish(msg5); // Publish value at top of queue
			
			ROS_INFO("published angle ca: %lf", msg4.data);
			ROS_INFO("published brake ca: %lf", msg5.data);
			
		}
		}

		ros
	::spinOnce();
}

}

int main(int argc, char **argv) {

file_init();
ros
::init(argc, argv, "Receive_uart_prescan_publisher");
std
::thread UART(UARTThread);
std
::thread PubThread(ROSPublisherThread);
UART.join();
PubThread.join();

return 0;
}
