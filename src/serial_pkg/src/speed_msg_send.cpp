#include "ros/ros.h"
#include "serial/serial.h"
//#include "serial_pkg/speed.h"
#include "chrono"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "serial_pkg/road_image.h"
#include "serial_pkg/red.h"
#include "std_msgs/Float32.h"

#define Start_Byte 0xA5
#define End_Byte 0x5A

serial::Serial ser;
//�������ݻ�����
unsigned char send_buff[9];
//
cv::VideoCapture cap;
cv::Mat frame;
road_ns::road_image road_image;
red_ns::red red;
int sidewalk_ = 0;

int8_t state = 0;//��¼״̬
int8_t lightnum = 0;//��¼���̵�
bool sidewalk = false;
bool ramp = false;
bool rate_limiting_on = false;
bool rate_limiting_off = false;
bool left_turn = false;
int limit = 0;
int leftWait = 0;
int leftturn = 0;
int greenwait = 0;
unsigned char signageLast;
unsigned char signage;
static bool left_turn_flag = false;
static bool show_image = true;

typedef union
{
    float float_data;
    unsigned char byte_data[4];
}Float_Byte;

struct
{
    Float_Byte vx;
    Float_Byte vx_last;
    uint16_t turn_pwm;
    uint16_t last_turn_pwm;
}speed_structure;

/**
 * @brief ��װ֡ͷ֡β������У��λ������������
*/
void Serial_Send(float vx,uint16_t turn_pwm)
{
	speed_structure.vx.float_data = vx;
	speed_structure.turn_pwm = turn_pwm;

    unsigned char sum = 0x00;
        
    send_buff[1] = speed_structure.vx.byte_data[0];
    send_buff[2] = speed_structure.vx.byte_data[1];
    send_buff[3] = speed_structure.vx.byte_data[2];
    send_buff[4] = speed_structure.vx.byte_data[3];

    send_buff[5] = ((speed_structure.turn_pwm & 0xFF00) >> 8);
    send_buff[6] = speed_structure.turn_pwm & 0x00FF;

    for(short i=1;i<7;i++)
        sum += send_buff[i];
    send_buff[7] = sum;
    ser.write(send_buff,9);

    ROS_INFO("%.2f %d",speed_structure.vx.float_data,speed_structure.turn_pwm);
}

//�򿪴���
int serial_init(void)
{
	try
        {
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch(serial::IOException &e)
        {
                ROS_ERROR_STREAM("Unable to open port ");
                return -1;
        }

	if(ser.isOpen())
                ROS_INFO_STREAM("Serial Port initialized");
	else
		return -1;
	
	send_buff[0] = Start_Byte;
    send_buff[8] = End_Byte;
    speed_structure.vx.float_data = 0;
    speed_structure.turn_pwm = 745;
	
	return 0;
}
//��������ͷ
int open_video(void)
{
    int device_ID = 0;
    int api_ID = cv::CAP_ANY;
    cap.open(device_ID + api_ID);
    if(!cap.isOpened())
    {
        ROS_ERROR("ERROR! Unable to open camera");
	    return -1;
    }
    return 0;
}

int init_all(void)
{
	if(serial_init() == -1)
		return -1;
	if(open_video() == -1)
		return -1;
	
	ros::param::set("sidewalk",false);
	ros::param::set("ramp",false);
	ros::param::set("rate_limiting_on",false);
    ros::param::set("rate_limiting_off",false);
    ros::param::set("left_turn",false);

    road_image.road_state = road_image.SideWalk;

	return 0;
}

void turn(uint16_t *pwm,uint16_t left,uint16_t right)
{
	if(*pwm == 0)
    {
		*pwm = left;
    }
    if(*pwm == 2000)
    {
        *pwm = right;
    }
}

void signage_check(float vx,uint16_t turn_pwm)
{
    ros::param::get("sidewalk", sidewalk);
    ros::param::get("ramp", ramp);
    ros::param::get("rate_limiting_on", rate_limiting_on);
    ros::param::get("rate_limiting_off", rate_limiting_off);
    ros::param::get("left_turn", left_turn);
	
    switch(road_image.road_state)
    {
        case road_image.SideWalk:
            turn(&turn_pwm,565,700); //560
            if(turn_pwm == 555)
                vx = 1.3;
            if(turn_pwm >= 740)
                turn_pwm = 740;
            if(turn_pwm == 565)
                sidewalk_++;
            if(sidewalk && sidewalk_ >= 10)
            {
                std::cout << "________sidewalk_________" << std::endl;
                road_image.road_state = road_image.Ramp;
                Serial_Send(0.0,700);
                ros::Duration(1).sleep();
            }
            break;
        case road_image.Ramp:
			turn(&turn_pwm,670,710);
            if(ramp)
            {
                road_image.road_state = road_image.Limit_Rate_Ready;
                Serial_Send(0.0,turn_pwm);
                ros::Duration(1).sleep();
            }
            break;
        case road_image.Limit_Rate_Ready:
            vx = 1.1;
            turn(&turn_pwm,555,700);
            if(rate_limiting_on)
            {
                road_image.road_state = road_image.Limit_Rate_On;
                Serial_Send(vx,780);
                ros::Duration(0.4).sleep(); // 0.2
            }
            break;
        case road_image.Limit_Rate_On:
            static double start1 = ros::Time::now().toSec();
			turn(&turn_pwm,700,700);
            vx = 0.4;
            if(rate_limiting_off || (ros::Time::now().toSec() - start1 >= 5.0))
            {
                road_image.road_state = road_image.S_Turn;
            }
            break;
        case road_image.S_Turn:
            turn(&turn_pwm,580,800);
            static bool left_state = false; 
            if(turn_pwm == 580 && left_state == false)
            {
                left_state = true;
            }
            if(turn_pwm != 580 && left_state == true)
            {
                vx = 0.4;
                if(turn_pwm == 800)
                {
                    road_image.road_state = road_image.Light;
                }
                // turn_pwm = 800;
            }
            break;
        case road_image.Light:
            vx = 0.72;
            if(turn_pwm < 660)
                turn_pwm = 660;
            if(turn_pwm > 740 && turn_pwm < 831)
                turn_pwm = 740;
            turn(&turn_pwm,700,831); // old_right : 825
            if(left_turn)
            {
				Serial_Send(0.0,turn_pwm);
                ros::Duration(3).sleep();
				bool light_detect = false;
                double start2 = ros::Time::now().toSec();
				while(light_detect == false)
				{
					cap.read(frame);
					light_detect = red.Red_Judge(frame);
                    if(ros::Time::now().toSec() - start2 > 10.0)
                        break;
				}
                road_image.road_state = road_image.Left_Turn;
            }
            break;
        case road_image.Left_Turn:
            vx = 0.8;
            static double start3 = ros::Time::now().toSec();
			turn(&turn_pwm,550,720);
            if(ros::Time::now().toSec() - start3 > 5.0)
                road_image.road_state = road_image.Stop;
            break;
        case road_image.Stop:
            vx = 0.0;
            break;
        default:
            break;
    }
	Serial_Send(vx,turn_pwm);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_send_msg");
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/road_image", 5);

    road_image.road_init();

	if(init_all() == -1)
		return -1;
        
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
		cap.read(frame);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    	pub.publish(msg);
		road_image.Calc_Speed(&frame,true);
        signage_check(road_image.vx,road_image.turn_pwm);
        loop_rate.sleep();
    }
    Serial_Send(0.0,700);
    return 0;
}