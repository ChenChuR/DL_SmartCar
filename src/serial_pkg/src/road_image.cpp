#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "iostream"

#include "std_msgs/String.h"

#include "std_msgs/Int16.h"

#include <string>
#include "sensor_msgs/Image.h"
#include <vector>
#include "serial_pkg/road_image.h"
#include "serial_pkg/pid_new.h"

#define TURN_PWM_0 700
#define TURN_PWM_LEFT 0
#define TURN_PWM_RIGHT 2000

static PidTypeDef angle_pid;
const float angle_PID[3] = {1.18,0,0.45};
float angle_max_out = 100;
float angle_max_iout = 30;
// 2023.05.25 19：20 霍夫变换(没有补线) 37s 无罚时跑完
namespace road_ns
{
	void road_image::road_init(void)
	{
		PID_Init(&angle_pid,PID_POSITION,angle_PID,angle_max_out,angle_max_iout);
	}

	cv::Point road_image::FitPoint(cv::Mat Coe,int n,int x)
	{
		int i;
		double y = 0;
		for (i = n; i >= 0; i--)
		{
			y += std::pow(x, i) * (Coe.at<double>(i, 0));
		}
		return cv::Point(y,x);
	}

	void road_image::FitPolynomialCurve(const std::vector<cv::Point>& points, int n, cv::Mat& Coe) 
	{
		//最小二乘法多项式曲线拟合原理与实现 
		int N = points.size();
		cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
		for (int i = 0; i < n + 1; i++) {
			for (int j = 0; j < n + 1; j++) {
				for (int k = 0; k < N; k++) {
					X.at<double>(i, j) = X.at<double>(i, j) +
						std::pow(points[k].x, i + j);
				}
			}
		}
		cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
		for (int i = 0; i < n + 1; i++) {
			for (int k = 0; k < N; k++) {
				Y.at<double>(i, 0) = Y.at<double>(i, 0) +
					std::pow(points[k].x, i) * points[k].y;
			}
		}
		Coe = cv::Mat::zeros(n + 1, 1, CV_64FC1);
		cv::solve(X, Y, Coe, cv::DECOMP_LU);
	}

	void road_image::Light_Calc(double leftk)
	{
		vx = 0.9;
		if(leftk > -1.12 && leftk < 0)
		{
			std::cout << "________right________" << std::endl;
		 	turn_pwm = TURN_PWM_RIGHT;
		}
		else if(leftk > 0.8)
		{
			std::cout << "________left________" << std::endl;
			turn_pwm = TURN_PWM_LEFT;
		}
	}

	void road_image::Default_Calc(double k)
	{
		
		if(k < 4.5 && k > 0)
		{
			std::cout << "________left________" << std::endl;
			turn_pwm = TURN_PWM_LEFT;
			return;
		}
		else if(k > -4.0 && k < 0)
		{
			std::cout << "________right________" << std::endl;
			turn_pwm = TURN_PWM_RIGHT;
			return;
		}
		
	}

	void road_image::T_Turn(std::vector<cv::Point> left,std::vector<cv::Point> right,int height)
	{
		if(right.size() < height - 50 && left.size() < height - 50)
		{
			std::cout <<"T left"<< std::endl;
			turn_pwm = TURN_PWM_LEFT;			
		}

	}

	void road_image::image_process(cv::Mat& input_img)
	{
		cv::Rect rect(0, 240, 600, 120);
		//0 240 600 120
		cv::resize(input_img, input_img, cv::Size(600, 360),(0,0) ,(0,0),cv::INTER_AREA);
		cv::GaussianBlur(input_img, input_img, cv::Size(3, 3), 0);
		cv::cvtColor(input_img, input_img, CV_BGR2HSV);
		cv::inRange(input_img, cv::Scalar(26, 43, 83), cv::Scalar(90, 255, 255), input_img);
		input_img(rect).copyTo(input_img);
		cv::resize(input_img, input_img, cv::Size(300, 60), (0, 0), (0, 0), cv::INTER_LINEAR);
		dilate(input_img, input_img, cv::Mat(3, 3, CV_8UC1), cv::Point(-1, -1));
        erode(input_img, input_img, cv::Mat(3, 3, CV_8UC1), cv::Point(-1, -1));
		// cv:: Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
		// cv::morphologyEx(input_img, input_img, cv::MORPH_CLOSE, kernel);
		// cv::morphologyEx(input_img, input_img, cv::MORPH_OPEN, kernel);
	}

	void road_image::choose_lines(std::vector<cv::Vec4i> lines, std::vector<cv::Vec4i>& left_lines, std::vector<cv::Vec4i>& right_lines)
	{	
		float k_min = 0.4;
		float k_max = 5;
		float k;

		cv::Vec4i line;

		for (size_t i = 0; i < lines.size(); i++)
		{
			line = lines[i];	
			k = (line[1] - line[3]) * 1.0 / (line[0] - line[2]);
			//std::cout << "k:" << k << std::endl;
			if (abs(k) < k_max && abs(k) > k_min)
			{
				if (k < 0)
				{
					if ((line[0] + line[2])/2 > 180)
					{
						continue;
					}
					else
					{
						left_lines.push_back(line);
					}
				}
				else
				{
					if((line[0] + line[2])/2 < 100)
					{
						continue;
					}
					else
					{
						right_lines.push_back(line);
					}
				}
			}
			else
			{
				continue;
			}
		}
	}

	void road_image::select_lines(int h, std::vector<cv::Vec4i>& lines)
	{
		int distance_threshold = 5;
		int count = 0,max_count = 0;
		int standard_x;
		std::vector<int> buttom_x;
		std::vector<cv::Vec4i> new_lines;
		cv::Vec4i line;

		for (size_t i = 0; i < lines.size(); i++)
		{
			line = lines[i];
			buttom_x.push_back((line[0] - line[2]) * 1.0 / (line[1] - line[3]) * (h - line[1]) + line[0]);
		}
		standard_x = buttom_x[0];

		for (size_t i = 0; i < buttom_x.size(); i++)
		{
			count = 0;
			for (size_t j = 0; j < buttom_x.size(); j++)
			{
				if (buttom_x[i] - distance_threshold < buttom_x[j] && buttom_x[i] + distance_threshold > buttom_x[j])
				{
					count++;
				}
			}
			if (count > max_count)
			{
				standard_x = buttom_x[i];
				max_count = count;
			}
		}
		for (size_t i = 0; i < buttom_x.size(); i++)
		{
			if (buttom_x[i] >= standard_x - distance_threshold && buttom_x[i] <= standard_x + distance_threshold)
			{
				new_lines.push_back(lines[i]);
			}
		}
		lines = new_lines;
	}

	void road_image::Show_Image(std::string name,cv::Mat frame,bool show_image)
	{
		if(show_image == false)
			return;
		cv::imshow(name, frame);
		cv::waitKey(1);
	}
	

	void road_image::Calc_Speed(cv::Mat *frame,bool show_image)
	{
		double begin =ros::Time::now().toSec();
		cv::Mat LeftCoe,RightCoe;	
		cv::Mat way(60, 300, CV_8UC3, cv::Scalar(0, 0, 0));
		std::vector<cv::Point> left_points,right_points;
		std::vector<cv::Vec4i> lines, left_lines, right_lines;

		cv::Vec4i line;

		cv::Point l1, l2, r1, r2;
		int TurnSign;
		double offset = 0;
		int order = 1;
		int height = 80;

		image_process(*frame);
		
		int h = (*frame).rows;
		
		cv::HoughLinesP(*frame,lines,1,CV_PI/180,15,37,60); //45
		
		if (lines.size())
		{
			choose_lines(lines, left_lines, right_lines);  //分类
			/*std::cout << left_lines.size() << std::endl;
			std::cout << right_lines.size() << std::endl;*/
			
			if (left_lines.size() > 0)
			{
				select_lines(h, left_lines);  //筛�?
		
				//cv::circle(way, cv::Point((int)(line[0] - line[2]) * 1.0 / (line[1] - line[3]) * (h - line[1]) + line[0], height - 1), 3, cv::Scalar(255, 255, 255), -1);	
				for (size_t i = 0; i < left_lines.size(); i++)
				{
					line = left_lines[i];
					cv::line(way, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(123, 123, 123), 3, cv::LINE_AA);
					left_points.push_back(cv::Point(line[1], line[0]));
					left_points.push_back(cv::Point(line[3], line[2]));
				}
				FitPolynomialCurve(left_points, order, LeftCoe);
				l1 = FitPoint(LeftCoe, order, 0);
				l2 = FitPoint(LeftCoe, order, height - 1);
			}

			if (right_lines.size() > 0)
			{
				select_lines(h, right_lines);
				
				//cv::circle(way, cv::Point((int)(line[0] - line[2]) * 1.0 / (line[1] - line[3]) * (h - line[1]) + line[0], height - 1), 3, cv::Scalar(255, 255, 255), -1);
				for (size_t i = 0; i < right_lines.size(); i++)
				{
					line = right_lines[i];
					cv::line(way, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(123, 123, 123), 3, cv::LINE_AA);
					right_points.push_back(cv::Point(line[1], line[0]));
					right_points.push_back(cv::Point(line[3], line[2]));
				}
				FitPolynomialCurve(right_points, order, RightCoe);
				r1 = FitPoint(RightCoe, order, 0);
				r2 = FitPoint(RightCoe, order, height - 1);
			}
			cv::line(way, l1, l2, cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
			cv::line(way, r1, r2, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
			//1顶部�?????????????2底部
		}

		double leftk = 0;
		double rightk = 0;

		offset = ((l1.x + r1.x)/2 + (l2.x + r2.x)/2)/2 - 150;
		std::cout << "len1:" << r1.x - l1.x << std::endl;
		std::cout << "len2:" << r2.x - l2.x << std::endl; 
		if(offset > 90) offset = 90;
		else if(offset < -90) offset = -90;

		std::cout << "offset:" << offset<<std::endl;

		switch(road_state)
		{
			case SideWalk:
				vx = 1.1;
				break;
			case Ramp:
				vx = 0.8;
				break;
			default:
				break;
		}

		if (left_points.size() > 0)
		{
			leftk = (l1.y - l2.y) * 1.0 / (l1.x - l2.x);
		}
		if (right_points.size()>0)
		{
			rightk = (r1.y - r2.y) * 1.0 / (r1.x - r2.x);
		}

		//std::cout << "leftk:" <<leftk << std::endl;
		//std::cout << "rightk:" <<rightk << std::endl;
		int narrow_d = 105;
		int wide_d = 240;

		if (left_points.size() == 0 && right_points.size() > 0)
		{
			if(1)
			{
				turn_pwm = TURN_PWM_LEFT;
				std::cout << "_______________left_____________" << std::endl;
				cv::Mat three_mask;
				cv::cvtColor(*frame, three_mask, cv::COLOR_GRAY2BGR);
				Show_Image("mask", three_mask, show_image);
				Show_Image("way", way, show_image);
				return;
			}
			offset = 0;

		}
		else if(left_points.size() > 0 && right_points.size() == 0)
		{
			if(road_state != SideWalk)
			{
				turn_pwm = TURN_PWM_RIGHT;
				std::cout << "______________right____________" << std::endl;
				cv::Mat three_mask;
				cv::cvtColor(*frame, three_mask, cv::COLOR_GRAY2BGR);
				Show_Image("mask", three_mask, show_image);
				Show_Image("way", way, show_image);
				return;
			}
		}

		uint16_t wz_pwm = PID_Calc(&angle_pid,offset,0);
		ROS_INFO("%f %f",TURN_PWM_0-angle_pid.out,offset);	
		turn_pwm = (TURN_PWM_0-angle_pid.out);

		// cv::imshow("frame", *frame);
		cv::Mat three_mask;
    	cv::cvtColor(*frame, three_mask, cv::COLOR_GRAY2BGR);
		Show_Image("mask", three_mask, show_image);
		Show_Image("way", way, show_image);


		double end =ros::Time::now().toSec();
		std::cout << "Time :" << (end - begin) * 1000 << "ms" << std::endl;

		return;
	
	}


}


