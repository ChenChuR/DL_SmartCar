#ifndef __ROAD_IMAGE_H
#define __ROAD_IMAGE_H

#include "stdio.h"
#include "serial_pkg/pid_new.h"

#define buff_len 7

namespace road_ns
{
        class road_image
        {
                public:
                        typedef enum
                        {
                                SideWalk = 0,
                                Ramp,
                                Limit_Rate_Ready,
                                Limit_Rate_On,
                                S_Turn,
                                Light,
                                Left_Turn,
                                Stop,
                        }Road_State;

                        Road_State road_state;

                        float vx = 0;
                        uint16_t turn_pwm = 750;

                        void road_init(void);
                        void Calc_Speed(cv::Mat *frame,bool show_image);

                private:
		        float buff[buff_len];

                        void Average_PWM_Init(void);
                        float Average_PWM(float current_pwm);
                        void Show_Image(std::string name,cv::Mat frame,bool show_image);
                        void Light_Calc(double leftk);
                        void Default_Calc(double k);
                        int T_Turn(int height, int col, cv::Mat& mask);
                        cv::Point FitPoint(cv::Mat Coe,int n,int x);
                        void FitPolynomialCurve(const std::vector<cv::Point>& points, int n, cv::Mat& Coe) ;
                        void image_process(cv::Mat& input_img);
                        void choose_lines(std::vector<cv::Vec4i> lines, std::vector<cv::Vec4i>& left_lines, std::vector<cv::Vec4i>& right_lines, float k_min);
                        void select_lines(int h, std::vector<cv::Vec4i>& lines);
        };
}

#endif