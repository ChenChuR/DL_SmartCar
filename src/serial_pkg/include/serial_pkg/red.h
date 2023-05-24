#ifndef __RED_H
#define __RED_H

#include "stdio.h"

namespace red_ns
{
        class red
        {
                public:
                        bool Red_Judge(cv::Mat frame);

                private:
                        // lightDetect function
                        int lightDetect(cv::Mat image);
                        int processImgR(cv::Mat);
                        int processImgG(cv::Mat);
                        bool isIntersected(cv::Rect, cv::Rect);
                        void circleDetection(cv::Mat frame, bool &detect);
                // lightDetect variable
        };
}

#endif
