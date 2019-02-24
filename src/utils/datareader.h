

/*

- Take in folder location for RGB and depth images
- Take in file locations for accelerometer and groundtruth pose
- Open files as a stream and load them
- Pass them to elastic fusion


*/

#ifndef DATAREADER_H

#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string.h>

class DataReader{
    public:
        void readFromDirectory(std::string);
        void getNext();
        bool hasMore();
        cv::Mat readRGBImage(std::string);
        cv::Mat readDepthImage(std::string);
        DataReader();

    private:
        int j;
};

#endif // !DATAREADER_H
