/*
 *
 * - Start service to listen for node creation requests 
 * - on request for node, create it and keep track of it
 * 
 * # Specifically:
 *  - initialise a DataReader node
 *  - inside the datareader, publish images read from a directory 
 *      as a image_transport stream.
 *  
 */

#include <ElasticFusion.h>

#include <dirent.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string.h>

#include "utils/datareader.h"
#include "EFusionWrapper.h"

using namespace std;

void loop() {
}

//int main(int argc, char const *argv[]) {
//        /*
//     *  load rgb and depth images
//     *  process images with elastic fusion
//     *
//     *
//     */
//
//        Resolution::getInstance(640, 480);
//        Intrinsics::getInstance(528, 528, 320, 240);
//
//        pangolin::Params windowParams;
//        windowParams.Set("SAMPLE_BUFFERS", 0);
//        windowParams.Set("SAMPLES", 0);
//        pangolin::CreateWindowAndBind("Main", 1280, 800, windowParams);
//
////        EFusionWrapper efSlam;
////        efSlam.init();
//
//        cv::Mat        rgbImage;
//        cv::Mat        depthImage;
//        unsigned char *rgbData;
//        unsigned char *depthData;
//
//        DataReader dataReader;
//        cout << "Here1" << endl;
//
//        string rgbDirPath   = "/home/noorvir/Documents/data/RGBD_freiburg/rgbd_dataset_freiburg1_xyz/rgb/";
//        string depthDirPath = "/home/noorvir/Documents/data/RGBD_freiburg/rgbd_dataset_freiburg1_xyz/depth/";
//        string rgbPath;
//        string depthPath;
//
//        DIR *rgbDir;
//        DIR *depthDir;
//
//        struct dirent *rgbName;
//        struct dirent *depthName;
//
//        rgbDir   = opendir(rgbDirPath.c_str());
//        depthDir = opendir(depthDirPath.c_str());
//
//        if ((depthDir == NULL) || (rgbDir == NULL)) {
//                perror("Falied to open directory\n");
//        }
//
//        cv::namedWindow("RGB Image", cv::WINDOW_AUTOSIZE);
//        cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
//
//        for (int i = 0; i < 10; i++) {
//                // get pointer to next file
//                rgbName   = readdir(rgbDir);
//                depthName = readdir(depthDir);
//
//                if ((rgbName == NULL) || depthName == NULL)
//                        break;
//
//                if ((strcmp(rgbName->d_name, ".") != 0) &&
//                    (strcmp(rgbName->d_name, "..") != 0)) {
//                        rgbPath  = rgbDirPath + string(rgbName->d_name);
//                        rgbImage = dataReader.readRGBImage(rgbPath);
//                        cv::imshow("RGB Image", rgbImage);
//                }
//
//                if ((strcmp(depthName->d_name, ".") != 0) &&
//                    (strcmp(depthName->d_name, "..") != 0)) {
//                        depthPath  = depthDirPath + string(depthName->d_name);
//                        depthImage = dataReader.readDepthImage(depthPath);
//                        cv::imshow("Depth Image", depthImage);
//                }
//                cv::waitKey(1000);
//
//                rgbData   = rgbImage.data;
//                depthData = depthImage.data;
//
//                // eFusion.processFrame(rgbData, depthData, logReader->timestamp, currentPose, weightMultiplier);;
//        }
//
//        closedir(depthDir);
//        closedir(rgbDir);
//
//        return 0;
//}
