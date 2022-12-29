#ifndef PLANEEXTRACTOR_H
#define PLANEEXTRACTOR_H

#include<stdint.h>
#include<stdio.h>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<numeric>
#include<random>
#include<vector>
#include<utility>
#include<ctime>
#include "KeyFrame.h"
#include "MapPoint.h"
#include <opencv2/core/eigen.hpp>
#include<boost/filesystem.hpp>


namespace ORB_SLAM2 {
    class KeyFrame;
}
namespace cv {
    class Mat;
}
class PlaneExtractor
{
public:
    PlaneExtractor();
    void ComputePlanes(ORB_SLAM2::KeyFrame* kf);
    void GetPlanesfromLines(ORB_SLAM2::KeyFrame* kf);
    bool isZero(cv::Point3d pt);

    cv::Point3d getLineDirection(cv::Mat lines);

    bool checkFeasibility(int i,int j, ORB_SLAM2::KeyFrame* kf);

    bool checkRedundant(cv::Vec4f plane,ORB_SLAM2::KeyFrame *kf);

    cv::Point3d getIntersectionPoint(cv::Mat line1, cv::Mat line2);

    void saveAllPlanes(std::string mStrDateTime);
    void SaveAllPlaneLineSegments(std::string mStrDateTime);
    void InterKFPlaneMatch(ORB_SLAM2::KeyFrame *kf);
    cv::Mat CameraFramePlane(cv::Vec4f plane,ORB_SLAM2::KeyFrame *kf);
    cv::Mat WorldFramePlane(cv::Vec4f plane,ORB_SLAM2::KeyFrame *kf);
    cv::Mat CameraFrameLine(cv::Mat line,ORB_SLAM2::KeyFrame *kf);
    cv::Mat WorldFrameLine(cv::Mat line,ORB_SLAM2::KeyFrame *kf);
    void GetPlanesFromParallelLines(int i, int j,ORB_SLAM2::KeyFrame *kf);
    
    void Reset();

    std::pair<cv::Mat,cv::Mat> getPlaneLines(int i, int j, ORB_SLAM2::KeyFrame *kf);
    

private:

    std::vector<cv::Vec4f> mAllPlanes;
    std::vector<int> mPlaneValidity;
    std::vector<std::pair<cv::Mat,cv::Mat>> mAllPlaneLines;
    
};


#endif //LINEDETECTOR_H
