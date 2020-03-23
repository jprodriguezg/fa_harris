/****************************************************************************************
*  FA-Harris corner detection. 
*
*  This method will save corner events to `/corner/scene_/fa_harris.txt` 
*  with (x, y, t, p). Modify `corner.launch` file to choose scene before `launch`.
*
*  Author: Ruoxiang Li
*  Date: 2019.1.20
*
****************************************************************************************/

#ifndef FA_HARRIS_DETECTOR_H
#define FA_HARRIS_DETECTOR_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

namespace fa_harris 
{

class FAHarrisDetector
{
public:
    FAHarrisDetector();
    ~FAHarrisDetector();

    bool isCorner(const dvs_msgs::Event &e);
    bool isCornerWithoutFilter(const dvs_msgs::Event &e);
    bool addNewEventToTimeSurface(const dvs_msgs::Event &e);
    bool isCornerCandidate(const dvs_msgs::Event &e);
    ////  
    bool isCornerCandidateRefined(const dvs_msgs::Event &e);
    double getHarrisScore(int img_x, int img_y, bool polarity);
    bool checkPatch(const dvs_msgs::Event &e);
    Eigen::MatrixXi getPatch();

    bool isFiltered(const dvs_msgs::Event &e);

    void setSensorParams(int sensor_width, int sensor_height);
  
private:
    ///////////////////
    // Filter Parameters
    constexpr static const double filter_threshold_ = 0.050; //50 ms
    Eigen::MatrixXd sae_latest_[2];

    // Surface of Active Events
    Eigen::MatrixXd sae_[2];

    ///////////////////
    // Circular Breshenham Masks
    const int kSmallCircle_[16][2];
    const int kLargeCircle_[20][2];    

    ///////////////////
    Eigen::MatrixXi patch_;

    // pixels on local window 9*9
    int latest_event_local_;

    int window_size_;
    int kernel_size_;
    int sensor_width_;
    int sensor_height_;
    double harris_threshold_;

    // kernels
    Eigen::MatrixXd Gx_, h_;
    int factorial(int n) const;
    int pasc(int k, int n) const;
};

}

#endif // FA_HARRIS_DETECTOR_H