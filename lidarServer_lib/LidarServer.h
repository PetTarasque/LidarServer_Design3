//
// Created by Hugo PJ on 2024-03-05.
//

#ifndef UNTITLED1_LIDARSERVER_H
#define UNTITLED1_LIDARSERVER_H

#include <iostream>
#include <map>
#include <vector>
#include "ldlidar_driver/ldlidar_driver_linux.h"

using namespace std;

class LidarServer {
private:

    int frontDistance;
    int frontRightDistance;
    int rightDistance;
    int behindRightDistance;
    int rightFrontHalfDistance;
    int rightBehindHalfDistance;
    int leftDistance;
    int leftBehindHalfDistance;
    int rightWallDistance;
    int frontWallDistance;

    float deviationAngle;
    float deviationAngleAlt;

    int frontRightAnchorDistance = 800;
    int leftAnchorDistance = 610;
    int rightAnchorDistance = 610;
    
    ldlidar::Points2D m_points;

public:
    map<string, double> detectObstacles();
    void updatePoints(ldlidar::Points2D laser_scan_points);

    std::string getPositions();
    void calculateData();
    void resetValues();
    std::string formatPositions();
    void calculateDeviation();
    void calculateRightWallDistance();
    void calculateFrontWallDistance();
    void calculateDeviationAngleAlt();
};
#endif //UNTITLED1_LIDARSERVER_H
