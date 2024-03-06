//
// Created by Hugo PJ on 2024-03-05.
//

#ifndef UNTITLED1_LIDARSERVER_H
#define UNTITLED1_LIDARSERVER_H

#include <iostream>
#include "Point.h"

using namespace std;

class LidarServer {
private:
    string m_filePath;
    vector<Point> points;


public:
    explicit LidarServer(string filePath);
    map<string, double> calculatePositions();
    double calculateRightWallDistance();
    void cleanValues();
    void deleteValuesCollidingWithRobot();
    void deleteAbhorrentValues();

    void readLidar();
    vector<Point> getPoints();
};


#endif //UNTITLED1_LIDARSERVER_H
