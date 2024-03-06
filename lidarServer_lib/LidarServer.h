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
    map<string, double> m_positions;
    pair<int, int> right_arc = {330, 30};

public:
    explicit LidarServer(string filePath);
    void calculatePositions();
    double calculateRightWallDistance();
    void cleanValues();
    void deleteValuesCollidingWithRobot();
    void deleteAbhorrentValues();

    static double angleOfArc(pair<double, double> arc);
    static double calculateAverageDistance(const std::vector<Point>& points);
    void readLidar();
    vector<Point> getPoints();
    map<string, double> getPositions();
    vector<Point> getAngleIntervals(pair<int, int> arc);
};


#endif //UNTITLED1_LIDARSERVER_H
