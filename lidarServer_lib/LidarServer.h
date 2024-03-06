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
    void cleanValues();
    void deleteValuesCollidingWithRobot();
    void deleteAbhorrentValues();
    pair<double, double> calculateRightWallPositions();

    static double angleOfArc(pair<double, double> arc);
    static pair<double, double> splitArcInSubInterval(pair<double, double> arc, int numberOfSections, int sectionSelected);
    static double calculateAverageDistance(const std::vector<Point>& points);
    void readLidar();
    vector<Point> getPoints();
    map<string, double> getPositions();
    vector<Point> getPointsInInterval(pair<int, int> arc);
    static double calculateHeightTriangle(double A, double B, double angle);
    static double calculateDeviation(double A, double B, double angle, double angleA);
    static double moduloAngle(double angle);
    static double angleBetweenArcs(pair<double, double> firstArc, pair<double, double> secondArc);
    static double middleOfArc(pair<double, double> arc);
};


#endif //UNTITLED1_LIDARSERVER_H
