//
// Created by Hugo PJ on 2024-03-05.
//

#ifndef UNTITLED1_LIDARSERVER_H
#define UNTITLED1_LIDARSERVER_H

#include <iostream>
#include "Point.h"
#include <map>
#include <vector>

using namespace std;

class LidarServer {
private:
    vector<Point> m_points;
    pair<int, int> right_arc = {270, 359};
    pair<int, int> frontal_arc = {265, 275};

public:
    map<string, double> detectObstacles();
    void updatePoints(vector<Point> points);

    //don't touch the following functions
    void cleanValues();
    void deleteValuesCollidingWithRobot();
    void deleteAbhorrentValues();
    pair<double, double> calculateRightWallPositions();
    map<string, double> calculatePositions();

    static double angleOfArc(pair<double, double> arc);
    static pair<double, double> splitArcInSubInterval(pair<double, double> arc, int numberOfSections, int sectionSelected);
    static Point calculateAveragePointOfArc(const std::vector<Point>& points);
    vector<Point> getPoints();
    vector<Point> getPointsInInterval(pair<int, int> arc);
    static double calculateHeightTriangle(double A, double B, double angle);
    static double calculateDeviation(double A, double B, double angleB, double angle);
    static double moduloAngle(double angle);
    static double angleBetweenArcs(pair<double, double> firstArc, pair<double, double> secondArc);
    static double middleOfArc(pair<double, double> arc);
    double calculateFrontWallDistance(double deviationAngle);
    pair<double, double> triangulate(int firstSectionSelected, int secondSectionSelected);
};


#endif //UNTITLED1_LIDARSERVER_H
