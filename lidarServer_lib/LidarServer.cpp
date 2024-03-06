//
// Created by Hugo PJ on 2024-03-05.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <map>

#include "LidarServer.h"


using namespace std;

LidarServer::LidarServer(string filePath) : m_filePath { std::move(filePath) }{
}

void LidarServer::readLidar() {
    std::ifstream inFile(m_filePath);

    if (!inFile) {
        std::cerr << "Error: Couldn't open file " << m_filePath<< " for reading\n";
    }

    Point point;
    while (inFile >> point.angle >> point.distance) {
        points.push_back(point);
    }

    inFile.close();
};

map<string, double> LidarServer::getPositions(){
    return m_positions;
};

vector<Point> LidarServer::getPoints(){
    return points;
};

void LidarServer::calculatePositions() {
    map<string, double> positions;
    positions["rightWall"] = calculateRightWallDistance();
    positions["leftWall"] = 412412.223;
    positions["frontWall"] = 76542.21;
    positions["rearWall"] = 7432.4234;
    positions["angle"] = 3764.21;
    m_positions = positions;
}

void LidarServer::cleanValues(){
    deleteAbhorrentValues();
    deleteValuesCollidingWithRobot();
    //should maybe try to delete collisions with other robots part?
}

void LidarServer::deleteAbhorrentValues(){
    //this deletes the points that are below 4 cm, as they are for sure irrelevant
    points.erase(std::remove_if(points.begin(), points.end(), [&](const Point& point) {
        if (point.distance < 10){
            return true;
        }
        return false;
    }), points.end());
};

void LidarServer::deleteValuesCollidingWithRobot(){
    //<angleIntervalStart, angleIntervalEnd, minimumDistance>
    vector<tuple<double, double, int>> AnglesIntervals = {{15, 25, 60}, {48, 58, 120}, {122, 131, 120}, {175,185, 60}};

    points.erase(std::remove_if(points.begin(), points.end(), [&](const Point& point) {
        for (const auto& rangeAndDist : AnglesIntervals) {
            double minAngle = get<0>(rangeAndDist);
            double maxAngle = get<1>(rangeAndDist);
            double distance = get<2>(rangeAndDist);
            if ((point.angle >= minAngle && point.angle <= maxAngle && point.distance < distance)) {
                return true; // Delete if angle is within range and distance is below maxDistance
            }
        }
        return false;
    }), points.end());
};

double LidarServer::calculateAverageDistance(const std::vector<Point>& points) {
    double sum = 0.0;
    for (const auto& point : points) {
        sum += point.distance;
    }
    return points.empty() ? 0.0 : sum / points.size();
}

double LidarServer::angleOfArc(pair<double, double> arc){
    int startInterval = arc.first;
    int endInterval = arc.second;
    double Angle;
    if (startInterval> endInterval){
        Angle = (360 - startInterval) + endInterval;
    } else {
        Angle = endInterval - startInterval;
    }
    return Angle;
}

vector<Point> LidarServer::getPointsInInterval(pair<int, int> arc){
    vector<Point> pointsOfInterval;
    int startInterval = arc.first;
    int endInterval = arc.second;
    if(startInterval > endInterval){
        //This distinction is necessary if for example the start of the interval is 345 and the end is 15
        copy_if(points.begin(), points.end(), std::back_inserter(pointsOfInterval),
                     [&startInterval, &endInterval](const Point& point) { return point.angle >= startInterval || point.angle <= endInterval;});
    } else {
        copy_if(points.begin(), points.end(), std::back_inserter(pointsOfInterval),
                [&startInterval, &endInterval](const Point& point) { return point.angle >= startInterval && point.angle <= endInterval;});
    }
    return pointsOfInterval;
}

double LidarServer::calculateRightWallDistance() {
    vector<Point> filteredPoint = getPointsInInterval(right_arc);
    double Angle = angleOfArc(right_arc);
    cout << "Angle : "<< Angle<<endl;
    double startFirstArc = (right_arc.first + Angle/6);
    double startFirstArcAdjusted = startFirstArc > 360 ? startFirstArc - 360 : startFirstArc;
    double endFirstArc = (right_arc.first + 2*Angle/6);
    double endFirstArcAdjusted = endFirstArc > 360 ? endFirstArc - 360 : endFirstArc;
    pair<double, double> firstPointOfTriangle= {startFirstArcAdjusted, endFirstArcAdjusted };
    cout << "firstPointOfTriangle : "<< startFirstArcAdjusted << " " << endFirstArcAdjusted <<endl;
    double A = calculateAverageDistance(getPointsInInterval(firstPointOfTriangle));
    cout << "A : "<< A<<endl;

    double startSecondArc = (right_arc.first + 4*Angle/6);
    double startSecondArcAdjusted = startSecondArc > 360 ? startSecondArc - 360 : startSecondArc;
    double endSecondArc = (right_arc.first + 5*Angle/6);
    double endSecondArcAdjusted = endSecondArc > 360 ? endSecondArc - 360 : endSecondArc;
    pair<double, double> secondPointOfTriangle= {startSecondArcAdjusted, endSecondArcAdjusted };
    cout << "secondPointOfTriangle : "<< startSecondArcAdjusted << " " << endSecondArcAdjusted <<endl;
    double B = calculateAverageDistance(getPointsInInterval(secondPointOfTriangle));
    cout << "B : "<< B <<endl;

    return calculateHeightTriangle(A, B, Angle);
}

double LidarServer::calculateHeightTriangle(double A, double B, double angle){
    double angleRadiant = angle * (M_PI / 180.0);
    double Area = 0.5 * A * B * sin(angleRadiant);
    double base = sqrt(A*A+ B*B- 2*A*B*cos(angleRadiant));
    double heightOfTriangle = 2 * Area / base;
    return heightOfTriangle;
}

