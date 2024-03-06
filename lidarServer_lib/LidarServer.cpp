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


vector<Point> LidarServer::getPoints(){
    return points;
};

map<string, double> LidarServer::calculatePositions() {
    map<string, double> positions;
    positions["rightWall"] = calculateRightWallDistance();
    positions["leftWall"] = 412412.223;
    positions["frontWall"] = 76542.21;
    positions["rearWall"] = 7432.4234;
    positions["angle"] = 3764.21;
    return positions;
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

double LidarServer::calculateRightWallDistance() {
    return 16.5;
};
