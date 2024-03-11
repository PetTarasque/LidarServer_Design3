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
    pair<double, double> rightWallPositions = calculateRightWallPositions();
    positions["rightWall"] = rightWallPositions.first;
    positions["leftWall"] = 412412.223;
    positions["frontWall"] = 76542.21; //TODO
    positions["rearWall"] = 7432.4234;
    //positions["cylinderDistance"] = 7432.4234; ?
    positions["angle"] = rightWallPositions.second;
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

double LidarServer::moduloAngle(double angle){
    while(angle >= 360){
        angle -= 360;
    }
    if (angle < 0){
        angle += 360;
    }
    return angle;
}

pair<double, double> LidarServer::splitArcInSubInterval(pair<double, double> arc, int numberOfSections, int sectionSelected){
    //this function splits an angle in sections and returns the arc of that subsection
    int sectionSelectedAdjusted = clamp(sectionSelected, 1, 6); //ensure the section selected is within acceptable values
    if(sectionSelectedAdjusted != sectionSelected){
        cerr << "The given sectionSelected was invalid : " <<sectionSelected<<endl;
    }
    double sizeOfSections = angleOfArc(arc)/numberOfSections;
    double startOfSection = arc.first + (sectionSelected-1)*sizeOfSections;
    double endOfSection = startOfSection + sizeOfSections;
    return {moduloAngle(startOfSection), moduloAngle(endOfSection)};
}

double LidarServer::angleBetweenArcs(pair<double, double> firstArc, pair<double, double> secondArc){
    double middleFirstArc = middleOfArc(firstArc);
    double middleSecondArc = middleOfArc(secondArc);
    return angleOfArc({middleFirstArc, middleSecondArc});
};

double LidarServer::middleOfArc(pair<double, double> arc){
    if(arc.second < arc.first){
        return moduloAngle((arc.first + arc.second +360)/2);
    } else {
        return (arc.first + arc.second)/2;
    }
};

pair<double, double> LidarServer::calculateRightWallPositions() {
    //
    //the logic for calculating arcs should be more generic, and should detect if one of the interval is fucked
    //
    //take the front part of the arc? maybe have 3 triangles and verify if 2 are saying the same thing?
    //also need to take average of angle in the sample sections (and if section hasn't enough points, take another one)
    vector<Point> filteredPoint = getPointsInInterval(right_arc);

    pair<double, double> samplingForFirstPoint= splitArcInSubInterval(right_arc, 6, 2);
    double A = calculateAverageDistance(getPointsInInterval(samplingForFirstPoint));

    pair<double, double> samplingForSecondPoint= splitArcInSubInterval(right_arc, 6, 5);
    double B = calculateAverageDistance(getPointsInInterval(samplingForSecondPoint));

    double Angle = angleBetweenArcs(samplingForFirstPoint, samplingForSecondPoint);

    double triangleHeight = calculateHeightTriangle(A, B, Angle);
    double deviation = calculateDeviation(A, B, samplingForSecondPoint.second, Angle);

    return {triangleHeight, deviation};
}

double LidarServer::calculateHeightTriangle(double A, double B, double angle){
    double angleRadiant = angle * (M_PI / 180.0);
    double Area = 0.5 * A * B * sin(angleRadiant);
    double base = sqrt(A*A+ B*B- 2*A*B*cos(angleRadiant));
    double heightOfTriangle = 2 * Area / base;
    return heightOfTriangle;
}

double LidarServer::calculateDeviation(double A, double B, double angleB, double angle_AtoB){
    double angleRadiant_AtoB = angle_AtoB * (M_PI / 180.0);
    //we need to multiply by -1, because the Lidar has the positive angle going in a clockwise direction
    double angleFromHeightToB = -1 * atan((A*cos(angleRadiant_AtoB)-B)/(A*sin(angleRadiant_AtoB))) * (180.0/M_PI);
    double deviationAngle = angleFromHeightToB - angleB;
    return deviationAngle;
}
