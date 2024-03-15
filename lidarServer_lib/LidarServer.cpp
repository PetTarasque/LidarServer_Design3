//
// Created by Hugo PJ on 2024-03-05.
//

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <utility>
#include <map>

#include "LidarServer.h"


using namespace std;


void LidarServer::updatePoints(vector<Point> points){
    m_points = std::move(points);;
}

map<string, double> LidarServer::detectObstacles(){
    cleanValues();
    return calculatePositions();
}

vector<Point> LidarServer::getPoints(){
    return m_points;
};

map<string, double> LidarServer::calculatePositions() {
    map<string, double> positions;
    pair<double, double> rightWallPositions = calculateRightWallPositions();
    double deviationAngle = rightWallPositions.second;
    positions["rightWall"] = rightWallPositions.first;
    positions["frontWall"] = calculateFrontWallDistance(deviationAngle);
    positions["cylinderDistance"] = 7432.4234;//To actually update
    positions["angle"] = deviationAngle;
    return positions;
}

void LidarServer::cleanValues(){
    deleteAbhorrentValues();
    deleteValuesCollidingWithRobot();
}

void LidarServer::deleteAbhorrentValues(){
    //this deletes the m_points that are below 4 cm, as they are for sure irrelevant
    m_points.erase(std::remove_if(m_points.begin(), m_points.end(), [&](const Point& point) {
        if (point.distance < 10){
            return true;
        }
        return false;
    }), m_points.end());
};

void LidarServer::deleteValuesCollidingWithRobot(){
    //<angleIntervalStart, angleIntervalEnd, minimumDistance>
    vector<tuple<double, double, int>> AnglesIntervals = {{15, 25, 60}, {48, 58, 120}, {122, 131, 120}, {175,185, 60}};

    m_points.erase(std::remove_if(m_points.begin(), m_points.end(), [&](const Point& point) {
        for (const auto& rangeAndDist : AnglesIntervals) {
            double minAngle = get<0>(rangeAndDist);
            double maxAngle = get<1>(rangeAndDist);
            double distance = get<2>(rangeAndDist);
            if ((point.angle >= minAngle && point.angle <= maxAngle && point.distance < distance)) {
                return true; // Delete if angle is within range and distance is below maxDistance
            }
        }
        return false;
    }), m_points.end());
};

Point LidarServer::calculateAveragePointOfArc(const std::vector<Point>& points) {
    double distanceSum = 0.0;
    double angleSum = 0.0;
    for (const auto& point : points) {
        distanceSum += point.distance;
        angleSum += point.angle;
    }
    Point pointAverage{};
    points.empty() ? pointAverage.angle = 0.0, pointAverage.distance=0.0 :
            pointAverage.angle = angleSum/points.size(), pointAverage.distance = distanceSum / points.size();
    return pointAverage;
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
        copy_if(m_points.begin(), m_points.end(), std::back_inserter(pointsOfInterval),
                [&startInterval, &endInterval](const Point& point) { return point.angle >= startInterval || point.angle <= endInterval;});
    } else {
        copy_if(m_points.begin(), m_points.end(), std::back_inserter(pointsOfInterval),
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
    /** TODO
     * should have a logic to deal with m_points that aren't valid.
     * maybe have 3 triangles taken across the wall and compare them
     * an arc sample should also verify if there's enough m_points for the sample, and if not, increase the aperture of the angle
     * or take another point
     */

    vector<Point> filteredPoint = getPointsInInterval(right_arc);

    pair<double, double> samplingForFirstPoint= splitArcInSubInterval(right_arc, 6, 2);
    Point A = calculateAveragePointOfArc(getPointsInInterval(samplingForFirstPoint));

    pair<double, double> samplingForSecondPoint= splitArcInSubInterval(right_arc, 6, 5);
    Point B = calculateAveragePointOfArc(getPointsInInterval(samplingForSecondPoint));

    double Angle = angleBetweenArcs(samplingForFirstPoint, samplingForSecondPoint);

    double triangleHeight = calculateHeightTriangle(A.distance, B.distance, Angle);
    double deviation = calculateDeviation(A.distance, B.distance, B.angle, Angle);

    return {triangleHeight, deviation};
}

double LidarServer::calculateFrontWallDistance(double deviationAngle){
    //this is the distance from the wall in front of the vehicle
    pair<double, double> adjustedFrontalArc = {frontal_arc.first-deviationAngle, frontal_arc.second-deviationAngle};
    vector<Point> frontalPoints = getPointsInInterval(adjustedFrontalArc);
    Point pointAverage = calculateAveragePointOfArc(frontalPoints);
    return pointAverage.distance;
};

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
    return moduloAngle(deviationAngle);
}
