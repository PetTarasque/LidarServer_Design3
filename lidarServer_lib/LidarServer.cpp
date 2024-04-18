//
// Created by Hugo PJ on 2024-03-05.
//

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <utility>
#include <map>
#include <algorithm>
#include <cmath>
#include "ldlidar_driver/ldlidar_driver_linux.h"

#include "LidarServer.h"

using namespace std;


void LidarServer::updatePoints(ldlidar::Points2D laser_scan_points){
    m_points = std::move(laser_scan_points);;
}

double toRadians(int angle){
    return angle*M_PI/180;
}

int cosDistanceWithReferenceAngle(int angle, int distance, int referenceAngle){
    return (int) (distance * cos(toRadians(abs(referenceAngle - angle))));
}

bool isFrontArc(int angle){
    return angle > 255 && angle < 285;
}

bool isFrontRightArc(int angle){
    return angle > 300 && angle < 330;
}

bool isFrontRightAnchorArc(int angle){
    return angle > 270 && angle < 330;
}

bool isRightAnchorArc(int angle){
    return angle > 315 || angle < 45;
}

bool isBehindRightArc(int angle){
    return angle > 30 && angle < 60;
}

bool isLeftAnchorArc(int angle){
    return angle > 135 && angle < 225;
}

bool isLeftArc(int angle){
    return angle > 172.5 && angle < 187.5;
}

bool isBehindLeftArc(int angle){
    return angle > 157.5 && angle < 172.5;
}


void LidarServer::calculateData(){
    int nbFrontDistancePoints = 0;
    int nbFrontRightDistancePoints = 0;
    int nbRightDistancePoints = 0;
    int nbBehindRightDistancePoints = 0;
    int nbRightFrontHalfDistancePoints = 0;
    int nbRightBehindHalfDistancePoints = 0;
    int nbLeftDistancePoints = 0;
    int nbLeftBehindHalfDistancePoints = 0;

    int frontDistanceTotal = 0;
    int frontRightDistanceTotal = 0;
    int rightDistanceTotal = 0;
    int behindRightDistanceTotal = 0;
    int rightFrontHalfDistanceTotal = 0;
    int rightBehindHalfDistanceTotal = 0;
    int leftDistanceTotal = 0;
    int leftBehindHalfDistanceTotal = 0;

    for (int i = 0; i < m_points.size(); i++) {
        int distance = m_points[i].distance;
        int angle = m_points[i].angle;

        if (distance <= 80) {
            continue;
        }
        
        if (isFrontArc(angle)) {
            frontDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 270);
            nbFrontDistancePoints++;
        }
        else if (isFrontRightAnchorArc(angle)) {
            if (distance < frontRightAnchorDistance && cosDistanceWithReferenceAngle(angle, distance, 360) < 150) {
                frontRightAnchorDistance = cosDistanceWithReferenceAngle(angle, distance, 360);
            }

            if (isFrontRightArc(angle)) {
                frontRightDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 315);
                nbFrontRightDistancePoints++;
            } 
        }
        if (isRightAnchorArc(angle)) {
            if (distance < rightAnchorDistance) {
                rightAnchorDistance = distance;
            }

            else if (angle > 345) {
                rightDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 360);
                rightFrontHalfDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 352.5);
                nbRightDistancePoints++;
                nbRightFrontHalfDistancePoints++;
            }
            else if (angle < 15) {
                rightDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 0);
                rightBehindHalfDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 7.5);
                nbRightDistancePoints++;
                nbRightBehindHalfDistancePoints++;
            }
        }
        else if (isBehindRightArc(angle)) {
            behindRightDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 45);
            nbBehindRightDistancePoints++;
        }
        else if (isLeftAnchorArc(angle)) {
            if (distance < leftAnchorDistance && distance > 335) {
                leftAnchorDistance = distance;
            }
            
            if (isLeftArc(angle)) {
                leftDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 180);
                nbLeftDistancePoints++;
            }
            else if (isBehindLeftArc(angle)) {
                leftBehindHalfDistanceTotal += cosDistanceWithReferenceAngle(angle, distance, 165);
                nbLeftBehindHalfDistancePoints++;
            }
        }
    }

    frontDistance = frontDistanceTotal / nbFrontDistancePoints;
    frontRightDistance = frontRightDistanceTotal / nbFrontRightDistancePoints;
    rightDistance = rightDistanceTotal / nbRightDistancePoints;
    behindRightDistance = behindRightDistanceTotal / nbBehindRightDistancePoints;
    rightFrontHalfDistance = rightFrontHalfDistanceTotal / nbRightFrontHalfDistancePoints;
    rightBehindHalfDistance = rightBehindHalfDistanceTotal / nbRightBehindHalfDistancePoints;
    leftDistance = leftDistanceTotal / nbLeftDistancePoints;
    leftBehindHalfDistance = leftBehindHalfDistanceTotal / nbLeftBehindHalfDistancePoints;
}

void LidarServer::resetValues(){
    frontDistance = 0;
    frontRightDistance = 0;
    rightDistance = 0;
    behindRightDistance = 0;
    rightFrontHalfDistance = 0;
    rightBehindHalfDistance = 0;
    leftDistance = 0;
    leftBehindHalfDistance = 0;
    rightWallDistance = 0;
    frontWallDistance = 0;

    deviationAngle = 0.0;
    deviationAngleAlt = 0.0;

    frontRightAnchorDistance = 800;
    leftAnchorDistance = 610;
    rightAnchorDistance = 610;
}

void LidarServer::calculateDeviation(){
    deviationAngle = atan((rightFrontHalfDistance * cos(toRadians(7.5)) - rightBehindHalfDistance * cos(toRadians(7.5))) / 
                                    (rightFrontHalfDistance * sin(toRadians(7.5)) + rightBehindHalfDistance * sin(toRadians(7.5))));
    if (isnan(deviationAngle)) {
        deviationAngle = 0.0;
    }
}

void LidarServer::calculateRightWallDistance(){
    rightWallDistance = rightDistance * cos(deviationAngle);
}

void LidarServer::calculateFrontWallDistance(){
     frontWallDistance = frontDistance * cos(deviationAngle);

}

void LidarServer::calculateDeviationAngleAlt(){
    deviationAngleAlt = atan((leftBehindHalfDistance* cos(toRadians(15)) - leftDistance) / 
                                        (leftBehindHalfDistance * sin(toRadians(15))));
    if (isnan(deviationAngleAlt)) {
        deviationAngleAlt = 0.0;
    }
}

std::string LidarServer::formatPositions(){
    return std::string("{\"distanceFront\": ") + std::to_string(frontDistance - 60) + 
                              std::string(", \"distanceFrontRight\": ") + std::to_string(frontRightDistance -  sqrt(pow(80, 2) + pow(60, 2))) + 
                              std::string(", \"distanceRight\": ") + std::to_string(rightDistance - 80) + 
                              std::string(", \"distanceBehindRight\": ") + std::to_string(behindRightDistance - sqrt(pow(80, 2) + pow(60, 2))) +
                              std::string(", \"distanceRightWall\": ") + std::to_string(rightWallDistance - 80) +
                              std::string(", \"distanceFrontWall\": ") + std::to_string(frontWallDistance - 60) +
                              std::string(", \"distanceLeftAnchor\": ") + std::to_string(leftAnchorDistance - 80) + 
                              std::string(", \"distanceRightAnchor\": ") + std::to_string(rightAnchorDistance - 80) + 
                              std::string(", \"distanceFrontRightAnchor\": ") + std::to_string(frontRightAnchorDistance - 80) + 
                              std::string(", \"deviationAngle\": ") + std::to_string(deviationAngle) +
                              std::string(", \"deviationAngleAlt\": ") + std::to_string(deviationAngleAlt) + std::string("}");
        
}

std::string LidarServer::getPositions(){
    calculateData();

    //calcuate Data before this part:
    calculateDeviation();
    calculateFrontWallDistance();
    calculateRightWallDistance();
    calculateDeviationAngleAlt();
    std::string formatedPositions = formatPositions();
    resetValues();
    return formatedPositions;
}
