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

int cosDistance(int angle, int distance, int referenceAngle){
    return (int) (distance * cos(abs(referenceAngle - angle) * M_PI / 180));
}

bool frontArc(int angle){
    return angle > 255 && angle < 285;
}

bool frontRightArc(int angle){
    return angle > 300 && angle < 330;
}

bool rightAnchorArc(int angle){
    return angle > 315 || angle < 45;
}

bool behindRightArc(int angle){
    return angle > 30 && angle < 60;
}

bool leftAnchorArc(int angle){
    return angle > 135 && angle < 225;
}

bool leftArc(int angle){
    return angle > 172.5 && angle < 187.5;
}

bool behindLeftArc(int angle){
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

    for (auto point : m_points) {
        int distance = point.distance;
        int angle = point.angle;

        if (distance <= 80) {
            continue;
        }
        
        if (frontArc(angle)) {
            frontDistanceTotal += cosDistance(angle, distance, 270);
            nbFrontDistancePoints++;
        }
        else if (frontRightArc(angle)) {
            frontRightDistanceTotal += cosDistance(angle, distance, 315);
            nbFrontRightDistancePoints++;
        }
        if (rightAnchorArc(angle)) {
            if (distance < rightAnchorDistance) {
                rightAnchorDistance = distance;
            }

            else if (angle > 345) {
            rightDistanceTotal += cosDistance(angle, distance, 360);
            rightFrontHalfDistanceTotal += cosDistance(angle, distance, 352.5);
            nbRightDistancePoints++;
            nbRightFrontHalfDistancePoints++;
            }
            else if (angle < 15) {
                rightDistanceTotal += cosDistance(angle, distance, 0);
                rightBehindHalfDistanceTotal += cosDistance(angle, distance, 7.5);
                nbRightDistancePoints++;
                nbRightBehindHalfDistancePoints++;
            }
        }
        else if (behindRightArc(angle)) {
            behindRightDistanceTotal += cosDistance(angle, distance, 45);
            nbBehindRightDistancePoints++;
        }
        else if (leftAnchorArc(angle)) {
            if (distance < leftAnchorDistance) {
                leftAnchorDistance = distance;
            }
            
            if (leftArc(angle)) {
                leftDistanceTotal += cosDistance(angle, distance, 180);
                nbLeftDistancePoints++;
            }
            else if (behindLeftArc(angle)) {
                leftBehindHalfDistanceTotal += cosDistance(angle, distance, 165);
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
    rightWallDistance =0;
    frontWallDistance=0;
    deviationAngle = 0.0;
    deviationAngleAlt =0.0;
}

void LidarServer::calculateDeviation(){
    deviationAngle = atan((rightFrontHalfDistance * cos(7.5 * M_PI / 180) - rightBehindHalfDistance * cos(7.5 * M_PI / 180)) / 
                                    (rightFrontHalfDistance * sin(7.5 * M_PI / 180) + rightBehindHalfDistance * sin(7.5 * M_PI / 180)));
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
    deviationAngleAlt = atan((leftBehindHalfDistance* cos(15 * M_PI / 180) - leftDistance) / 
                                        (leftBehindHalfDistance * sin(15 * M_PI / 180)));
    if (isnan(deviationAngleAlt)) {
        deviationAngleAlt = 0.0;
    }
}

std::string LidarServer::formatMessage(){
    return std::string("{\"distanceFront\": ") + std::to_string(frontDistance - 60) + 
                              std::string(", \"distanceFrontRight\": ") + std::to_string(frontRightDistance -  sqrt(pow(80, 2) + pow(60, 2))) + 
                              std::string(", \"distanceRight\": ") + std::to_string(rightDistance - 80) + 
                              std::string(", \"distanceBehindRight\": ") + std::to_string(behindRightDistance - sqrt(pow(80, 2) + pow(60, 2))) +
                              std::string(", \"distanceRightWall\": ") + std::to_string(rightWallDistance - 80) +
                              std::string(", \"distanceFrontWall\": ") + std::to_string(frontWallDistance - 60) +
                              std::string(", \"distanceLeftAnchor\": ") + std::to_string(leftAnchorDistance - 80) + 
                              std::string(", \"distanceRightAnchor\": ") + std::to_string(rightAnchorDistance - 80) + 
                              std::string(", \"deviationAngle\": ") + std::to_string(deviationAngle) +
                              std::string(", \"deviationAngleAlt\": ") + std::to_string(deviationAngleAlt) + std::string("}");
        
}

std::string LidarServer::getPositions(){
        calculateData();
        
        calculateDeviation();
        calculateFrontWallDistance();
        calculateRightWallDistance();
        calculateDeviationAngleAlt();
        std::string formatedMessage = formatMessage();
        resetValues();
        return formatedMessage;
}
