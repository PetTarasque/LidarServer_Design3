//
// Created by Hugo PJ on 2024-03-05.
//
#include "./lib/googletest/include/gtest/gtest.h"
#include "LidarServer.h"

//Positions of the robot in the sample file
using namespace std;
double RIGHT_WALL = 16;
double FRONT_WALL = 31;
double LEFT_WALL = 27;
double REAR_WALL = 53;
double ANGLE = 10; //this angle refers to the front of the robot, 10 degrees to the right
double PRECISION_VALUE = 1.0;
int NUMBER_OF_ANGLES = 500;
string FILE_PATH = "../../testData/box.txt";

class LidarServerFixture : public ::testing::Test{
protected:
    virtual void SetUp()
    {
        lidarServer = new LidarServer(FILE_PATH);
        lidarServer->readLidar();
    }

    virtual void TearDown() {
        delete lidarServer;
    }

    LidarServer* lidarServer;
};

bool checkIfWithinPrecisionRange(double referenceValue, double returnedValue){
    //check if the returned value is within PRECISION of the referenceValue
    return (referenceValue-PRECISION_VALUE < returnedValue && referenceValue+PRECISION_VALUE > returnedValue);
}

bool containsAngle(const vector<Point>& returnedPoints, int angle){
    return std::any_of(returnedPoints.begin(), returnedPoints.end(), [&](const Point& point) {
        return point.angle > angle && point.angle < angle + 1;
    });
}

TEST_F(LidarServerFixture, readLidarFetchesValues){
    vector<Point> returnedPoints = lidarServer->getPoints();
    ASSERT_FALSE(returnedPoints.empty());
}

TEST_F(LidarServerFixture, readLidarReturnsAllAngles){
    vector<Point> returnedPoints = lidarServer->getPoints();
    ASSERT_EQ(NUMBER_OF_ANGLES, returnedPoints.size());
}

TEST_F(LidarServerFixture, calculatePositionsUpdatesPositions){
    map<string, double> positions = lidarServer->calculatePositions();

    ASSERT_NE(positions.find("rightWall"), positions.end());
    ASSERT_NE(positions.find("leftWall"), positions.end());
    ASSERT_NE(positions.find("frontWall"), positions.end());
    ASSERT_NE(positions.find("rearWall"), positions.end());
    ASSERT_NE(positions.find("angle"), positions.end());
}

TEST_F(LidarServerFixture, cleanValuesEliminatesSomePoints){
    lidarServer->cleanValues();
    vector<Point> returnedPoints = lidarServer->getPoints();
    ASSERT_TRUE(NUMBER_OF_ANGLES >= returnedPoints.size());
}

TEST_F(LidarServerFixture, cleanPointCollidingWithRobot){
    lidarServer->cleanValues();

    vector<Point> returnedPoints = lidarServer->getPoints();
    ASSERT_FALSE(containsAngle(returnedPoints, 125));
}

TEST_F(LidarServerFixture, calculatesRightWallDistance){
    map<string, double> positions = lidarServer->calculatePositions();

    bool isWithinRange = checkIfWithinPrecisionRange(RIGHT_WALL, positions["rightWall"]);
    ASSERT_TRUE(isWithinRange);
}
