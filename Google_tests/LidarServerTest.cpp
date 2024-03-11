//
// Created by Hugo PJ on 2024-03-05.
//
#include "./lib/googletest/include/gtest/gtest.h"
#include "LidarServer.h"

//Positions of the robot in the sample file
using namespace std;
double RIGHT_WALL = 160;//in millimeter
double FRONT_WALL = 310;
double LEFT_WALL = 270;
double REAR_WALL = 530;
double ANGLE = 10; //this angle refers to the front of the robot, 10 degrees to the right
double PRECISION_VALUE = 10.0;
double ANGLE_PRECISION = 2.0;
int NUMBER_OF_ANGLES = 500;
string FILE_PATH = "../../testData/box.txt";

class LidarServerFixture : public ::testing::Test{
protected:
    virtual void SetUp()
    {
        lidarServer = new LidarServer(FILE_PATH);
        lidarServer->readLidar();
        lidarServer->cleanValues();
    }

    virtual void TearDown() {
        delete lidarServer;
    }

    LidarServer* lidarServer;
};

bool checkIfWithinPrecisionRange(double referenceValue, double returnedValue, double precision){
    //check if the returned value is within PRECISION of the referenceValue
    return referenceValue-precision < returnedValue && referenceValue+precision > returnedValue;
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
    LidarServer* serverNotCleaned = new LidarServer(FILE_PATH);
    serverNotCleaned->readLidar();
    vector<Point> returnedPoints = serverNotCleaned->getPoints();
    ASSERT_EQ(NUMBER_OF_ANGLES, returnedPoints.size());
}

TEST_F(LidarServerFixture, calculatePositionsUpdatesPositions){
    lidarServer->calculatePositions();
    map<string, double> positions = lidarServer->getPositions();

    ASSERT_NE(positions.find("rightWall"), positions.end());
    ASSERT_NE(positions.find("leftWall"), positions.end());
    ASSERT_NE(positions.find("frontWall"), positions.end());
    ASSERT_NE(positions.find("rearWall"), positions.end());
    ASSERT_NE(positions.find("angle"), positions.end());
}

TEST_F(LidarServerFixture, cleanValuesEliminatesSomePoints){
    LidarServer* serverNotCleaned = new LidarServer(FILE_PATH);
    serverNotCleaned->readLidar();
    serverNotCleaned->cleanValues();
    vector<Point> returnedPoints = serverNotCleaned->getPoints();
    ASSERT_TRUE(NUMBER_OF_ANGLES >= returnedPoints.size());
}

TEST_F(LidarServerFixture, cleanPointCollidingWithRobot){
    vector<Point> returnedPoints = lidarServer->getPoints();
    ASSERT_FALSE(containsAngle(returnedPoints, 125));
}

TEST_F(LidarServerFixture, angleIntervalReturnsPointsWithinRange){
    int StartOfAngleInterval = 15;
    int EndOfAngleInterval = 45;
    vector<Point> filteredPoints = lidarServer->getPointsInInterval({StartOfAngleInterval, EndOfAngleInterval});

    bool allAnglesWithinRange = true;
    for (const Point& point : filteredPoints){
        if(point.angle > EndOfAngleInterval && point.angle< StartOfAngleInterval){
            allAnglesWithinRange = false;
        }
    }

    ASSERT_TRUE(allAnglesWithinRange);
}

TEST_F(LidarServerFixture, angleIntervalReturnsPointsWithinRangeWhenPassing360){
    int StartOfAngleInterval = 330;
    int EndOfAngleInterval = 30;
    vector<Point> filteredPoints = lidarServer->getPointsInInterval({StartOfAngleInterval, EndOfAngleInterval});

    bool allAnglesWithinRange = true;
    for (const Point& point : filteredPoints){
        if(point.angle > EndOfAngleInterval && point.angle< StartOfAngleInterval){
            allAnglesWithinRange = false;
        }
    }

    ASSERT_TRUE(allAnglesWithinRange);
}

TEST_F(LidarServerFixture, angleInArc){
    ASSERT_EQ(30.0, lidarServer->angleOfArc({15.0, 45.0}));
    ASSERT_EQ(60.0, lidarServer->angleOfArc({330.0, 30.0}));
    ASSERT_EQ(270.0, lidarServer->angleOfArc({30.0, 300.0}));
}

TEST_F(LidarServerFixture, splitArcInSubInterval){
    pair<double, double> measuredArc = lidarServer->splitArcInSubInterval({330.0, 30.0}, 6, 2);
    pair<double, double> expectedArc = {340.0, 350.0};
    ASSERT_EQ(expectedArc, measuredArc);

    measuredArc = lidarServer->splitArcInSubInterval({330.0, 30.0}, 6, 5);
    expectedArc = {10.0, 20.0};
    ASSERT_EQ(expectedArc, measuredArc);

    measuredArc = lidarServer->splitArcInSubInterval({240.0, 290.0}, 5, 2);
    expectedArc = {250.0, 260.0};
    ASSERT_EQ(expectedArc, measuredArc);
}

TEST_F(LidarServerFixture, middleOfArc){
    ASSERT_EQ(345, lidarServer->middleOfArc({340, 350}));
    ASSERT_EQ(20, lidarServer->middleOfArc({0, 40}));
    ASSERT_EQ(5, lidarServer->middleOfArc({350, 20}));
    ASSERT_EQ(315, lidarServer->middleOfArc({300, 330}));
    ASSERT_EQ(125, lidarServer->middleOfArc({100, 150}));
    ASSERT_EQ(0, lidarServer->middleOfArc({300, 60}));
}

TEST_F(LidarServerFixture, calculateHeightOfTriangle){
    double height = lidarServer->calculateHeightTriangle(4, 6, 48);
    bool inRange = checkIfWithinPrecisionRange(4.0, height, 0.1);
    ASSERT_TRUE(inRange) << "Expected : " << 4.0 << endl<<"Actual : " << height << endl;

    height = lidarServer->calculateHeightTriangle(120, 100, 48);
    inRange = checkIfWithinPrecisionRange(97.64, height, 0.1);
    ASSERT_TRUE(inRange) << "Expected : " << 97.64 << endl<< "Actual : " << height << endl;

    height = lidarServer->calculateHeightTriangle(160, 237, 60);
    inRange = checkIfWithinPrecisionRange(157, height, 1);
    ASSERT_TRUE(inRange) << "Expected : " << 157 << endl<<"Actual : " << height << endl;
}

TEST_F(LidarServerFixture, moduloAngle){
    ASSERT_EQ(200, lidarServer->moduloAngle(200));
    ASSERT_EQ(80, lidarServer->moduloAngle(440));
    ASSERT_EQ(30, lidarServer->moduloAngle(390));
    ASSERT_EQ(20, lidarServer->moduloAngle(740));
    ASSERT_EQ(340, lidarServer->moduloAngle(-20));
    ASSERT_EQ(140, lidarServer->moduloAngle(-220));
    ASSERT_EQ(0, lidarServer->moduloAngle(0));
    ASSERT_EQ(0, lidarServer->moduloAngle(360));
}

TEST_F(LidarServerFixture, angleBetweenArcs){
    double calculatedAngleBetweenArcs = lidarServer->angleBetweenArcs({330, 340}, {10, 20});
    ASSERT_EQ(40, calculatedAngleBetweenArcs);

    calculatedAngleBetweenArcs = lidarServer->angleBetweenArcs({300, 310}, {320, 330});
    ASSERT_EQ(20, calculatedAngleBetweenArcs);

    calculatedAngleBetweenArcs = lidarServer->angleBetweenArcs({20, 50}, {320, 330});
    ASSERT_EQ(290, calculatedAngleBetweenArcs);
}

TEST_F(LidarServerFixture, splittingAnAngleInSubIntervals){
    pair<double, double> calculatedSubInterval = lidarServer->splitArcInSubInterval({330, 30}, 6, 2);
    pair<double, double> actualInterval = {340, 350};
    ASSERT_EQ(actualInterval, calculatedSubInterval);

    calculatedSubInterval = lidarServer->splitArcInSubInterval({330, 30}, 6, 5);
    actualInterval = {10, 20};
    ASSERT_EQ(actualInterval, calculatedSubInterval);

    calculatedSubInterval = lidarServer->splitArcInSubInterval({300, 330}, 6, 2);
    actualInterval = {305, 310};
    ASSERT_EQ(actualInterval, calculatedSubInterval);

    calculatedSubInterval = lidarServer->splitArcInSubInterval({10, 30}, 4, 1);
    actualInterval = {10, 15};
    ASSERT_EQ(actualInterval, calculatedSubInterval);
}

TEST_F(LidarServerFixture, calculateRightWallDistance){
    pair<double, double> rightWallPositions = lidarServer->calculateRightWallPositions();;
    bool isWithinRange = checkIfWithinPrecisionRange(RIGHT_WALL, rightWallPositions.first, PRECISION_VALUE);
    ASSERT_TRUE(isWithinRange)<<"Expected : " << RIGHT_WALL << "+-"<< PRECISION_VALUE << " Received : "<<rightWallPositions.first<<endl;;
}

TEST_F(LidarServerFixture, calculateRightWallAngle){
    pair<double, double> rightWallPositions = lidarServer->calculateRightWallPositions();;
    bool isWithinRange = checkIfWithinPrecisionRange(ANGLE, rightWallPositions.second, ANGLE_PRECISION);
    ASSERT_TRUE(isWithinRange)<<"Expected : " << ANGLE << "+-"<< ANGLE_PRECISION << " Received : "<<rightWallPositions.second<<endl;;
}