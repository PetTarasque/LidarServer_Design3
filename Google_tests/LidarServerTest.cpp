//
// Created by Hugo PJ on 2024-03-05.
//
#include "./lib/googletest/include/gtest/gtest.h"
#include "LidarServer.h"

using namespace std;
string FILE_PATH = "testData/box.txt";

class LidarServerFixture : public ::testing::Test{
protected:
    virtual void SetUp()
    {
        lidarServer = new LidarServer(FILE_PATH);
    }

    virtual void TearDown() {
        delete lidarServer;
    }

    LidarServer* lidarServer;
};

TEST_F(LidarServerFixture, testingGoogleTest){
    EXPECT_TRUE(true);
}