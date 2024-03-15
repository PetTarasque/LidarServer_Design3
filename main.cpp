#include <iostream>
#include <fstream>
#include <vector>
#include "Point.h"

int main() {
    //how to talk to the LidarServer:
    /*
     * instantiate a server by doing lidarServer = new LidarServer();
     * change you Points given by your Lidar into the Point class inside of lidarServer_lib
     * then update the points of the LidarServer using lidarServer->updatePoints(points);
     * then calculate the distances to the various obstacles using map<string, double> positions = lidarServer->detectObstacles();
     *
     * note : don't forget to update the points before calculating the positions!
     */
    return 0;
}
