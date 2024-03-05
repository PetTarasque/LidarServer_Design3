//
// Created by Hugo PJ on 2024-03-05.
//

#ifndef UNTITLED1_LIDARSERVER_H
#define UNTITLED1_LIDARSERVER_H

#include <iostream>
using namespace std;

class LidarServer {
private:
    string m_filePath;
public:
    LidarServer(string filePath);
    vector<int> calculatePositions();
};


#endif //UNTITLED1_LIDARSERVER_H
