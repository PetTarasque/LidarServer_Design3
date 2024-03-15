#include <iostream>
#include <fstream>
#include <vector>
#include "Point.h"
#include "ldlidar_driver/ldlidar_driver_linux.h"
#include "ldlidar_driver/ldlidar_datatype.h"

//#include "include/ldlidar_driver/ldlidar_driver_linux.h"

void saveData(const std::vector<Point>& points, const std::string& filename, const std::vector<std::string>& additionalLines) {
    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "Error: Couldn't open file for writing\n";
        return;
    }

    for (const auto& line : additionalLines) {
        outFile << line << "\n";
    }

    outFile << "StartOfPoints"<<std::endl;


    for (const auto& point : points) {
        outFile << point.distance << " " << point.angle << "\n";
    }

    outFile.close();
}

std::vector<Point> loadData(const std::string& filename) {
    std::vector<Point> points;
    std::ifstream inFile(filename);
    if (!inFile) {
        std::cerr << "Error: Couldn't open file for reading\n";
        return points;
    }


    Point point;
    while (inFile >> point.distance >> point.angle) {
        points.push_back(point);
    }

    inFile.close();
    return points;
}



int main() {
    std::cout << "Helo world!" << std::endl;
    ldlidar::LidarStatus status = ldlidar::LidarStatus::NORMAL;
    std::cout << "No bugs!" << std::endl;
    return 0;
}

/**
   while (false) {
       std::vector<Point> points = {{3.0, 45.0},
                                    {5.0, 60.0},
                                    {7.0, 90.0}
       const std::string path = "../testData/";
       std::cout << "----------------- "<<std::endl;
       std::cout << "Enter the name of the file: ";
       std::string name;
       std::cin >> name;
       const std::string filename = path + name + ".txt";
       std::vector<std::string> additionalLines;

       std::cout << "0 for wall follow, 1 for intersection, 2 to eliminate this measure, other to end process ";
       std::string mode;
       std::cin >> mode;
       additionalLines.push_back(mode);

       if (mode == "0") {
           std::cout << "Enter the ANGLE of deviation : ";
           std::string angleAnswer;
           std::cin >> angleAnswer;

           std::cout << "Enter the RIGHT wall of deviation : ";
           std::string rightWallAnswer;
           std::cin >> rightWallAnswer;

           std::cout << "Enter the LEFT wall of deviation : ";
           std::string leftWallAnswer;
           std::cin >> leftWallAnswer;

           std::cout << "Enter the FRONT wall of deviation : ";
           std::string frontWallAnswer;
           std::cin >> frontWallAnswer;

           std::cout << "Enter the REAR wall of deviation : ";
           std::string rearWallAnswer;
           std::cin >> rearWallAnswer;

           additionalLines.push_back(angleAnswer);
           additionalLines.push_back(rightWallAnswer);
           additionalLines.push_back(leftWallAnswer);
           additionalLines.push_back(frontWallAnswer);
           additionalLines.push_back(rearWallAnswer);
       }

       if (mode == "1") {
           std::cout << "Enter the ANGLE of deviation in the intersection: ";
           std::string angleAnswer;
           std::cin >> angleAnswer;

           std::cout << "Enter the RIGHT intersection of deviation: ";
           std::string rightWallAnswer;
           std::cin >> rightWallAnswer;

           std::cout << "Enter the LEFT wall of deviation : ";
           std::string leftWallAnswer;
           std::cin >> leftWallAnswer;

           std::cout << "Enter the FRONT wall of deviation : ";
           std::string frontWallAnswer;
           std::cin >> frontWallAnswer;

           std::cout << "Enter the REAR wall of deviation : ";
           std::string rearWallAnswer;
           std::cin >> rearWallAnswer;

           additionalLines.push_back(angleAnswer);
           additionalLines.push_back(rightWallAnswer);
           additionalLines.push_back(leftWallAnswer);
           additionalLines.push_back(frontWallAnswer);
           additionalLines.push_back(rearWallAnswer);
       }

       if (mode == "2") {
           std::cout << "Cancelling this measure : " << std::endl;
           continue;
       }

       if (mode != "1" && mode != "0" && mode != "2") {
           std::cout << "Ending sampling : " <<std::endl;
           return 0;
       }

       // Save data to file
       saveData(points, filename, additionalLines);
//1
//        // Load data from file
//        std::vector<Point> loadedPoints = loadData(filename);
//
//        // Display loaded data
//        for (const auto &point: loadedPoints) {
//            std::cout << "Distance: " << point.distance << ", Angle: " << point.angle << "\n";
//        }
   }
   */

//    std::vector<Point> points = loadData(".b./testData/box.txt");
//
//    if(!points.empty()){
//        std::cout<< "it works"<<std::endl;
//    }

//    std::ifstream inFile("./testData/box.txt");
//
//    if (!inFile) {
//        std::cerr << "Error: Couldn't open file for reading\n";
//    }
