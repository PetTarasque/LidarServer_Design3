#define _USE_MATH_DEFINES
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include "ldlidar_driver/ldlidar_driver_linux.h"
#include "lidarServer_lib/LidarServer.h"
#include "lidarServer_lib/Point.h"


// int main(){
//     LidarServer* lidarServer = new LidarServer();
//     Point point;
//     point.angle = 80.00;
//     point.distance = 20.00;
//     std::cout << point.angle << " " << point.distance<< std::endl;
//     ldlidar::Points2D laser_scan_points;
//     return 0;
// }



uint64_t GetTimestamp(void)
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}


int main(int argc, char **argv)
{
  int serverSocket, clientSocket;
  struct sockaddr_in serverAddr, clientAddr;
  socklen_t clientAddrLen = sizeof(clientAddr);

  std::string ldlidar_type_str("LD19");
  std::string serial_port_name("/dev/ldlidar");
  ldlidar::LDType ldlidar_type_dest = ldlidar::LDType::LD_19;
  uint32_t serial_baudrate_val = 230400;

  ldlidar::LDLidarDriverLinuxInterface *lidar_drv = ldlidar::LDLidarDriverLinuxInterface::Create();

  LOG_INFO("LDLiDAR SDK Pack Version is %s", lidar_drv->GetLidarSdkVersionNumber().c_str());

  lidar_drv->RegisterGetTimestampFunctional(std::bind(&GetTimestamp));

  lidar_drv->EnablePointCloudDataFilter(true);

  if (lidar_drv->Connect(ldlidar_type_dest, serial_port_name, serial_baudrate_val))
  {
    LOG_INFO("ldlidar serial connect is success", "");
  }
  else
  {
    LOG_ERROR("ldlidar serial connect is fail", "");
    exit(EXIT_FAILURE);
  }

  if (lidar_drv->WaitLidarComm(3500))
  {
    LOG_INFO("ldlidar communication is normal.", "");
  }
  else
  {
    LOG_ERROR("ldlidar communication is abnormal.", "");
    lidar_drv->Disconnect();
  }

  if (lidar_drv->Start())
  {
    LOG_INFO_LITE("ldlidar driver start is success.", "");
  }
  else
  {
    LOG_ERROR_LITE("ldlidar driver start is fail.", "");
  }

  // Create server socket
  serverSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (serverSocket == -1)
  {
    std::cerr << "Error creating server socket" << std::endl;
    return 1;
  }

  // Set up server address
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = INADDR_ANY;
  serverAddr.sin_port = htons(8080);

  // Bind to port
  if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
  {
    std::cerr << "Error binding to port" << std::endl;
    close(serverSocket);
    return 1;
  }

  // Start listening
  if (listen(serverSocket, SOMAXCONN) == -1)
  {
    std::cerr << "Error listening on socket" << std::endl;
    close(serverSocket);
    return 1;
  }

  std::cout << "Server listening on port 8080" << std::endl;

  while (true)
  {
    // Accept connection from client
    clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &clientAddrLen);
    if (clientSocket == -1)
    {
      std::cerr << "Error accepting connection" << std::endl;
      close(serverSocket);
      return 1;
    }

    std::cout << "Client connected" << std::endl;
    bool clientConnected = true;

    ldlidar::Points2D laser_scan_points;

    LidarServer* lidarServer = new LidarServer();//server used to treat the given points

    while (ldlidar::LDLidarDriverLinuxInterface::Ok() && clientConnected)
    {
      switch (lidar_drv->GetLaserScanData(laser_scan_points, 500))
      {
      case ldlidar::LidarStatus::NORMAL:
      {

        vector<Point> lidarPoints;
        Point point;
         for (auto pointLaser : laser_scan_points) {
            point.angle = pointLaser.angle;
            point.distance = pointLaser.distance;
            lidarPoints.push_back(point);
        }

        map<string, double> positions;
        lidarServer->updatePoints(lidarPoints);
        try {
            positions = lidarServer->detectObstacles();
            const std::string message = std::string(
                    std::string("{\"distanceRightWall\": ") + std::to_string(positions["rightWall"] - 80) +
                    std::string(", \"distanceFrontWall\": ") + std::to_string(positions["frontWall"]-20) +
                    std::string(", \"deviationAngle\": ") + std::to_string((-1)*positions["angle"]* (M_PI / 180.0)) + std::string("}"));

            // Prepend the message length to the JSON data
            uint32_t msg_length = message.length();
            // Convert the message length to network byte order (big-endian)
            uint32_t network_order_msg_length = htonl(msg_length);
            // Send the message length
            if (send(clientSocket, &network_order_msg_length, sizeof(network_order_msg_length), MSG_NOSIGNAL) == -1) {
                std::cerr << "Error sending data" << std::endl;
                clientConnected = false;
            }

            cout << message<<endl;

            // Send data to server
            if (send(clientSocket, message.c_str(), msg_length, MSG_NOSIGNAL) == -1) {
                std::cerr << "Error sending data" << std::endl;
                clientConnected = false;
            }
        } catch (const std::runtime_error& e){
            std::cerr << "This is supposed to have been ignored."<< std::endl;
        }
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT:
      {
        LOG_ERROR_LITE("point cloud data publish time out, please check your lidar device.", "");
        lidar_drv->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT:
      {
        break;
      }
      default:
        break;
      }

      usleep(1000 * 100); // sleep 100ms , 10hz
    }
  }

  close(clientSocket);
  close(serverSocket);

  lidar_drv->Stop();
  lidar_drv->Disconnect();

  ldlidar::LDLidarDriverLinuxInterface::Destory(lidar_drv);

  return 0;
}
