/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_sdk.h"
#include <csignal>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace unitree_lidar_sdk;

// Signal handler function
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    // Cleanup and close up stuff here  
    std::cout << "Set Lidar working mode to: STANDBY ... \n";
    // Assume lreader is a global or externally accessible pointer
    extern UnitreeLidarReader* lreader;
    lreader->setLidarWorkingMode(STANDBY);
    sleep(1);
    // Terminate program  
    exit(signum);  
}

UnitreeLidarReader* lreader = nullptr;

int main() {
    // Initialize Lidar Object
    lreader = createUnitreeLidarReader();
    int cloud_scan_num = 18;
    std::string port_name = "/dev/ttyUSB0";

    // Register signal SIGINT and signal handler  
    signal(SIGINT, signalHandler);

    if (lreader->initialize(cloud_scan_num, port_name)) {
        std::cerr << "Unilidar initialization failed! Exit here!\n";
        return -1;
    } else {
        std::cout << "Unilidar initialization succeed!\n";
    }

    // Set Lidar Working Mode
    std::cout << "Set Lidar working mode to: STANDBY ... \n";
    lreader->setLidarWorkingMode(STANDBY);
    sleep(1);

    std::cout << "Set Lidar working mode to: NORMAL ... \n";
    lreader->setLidarWorkingMode(NORMAL);
    sleep(1);

    std::cout << "\n";

    // Print Lidar Version
    while (true) {
        if (lreader->runParse() == VERSION) {
            std::cout << "lidar firmware version = " << lreader->getVersionOfFirmware() << "\n";
            break;
        }
        usleep(500);
    }
    std::cout << "lidar sdk version = " << lreader->getVersionOfSDK() << "\n\n";
    sleep(2);

    // Check lidar dirty percentage
    int count_percentage = 0;
    while (true) {
        if (lreader->runParse() == AUXILIARY) {
            std::cout << "Dirty Percentage = " << lreader->getDirtyPercentage() << " %\n";
            if (++count_percentage > 2) {
                break;
            }
            if (lreader->getDirtyPercentage() > 10) {
                std::cerr << "The protection cover is too dirty! Please clean it right now! Exit here ...\n";
                return 0;
            }
        }
        usleep(500);
    }
    std::cout << "\n";
    sleep(2);

    // Set LED
    std::cout << "Turn on all the LED lights ...\n";
    uint8_t led_table[45];
    std::fill_n(led_table, 45, 0xFF);
    lreader->setLEDDisplayMode(led_table);
    sleep(2);

    std::cout << "Turn off all the LED lights ...\n";
    std::fill_n(led_table, 45, 0x00);
    lreader->setLEDDisplayMode(led_table);
    sleep(2);

    std::cout << "Set LED mode to: FORWARD_SLOW ...\n";
    lreader->setLEDDisplayMode(FORWARD_SLOW);
    sleep(2);
    std::cout << "Set LED mode to: REVERSE_SLOW ...\n";
    lreader->setLEDDisplayMode(REVERSE_SLOW);
    sleep(2);

    std::cout << "Set LED mode to: SIXSTAGE_BREATHING ...\n";
    lreader->setLEDDisplayMode(SIXSTAGE_BREATHING);

    std::cout << "\n";
    sleep(2);

    // Parse PointCloud and IMU data
    MessageType result;
    while (true) {
        result = lreader->runParse(); // You need to call this function at least 1500Hz

        switch (result) {
            case NONE:
                break;

            case IMU:
                std::cout << "An IMU msg is parsed!\n";
                std::cout << "\tstamp = " << lreader->getIMU().stamp << ", id = " << lreader->getIMU().id << "\n";
                std::cout << "\tquaternion (x, y, z, w) = ["
                          << lreader->getIMU().quaternion[0] << ", "
                          << lreader->getIMU().quaternion[1] << ", "
                          << lreader->getIMU().quaternion[2] << ", "
                          << lreader->getIMU().quaternion[3] << "]\n";
                std::cout << "\ttimedelay (us) = " << lreader->getTimeDelay() << "\n\n";
                break;

            case POINTCLOUD:
                std::cout << "A Cloud msg is parsed! \n";
                std::cout << "\tstamp = " << lreader->getCloud().stamp << ", id = " << lreader->getCloud().id << "\n";
                std::cout << "\tcloud size = " << lreader->getCloud().points.size() << ", ringNum = " << lreader->getCloud().ringNum << "\n";
                std::cout << "\tfirst 10 points (x,y,z,intensity,time,ring) = \n";
                for (int i = 0; i < 10; ++i) { // print the first 10 points
                    std::cout << "\t  (" << lreader->getCloud().points[i].x << ", "
                              << lreader->getCloud().points[i].y << ", "
                              << lreader->getCloud().points[i].z << ", "
                              << lreader->getCloud().points[i].intensity << ", "
                              << lreader->getCloud().points[i].time << ", "
                              << lreader->getCloud().points[i].ring << ")\n";
                }
                std::cout << "\t  ...\n";
                std::cout << "\ttimedelay (us) = " << lreader->getTimeDelay() << "\n\n";
                break;

            default:
                break;
        }

        // usleep(500);
    }

    return 0;
}
