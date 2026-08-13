// This file print out camera info

#include "depthai/depthai.hpp"
#include <iostream>

int main() {
    dai::Device device;
    
    std::cout << "Device name: " << device.getDeviceName() << std::endl;
    
    auto cameras = device.getConnectedCameras();
    std::cout << "Connected cameras (" << cameras.size() << "):" << std::endl;
    
    for (const auto& cam : cameras) {
        std::cout << "  " << cam << std::endl;
    }
    
    auto camFeatures = device.getConnectedCameraFeatures();
    for (const auto& feature : camFeatures) {
        std::cout << "Socket: " << feature.socket 
                  << ", Sensor: " << feature.sensorName
                  << ", Max resolution: " << feature.width << "x" << feature.height
                  << std::endl;
    }
    
    return 0;
}