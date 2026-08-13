#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

int main() {

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    std::cout << "Camera connected" << std::endl;

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create camera node
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto videoQueue = cam->requestOutput(std::make_pair(1200, 900))->createOutputQueue();

    // Create IMU node
    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    auto imuQueue = imu->out.createOutputQueue();

    std::deque<dai::IMUPacket> imuBuffer; // used for imu preintegration, holds both acc and gyro


    // Start pipeline
    pipeline.start();

    int frameCount = 0;
    auto fpsWindowStart = std::chrono::steady_clock::now();

    while(pipeline.isRunning() && !quitEvent) {
        // get imu data
        if (auto imuData = imuQueue->tryGet<dai::IMUData>()) {
            for (auto& packet : imuData->packets) {
                imuBuffer.push_back(packet);
            }
        }

        // get video data
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr){
            std::cout << "There is no video input" << std::endl;
            continue; // if no video input then skip this iteration
        }
        auto frameTs = videoIn->getTimestamp();

        // wrapping up imu batch
        std::vector<dai::IMUPacket> imuBatch;
        while (!imuBuffer.empty() && imuBuffer.front().acceleroMeter.getTimestamp() <= frameTs) {
            imuBatch.push_back(imuBuffer.front());
            imuBuffer.pop_front();
        }

        // counting FPS
        frameCount++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fpsWindowStart).count();
        if(elapsed >= 1.0) {
            std::cout << "FPS: " << frameCount / elapsed << std::endl;
            frameCount = 0;
            fpsWindowStart = now;
        }

        cv::imshow("video", videoIn->getCvFrame());

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    pipeline.stop();
    pipeline.wait();

    return 0;
}