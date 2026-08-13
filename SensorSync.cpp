#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

struct SyncedPacket {
    std::shared_ptr<dai::ImgFrame> frame;
    std::vector<dai::IMUPacket> imuBatch;
};

class SensorSync {
public:
    explicit SensorSync(dai::Pipeline& pipeline);
    std::optional<SyncedPacket> tryGetSyncedPacket();

private:
    std::shared_ptr<dai::MessageQueue> videoQueue_;
    std::shared_ptr<dai::MessageQueue> imuQueue_;
    std::deque<dai::IMUPacket> imuBuffer_;
    std::deque<std::shared_ptr<dai::ImgFrame>> frameBuffer_;  // frames waiting for IMU data to catch up past their timestamp
};

SensorSync::SensorSync(dai::Pipeline& pipeline) {
    // Create cam queue
    auto cam = pipeline.create<dai::node::Camera>()->build();
    videoQueue_ = cam->requestOutput(std::make_pair(1200, 900))->createOutputQueue();

    // Create IMU queue
    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imuQueue_ = imu->out.createOutputQueue();
}

std::optional<SyncedPacket> SensorSync::tryGetSyncedPacket() {
    // buffer any new frame - don't try to sync it immediately, camera frames take
    // noticeably longer than IMU samples to reach the host (ISP processing, larger
    // payload), so a frame's timestamp can be "behind" IMU samples that already
    // arrived by the time we check. Releasing frames only once IMU data has caught
    // up past them (below) avoids depending on that latency gap being any particular size.
    if (auto frame = videoQueue_->tryGet<dai::ImgFrame>()) {
        frameBuffer_.push_back(frame);
    }

    // drain every IMUData message currently queued, not just one - IMU runs much
    // faster than the camera, so several messages can pile up between frames
    while (auto imuData = imuQueue_->tryGet<dai::IMUData>()) {
        for (auto packet : imuData->packets) {
            imuBuffer_.push_back(packet);
        }
    }

    if (frameBuffer_.empty()) return std::nullopt;

    auto frameTs = frameBuffer_.front()->getTimestamp();

    // only release the oldest pending frame once IMU data has actually progressed past it
    if (imuBuffer_.empty() || imuBuffer_.back().acceleroMeter.getTimestamp() < frameTs) {
        return std::nullopt;
    }

    auto frame = frameBuffer_.front();
    frameBuffer_.pop_front();

    std::vector<dai::IMUPacket> imuBatch;
    while(!imuBuffer_.empty() && imuBuffer_.front().acceleroMeter.getTimestamp() <= frameTs) {
        imuBatch.push_back(imuBuffer_.front());
        imuBuffer_.pop_front();
    }

    return SyncedPacket{frame, imuBatch};
}