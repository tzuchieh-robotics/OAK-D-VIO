#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "SensorSync.cpp"
#include "VisualFrontend.cpp"
#include "Backend.cpp"

#include "depthai/depthai.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

int main() {
try {

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create device and pipeline
    // The device's own firmware self-crashes if its watchdog isn't serviced within
    // watchdogTimeoutMs (confirmed via a crash dump: default is 1500ms - see
    // C:/Users/User/AppData/Local/depthai/cache/crashdumps). If the host stalls that long -
    // e.g. a slow keyframe (feature detection + iSAM2 update) or a USB hiccup - the device
    // reboots mid-stream, which then surfaces on our side as a corrupted frame crashing
    // OpenCV. Raising the timeout gives the host more slack before that happens.
    dai::DeviceBase::Config deviceConfig;
    deviceConfig.board.watchdogTimeoutMs = 8000;
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(deviceConfig);
    dai::Pipeline pipeline(device);

    SensorSync sensorsync(pipeline);
    VisualFrontend visualFrontend;

    pipeline.start();

    // camera intrinsics -> gtsam::Cal3_S2 (needed by the SmartProjectionPoseFactor in Backend)
    auto calib = device->getCalibration();
    auto intrinsics = calib.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, 1200, 900);  // match SensorSync's requestOutput size
    auto K = boost::make_shared<gtsam::Cal3_S2>(intrinsics[0][0], intrinsics[1][1], 0, intrinsics[0][2], intrinsics[1][2]);

    // IMU-to-camera extrinsics -> gtsam::Pose3
    // On this specific device (OAK-D board BW1098) the IMU was never factory-calibrated against
    // the cameras - imuExtrinsics.toCameraSocket == -1 ("not linked to any camera"), and every
    // depthai-core read call that tries to resolve a transform through that missing link
    // crashes/hangs (confirmed with 4 different call combinations - looks like a depthai-core bug).
    // Values below are the real BNO086-on-OAK-D design-spec extrinsics from Luxonis's own board
    // database (github.com/luxonis/depthai-boards, boards/OAK-D.json, "imuExtrinsics" section) -
    // not measured on this specific unit, but the actual CAD spec for this board type, confirmed
    // by a Luxonis forum thread describing this exact "IMU calibration data is not available"
    // error and this exact fix. R happens to be its own inverse and t_z=0 makes -R*t == t, so the
    // same numbers work whether this is read as IMU->CAM_A or CAM_A->IMU.
    // Following the official pattern from depthai-core's own examples/cpp/IMU/imu_rotation_vector.cpp
    // (resolveImuExtrinsicsDestination there does the exact same AUTO->CAM_A fallback): patch this
    // into the calibration and push it back to the device with setCalibration(), instead of just
    // reading a broken value.
    std::vector<std::vector<float>> imuRotationSpec = {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    std::vector<float> imuTranslationSpecCm = {1.5f, 1.3662f, 0.0f};

    dai::CameraBoardSocket imuDestSocket = calib.getEepromData().imuExtrinsics.toCameraSocket;
    if (imuDestSocket == dai::CameraBoardSocket::AUTO) {
        imuDestSocket = dai::CameraBoardSocket::CAM_A;
    }
    calib.setImuExtrinsics(imuDestSocket, imuRotationSpec, imuTranslationSpecCm, imuTranslationSpecCm);
    device->setCalibration(calib);

    gtsam::Rot3 imuToCamRot(imuRotationSpec[0][0], imuRotationSpec[0][1], imuRotationSpec[0][2],
                             imuRotationSpec[1][0], imuRotationSpec[1][1], imuRotationSpec[1][2],
                             imuRotationSpec[2][0], imuRotationSpec[2][1], imuRotationSpec[2][2]);
    gtsam::Point3 imuToCamTrans(imuTranslationSpecCm[0] / 100.0, imuTranslationSpecCm[1] / 100.0, imuTranslationSpecCm[2] / 100.0);
    gtsam::Pose3 imuToCam(imuToCamRot, imuToCamTrans);

    Backend backend(K, imuToCam);

    while(pipeline.isRunning() && !quitEvent) {
        auto syncpacket = sensorsync.tryGetSyncedPacket();
        if (!syncpacket) continue;

        auto tracked = visualFrontend.track(syncpacket->frame->getCvFrame());

        for (size_t i = 1; i < syncpacket->imuBatch.size(); i++) {
            double dt = std::chrono::duration<double>(
                syncpacket->imuBatch[i].acceleroMeter.getTimestamp() - syncpacket->imuBatch[i - 1].acceleroMeter.getTimestamp()
            ).count();
            Eigen::Vector3d acc(syncpacket->imuBatch[i].acceleroMeter.x, syncpacket->imuBatch[i].acceleroMeter.y, syncpacket->imuBatch[i].acceleroMeter.z);
            Eigen::Vector3d gyro(syncpacket->imuBatch[i].gyroscope.x, syncpacket->imuBatch[i].gyroscope.y, syncpacket->imuBatch[i].gyroscope.z);
            backend.addImuMeasurement(acc, gyro, dt);
        }
        backend.addKeyframe(tracked);

        std::cout << backend.latestPose() << std::endl;
    }

    pipeline.stop();
    pipeline.wait();

    return 0;
} catch (const std::exception& e) {
    std::cerr << "[fatal] " << e.what() << std::flush << std::endl;
    return 1;
} catch (...) {
    std::cerr << "[fatal] unknown exception" << std::flush << std::endl;
    return 1;
}
}