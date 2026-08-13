#include "depthai/depthai.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <vector>

// State: [deltaR, deltaV, deltaP] mean, plus their covariance and Jacobians w.r.t. bias.
struct Preintegrated {
    Eigen::Matrix3d deltaR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d deltaV = Eigen::Vector3d::Zero();
    Eigen::Vector3d deltaP = Eigen::Vector3d::Zero();

    // Covariance of [deltaPhi, deltaV, deltaP] (9x9, noise propagated through the loop below)
    Eigen::Matrix<double, 9, 9> cov = Eigen::Matrix<double, 9, 9>::Zero();

    // Jacobians for first-order bias correction without re-integrating
    Eigen::Matrix3d dR_dbg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dV_dbg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dV_dba = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dP_dbg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dP_dba = Eigen::Matrix3d::Zero();
};

Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d skew;
    skew <<     0, -w.z(),  w.y(),
            w.z(),      0, -w.x(),
           -w.y(),  w.x(),      0;
    return skew;
}

// so(3) -> SO(3), Rodrigues' formula
Eigen::Matrix3d expSO3(const Eigen::Vector3d& theta) {
    double angle = theta.norm();
    Eigen::Matrix3d W = hat(theta);
    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity() + W;
    }
    return Eigen::Matrix3d::Identity()
         + (std::sin(angle) / angle) * W
         + ((1 - std::cos(angle)) / (angle * angle)) * (W * W);
}

// Right Jacobian of SO(3) at theta
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& theta) {
    double angle = theta.norm();
    Eigen::Matrix3d W = hat(theta);
    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity() - 0.5 * W;
    }
    return Eigen::Matrix3d::Identity()
         - ((1 - std::cos(angle)) / (angle * angle)) * W
         + ((angle - std::sin(angle)) / (angle * angle * angle)) * (W * W);
}

// bg/ba: 目前的 bias 估計 (從 backend 拿，這裡當常數用)
// noiseCov: 感測器原始雜訊協方差 Σ_η (6x6, 對角: [gyro_var x3, accel_var x3])，
//           要用感測器規格書的 noise density 換算，這裡先當參數傳入，還沒填實際數值
Preintegrated preintegrate(const std::vector<dai::IMUPacket>& imuBatch,
                            const Eigen::Vector3d& bg,
                            const Eigen::Vector3d& ba,
                            const Eigen::Matrix<double, 6, 6>& noiseCov) {
    Preintegrated s;

    for (size_t i = 1; i < imuBatch.size(); i++) {
        double dt = std::chrono::duration<double>(
            imuBatch[i].acceleroMeter.getTimestamp() - imuBatch[i - 1].acceleroMeter.getTimestamp()
        ).count();

        Eigen::Vector3d wMeas(imuBatch[i].gyroscope.x, imuBatch[i].gyroscope.y, imuBatch[i].gyroscope.z);
        Eigen::Vector3d aMeas(imuBatch[i].acceleroMeter.x, imuBatch[i].acceleroMeter.y, imuBatch[i].acceleroMeter.z);

        // ① 扣掉 bias
        Eigen::Vector3d w = wMeas - bg;
        Eigen::Vector3d a = aMeas - ba;

        // ② 這一步的旋轉增量 + right Jacobian
        Eigen::Vector3d theta = w * dt;
        Eigen::Matrix3d dR = expSO3(theta);
        Eigen::Matrix3d Jr = rightJacobian(theta);
        Eigen::Matrix3d aHat = hat(a);

        // ④ 協方差傳遞：A (9x9 state transition), B (9x6 noise input)
        Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
        A.block<3, 3>(0, 0) = dR.transpose();
        A.block<3, 3>(3, 0) = -s.deltaR * aHat * dt;
        A.block<3, 3>(6, 0) = -0.5 * s.deltaR * aHat * dt * dt;
        A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
        B.block<3, 3>(0, 0) = Jr * dt;
        B.block<3, 3>(3, 3) = s.deltaR * dt;
        B.block<3, 3>(6, 3) = 0.5 * s.deltaR * dt * dt;

        s.cov = A * s.cov * A.transpose() + B * noiseCov * B.transpose();

        // ⑤ bias Jacobian 更新（要用「這一步更新前」的舊值，所以先算再統一寫回）
        Eigen::Matrix3d dR_dbg_new = dR.transpose() * s.dR_dbg - Jr * dt;
        Eigen::Matrix3d dP_dbg_new = s.dP_dbg + s.dV_dbg * dt - 0.5 * s.deltaR * aHat * s.dR_dbg * dt * dt;
        Eigen::Matrix3d dP_dba_new = s.dP_dba + s.dV_dba * dt - 0.5 * s.deltaR * dt * dt;
        Eigen::Matrix3d dV_dbg_new = s.dV_dbg - s.deltaR * aHat * s.dR_dbg * dt;
        Eigen::Matrix3d dV_dba_new = s.dV_dba - s.deltaR * dt;

        // ③ 均值更新（要用「更新前」的 ΔR/Δv/Δp）
        s.deltaP += s.deltaV * dt + 0.5 * (s.deltaR * a) * dt * dt;
        s.deltaV += (s.deltaR * a) * dt;
        s.deltaR  = s.deltaR * dR;

        // ⑥ 把這一步算出的新值設回累積變數
        s.dR_dbg = dR_dbg_new;
        s.dP_dbg = dP_dbg_new;
        s.dP_dba = dP_dba_new;
        s.dV_dbg = dV_dbg_new;
        s.dV_dba = dV_dba_new;
    }

    return s;
}
