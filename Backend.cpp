#include <Eigen/Dense>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

using gtsam::symbol_shorthand::X;  // Pose3, body pose at each keyframe
using gtsam::symbol_shorthand::V;  // Vector3, velocity at each keyframe
using gtsam::symbol_shorthand::B;  // imuBias::ConstantBias, one per keyframe, chained via CombinedImuFactor's built-in random walk

// Shortest-arc rotation that maps unit vector `from` onto unit vector `to`
// (Rodrigues' formula). Used to align the IMU's measured "up" direction with
// the nav frame's "up" axis for gravity-aligned initialization - this only
// constrains roll/pitch (rotation about the up axis is unobservable from a
// single gravity vector alone), which is why this gives the *shortest* such
// rotation rather than picking a yaw convention.
gtsam::Rot3 rotationAligningVectors(const gtsam::Vector3& from, const gtsam::Vector3& to) {
    gtsam::Vector3 a = from.normalized();
    gtsam::Vector3 b = to.normalized();
    gtsam::Vector3 v = a.cross(b);
    double s = v.norm();
    double c = a.dot(b);
    if (s < 1e-8) {
        if (c > 0) return gtsam::Rot3();
        // a and b point in exactly opposite directions - any axis perpendicular to a
        // gives a valid 180-degree rotation; pick one via a non-parallel reference vector.
        gtsam::Vector3 ref = (std::abs(a.x()) < 0.9) ? gtsam::Vector3(1, 0, 0) : gtsam::Vector3(0, 1, 0);
        gtsam::Vector3 axis = a.cross(ref).normalized();
        return gtsam::Rot3::AxisAngle(axis, 3.14159265358979323846);
    }
    gtsam::Matrix3 vx;
    vx << 0, -v.z(), v.y(),
          v.z(), 0, -v.x(),
          -v.y(), v.x(), 0;
    gtsam::Matrix3 R = gtsam::I_3x3 + vx + vx * vx * ((1 - c) / (s * s));
    return gtsam::Rot3(R);
}

class Backend {
public:
    Backend(const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& imuToCam);

    // called once per IMU sample between keyframes (from SyncedPacket.imuBatch)
    void addImuMeasurement(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

    // called once per synced frame: closes out the current IMU preintegration window,
    // adds the ImuFactor + this frame's visual observations, re-optimizes
    void addKeyframe(const TrackedFeatures& features);

    gtsam::Pose3 latestPose() const { return latestPose_; }

private:
    int index_ = 0;
    boost::shared_ptr<gtsam::Cal3_S2> K_;
    gtsam::Pose3 imuToCam_;

    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegrationParams_;
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_;

    // incremental solver: each addKeyframe() call feeds it only THIS keyframe's new factors/values
    // (not the whole graph) and it re-uses its internal Bayes tree from last time instead of
    // re-solving from scratch - the "not scalable" LevenbergMarquardtOptimizer re-solve is gone.
    gtsam::ISAM2 isam2_;

    // Default SmartProjectionParams degeneracyMode is IGNORE_DEGENERACY, which still folds a
    // degenerate landmark's (near-singular) information into the graph instead of dropping it -
    // with insufficient parallax between observations (e.g. mostly-rotational motion) that alone
    // was enough to make ISAM2's linear solve indeterminate. ZERO_ON_DEGENERACY makes a degenerate
    // landmark contribute nothing that keyframe instead of bad/singular information.
    gtsam::SmartProjectionParams smartFactorParams_{gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY};

    std::map<int, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr> smartFactors_;  // keyed by feature id
    // ISAM2 bakes each factor it's given into its internal Bayes tree at whatever index update()
    // assigns it - mutating a SmartProjectionPoseFactor object in place (adding a new observation)
    // after that doesn't change what ISAM2 already solved with. So an updated landmark's factor
    // must be removed from its old slot and re-inserted as a "new" factor; this tracks the current
    // slot for each landmark id so the next update knows what to remove.
    std::map<int, gtsam::FactorIndex> smartFactorIndices_;

    gtsam::Pose3 latestPose_;
    gtsam::Vector3 latestVelocity_ = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias latestBias_;

    // accumulated for gravity-aligned initialization of X(0) - see addImuMeasurement()
    gtsam::Vector3 accelAccum_ = gtsam::Vector3::Zero();
    int accelSampleCount_ = 0;
};

Backend::Backend(const boost::shared_ptr<gtsam::Cal3_S2>& K, const gtsam::Pose3& imuToCam)
    : K_(K), imuToCam_(imuToCam) {
    // Real BNO086 noise density read off this device's own EEPROM (calib_dump.json,
    // imuCalibrationParams.noise), averaged across x/y/z, squared to get variance
    // (GTSAM wants sigma in units/sqrt(Hz), so covariance = noiseDensity^2):
    //   accel noiseDensity ~ (0.0692, 0.0742, 0.0852) -> avg ~0.0762 -> var ~5.81e-3
    //   gyro  noiseDensity ~ (0.4652, 0.5860, 0.3907) -> avg ~0.4806 -> var ~2.31e-1
    // Not 100% sure these match GTSAM's expected units exactly (BNO086 datasheet nominal
    // noise density is usually smaller than what's reported here), but it's this device's
    // own reported data, not a guess - a better basis than an arbitrary placeholder.
    // integrationCovariance also bumped off the previous 1e-8: that was ~1e4 smaller than
    // the measurement covariances, which on its own can make the combined 9x9 preintMeasCov_
    // matrix ill-conditioned enough to blow up to NaN when GTSAM inverts it for the ImuFactor
    // error - keeping every covariance within a few orders of magnitude of each other avoids that.
    preintegrationParams_ = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
    preintegrationParams_->setGyroscopeCovariance(gtsam::I_3x3 * 0.231);
    preintegrationParams_->setAccelerometerCovariance(gtsam::I_3x3 * 5.81e-3);
    preintegrationParams_->setIntegrationCovariance(gtsam::I_3x3 * 1e-6);

    // Factory (Allan-variance-derived) bias random-walk coefficients, read via
    // calib.getImuNoiseParameters() on this device - describes how fast the bias ITSELF is
    // expected to drift, independent of the measurement noiseDensity above.
    // Used directly as the continuous-time variance here (NOT squared like noiseDensity is) -
    // squaring first gave a variance as small as ~2e-9, and even though GTSAM's own
    // ImuFactorsExample.cpp uses comparably tiny bias covariances without issue on its own, that
    // example has no vision factors in the mix. Combined with this graph's SmartProjectionPoseFactor
    // (already flagged as numerically marginal - see ZERO_ON_DEGENERACY/outlier-rejection above),
    // squaring first made ISAM2's linear solve indeterminate within ~15-50 keyframes every run.
    // Using the raw coefficient as variance keeps the bias chain enough slack to avoid that; unit
    // convention for this field isn't authoritatively documented, so this is an empirical call.
    gtsam::Vector3 accelBiasRandomWalk(4.61329e-05, 1.00647e-04, 3.60364e-05);
    gtsam::Vector3 gyroBiasRandomWalk(0.0491917, 0.09815, 0.0609802);
    preintegrationParams_->setBiasAccCovariance(accelBiasRandomWalk.asDiagonal());
    preintegrationParams_->setBiasOmegaCovariance(gyroBiasRandomWalk.asDiagonal());

    preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preintegrationParams_, latestBias_);

    // Reject a landmark's contribution outright once its reprojection error exceeds this many
    // pixels (in the same units as pixelNoise's sigma, below) - without this, a single bad
    // correspondence that RANSAC didn't catch (still possible - RANSAC only ran once, on the 2D
    // track between consecutive frames, not against the smart factor's full multi-view
    // triangulation) can drag the whole optimization off track instead of just being downweighted.
    smartFactorParams_.setDynamicOutlierRejectionThreshold(3.0);
}

void Backend::addImuMeasurement(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt) {
    // only needed to gravity-align X(0) - stop accumulating once the first keyframe has used it
    if (index_ == 0) {
        accelAccum_ += acc;
        accelSampleCount_++;
    }
    preintegrated_->integrateMeasurement(acc, gyro, dt);
}

void Backend::addKeyframe(const TrackedFeatures& features) {
    // Guard against building an ImuFactor from a preintegration window with zero measurements
    // (deltaTij() == 0 - happens when no IMU samples arrived between the previous keyframe and
    // this one, e.g. right at pipeline startup). GTSAM needs to invert the accumulated covariance
    // to compute the ImuFactor's error, and a zero-measurement window means that covariance is
    // still all-zero/singular, which comes out as NaN - and unlike the old batch-graph_ version,
    // ISAM2 would bake that NaN permanently into its internal Bayes tree (no way to undo an
    // update() short of removing the factor). Simplest safe fix: skip this frame's keyframe
    // entirely (drop its visual observations too) rather than let a broken ImuFactor in.
    if (index_ > 0 && preintegrated_->deltaTij() == 0) {
        return;
    }
    // similarly: skip creating X(0) until at least one accel sample has arrived, so gravity
    // alignment below has something to align to instead of silently falling back to identity.
    if (index_ == 0 && accelSampleCount_ == 0) {
        return;
    }

    // unlike the old batch version, these hold ONLY this keyframe's new factors/values -
    // isam2_ already remembers everything from earlier keyframes internally.
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues;

    if (index_ == 0) {
        // anchor the graph so it isn't underconstrained - placeholder noise sigmas, needs tuning
        auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
        auto velNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
        auto biasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);

        // Gravity-align the initial orientation instead of assuming identity (perfectly level).
        // A stationary accelerometer reads the reaction to gravity - roughly +9.81 along whichever
        // body axis is pointing "up". Rotating that measured direction onto the nav frame's up axis
        // (MakeSharedU -> gravity is -Z, so "up" is +Z) recovers roll/pitch; yaw stays unobservable
        // from gravity alone and is left at the arbitrary reference (0). Same technique Kimera-VIO's
        // InitializationFromImu uses for a static/near-static start.
        gtsam::Rot3 initialRot;
        if (accelSampleCount_ > 0) {
            gtsam::Vector3 measuredUp = accelAccum_ / accelSampleCount_;
            initialRot = rotationAligningVectors(measuredUp, gtsam::Vector3(0, 0, 1));
        }
        latestPose_ = gtsam::Pose3(initialRot, gtsam::Point3(0, 0, 0));

        newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), latestPose_, poseNoise));
        newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), latestVelocity_, velNoise));
        newFactors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), latestBias_, biasNoise));

        newValues.insert(X(0), latestPose_);
        newValues.insert(V(0), latestVelocity_);
        newValues.insert(B(0), latestBias_);
    } else {
        // CombinedImuFactor bundles the ImuFactor's pose/velocity relation AND the bias
        // random-walk relation (via preintegrationParams_'s biasAccCovariance/biasOmegaCovariance,
        // set in the constructor) into one factor over 6 keys - the bias evolves keyframe-to-
        // keyframe (fresh key B(index_)) instead of staying fixed at one shared B(0) forever, with
        // GTSAM handling the dt-scaling internally as part of its own unified covariance
        // propagation instead of a separately-scaled BetweenFactor competing for numerical
        // precision against the rest of the graph.
        gtsam::CombinedImuFactor imuFactor(
            X(index_ - 1), V(index_ - 1), X(index_), V(index_), B(index_ - 1), B(index_), *preintegrated_);
        newFactors.add(imuFactor);
        newValues.insert(B(index_), latestBias_);

        // initial guess for the new state: propagate through the IMU preintegration instead of
        // reusing the previous pose unchanged. Reusing the same pose gave every X(i) the exact
        // same initial value - zero baseline between "different" camera poses - which makes
        // SmartProjectionPoseFactor's triangulation degenerate (can't triangulate depth with no
        // parallax). predict() uses the real (now non-zero) IMU motion to give each new pose a
        // distinct, physically-motivated starting guess instead.
        gtsam::NavState prevState(latestPose_, latestVelocity_);
        gtsam::NavState predictedState = preintegrated_->predict(prevState, latestBias_);
        newValues.insert(X(index_), predictedState.pose());
        newValues.insert(V(index_), predictedState.velocity());
    }

    // visual observations: feed each tracked point into the smart factor for its feature id,
    // creating a new smart factor the first time an id is seen. Without these the graph is pure
    // IMU dead-reckoning with nothing to bound its drift, which over enough keyframes makes the
    // linearized system numerically singular (GTSAM throws IndeterminantLinearSystemException) -
    // these factors are what keep the pose estimate anchored to something in the world.
    constexpr bool kUseVisualFactors = true;
    gtsam::FactorIndices factorsToRemove;
    std::vector<int> touchedLandmarkIds;  // ids of smart factors appended to newFactors below, in order
    size_t leadingFactorCount = newFactors.size();  // priors (index_==0) or the ImuFactor (else) added above
    if (kUseVisualFactors) {
        // 1px was unrealistically tight for KLT tracking accuracy - an overly confident noise
        // model makes the optimizer chase every last pixel of (often just tracking-jitter)
        // residual, which is a big part of why poses were swinging wildly between keyframes.
        auto pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 3.0);
        for (size_t i = 0; i < features.points.size(); i++) {
            int id = features.ids[i];
            gtsam::Point2 measurement(features.points[i].x, features.points[i].y);

            auto it = smartFactors_.find(id);
            if (it == smartFactors_.end()) {
                auto factor = boost::make_shared<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>(
                    pixelNoise, K_, imuToCam_, smartFactorParams_);
                factor->add(measurement, X(index_));
                smartFactors_[id] = factor;
                newFactors.add(factor);
                touchedLandmarkIds.push_back(id);
            } else {
                // ISAM2 holds the exact shared_ptr we gave it for the old version of this factor
                // at a fixed slot - mutating it in place (it->second->add(...)) would corrupt that
                // live object out from under ISAM2 (its key list would gain X(index_) before
                // ISAM2's variable index was ever told about it), which is exactly what caused
                // "indices and factors ... not consistent with the existing variable index" here.
                // Copy first, mutate the copy, and swap it in - the old object ISAM2 still
                // references for removal stays untouched.
                auto updatedFactor = boost::make_shared<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>(*it->second);
                updatedFactor->add(measurement, X(index_));

                auto idxIt = smartFactorIndices_.find(id);
                if (idxIt != smartFactorIndices_.end()) {
                    factorsToRemove.push_back(idxIt->second);
                }
                smartFactors_[id] = updatedFactor;
                newFactors.add(updatedFactor);
                touchedLandmarkIds.push_back(id);
            }
        }
    }

    // incremental update: isam2_ folds these new factors/values into its existing Bayes tree
    // instead of re-solving everything from scratch, so cost per keyframe stays roughly constant
    // as the trajectory grows instead of growing with it.
    gtsam::ISAM2Result updateResult = isam2_.update(newFactors, newValues, factorsToRemove);

    // newFactorsIndices is 1-to-1 with newFactors (documented in ISAM2Result.h) - the smart-factor
    // entries are the suffix after the leading priors/ImuFactor, in the same order they were added.
    for (size_t i = 0; i < touchedLandmarkIds.size(); i++) {
        smartFactorIndices_[touchedLandmarkIds[i]] = updateResult.newFactorsIndices[leadingFactorCount + i];
    }

    gtsam::Values currentEstimate = isam2_.calculateEstimate();

    latestPose_ = currentEstimate.at<gtsam::Pose3>(X(index_));
    latestVelocity_ = currentEstimate.at<gtsam::Vector3>(V(index_));
    latestBias_ = currentEstimate.at<gtsam::imuBias::ConstantBias>(B(index_));

    preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preintegrationParams_, latestBias_);
    index_++;
}
