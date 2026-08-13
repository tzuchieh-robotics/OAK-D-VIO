#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

struct TrackedFeatures {
    std::vector<cv::Point2f> points;
    std::vector<int> ids;  // stable per-feature id, persists across frames while tracked
};

class VisualFrontend {
public:
    TrackedFeatures track(const cv::Mat& frame);

private:
    void detectNewFeatures(const cv::Mat& gray, int numToAdd);

    cv::Mat prevGray_;
    std::vector<cv::Point2f> prevPoints_;
    std::vector<int> prevIds_;
    int nextId_ = 0;

    static constexpr int kMaxFeatures = 150;
    static constexpr int kMinFeatures = 80;
};

void VisualFrontend::detectNewFeatures(const cv::Mat& gray, int numToAdd) {
    if (numToAdd <= 0) return;

    // mask out a radius around existing points so new detections don't cluster on top of them
    cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar(255));
    for (const auto& p : prevPoints_) {
        cv::circle(mask, p, 15, cv::Scalar(0), -1);
    }

    std::vector<cv::Point2f> newPoints;
    cv::goodFeaturesToTrack(gray, newPoints, numToAdd, 0.01, 10, mask);

    for (const auto& p : newPoints) {
        prevPoints_.push_back(p);
        prevIds_.push_back(nextId_++);
    }
}

TrackedFeatures VisualFrontend::track(const cv::Mat& frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (prevGray_.empty()) {
        detectNewFeatures(gray, kMaxFeatures);
        prevGray_ = gray;
        return TrackedFeatures{prevPoints_, prevIds_};
    }

    // track previous points into the new frame
    std::vector<cv::Point2f> tracked;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prevGray_, gray, prevPoints_, tracked, status, err);

    // keep only points that tracked successfully and stayed inside the frame
    std::vector<cv::Point2f> prevOk, trackedOk;
    std::vector<int> idsOk;
    for (size_t i = 0; i < tracked.size(); i++) {
        if (!status[i]) continue;
        const auto& p = tracked[i];
        // NaN comparisons are always false, so a plain `p.x < 0 || p.x >= gray.cols` check
        // silently lets a NaN-coordinate point through (calcOpticalFlowPyrLK can occasionally
        // report one as "tracked" even when status[i]==1). A NaN point later reaching
        // cv::circle() in detectNewFeatures() crashes OpenCV with a Mat row-range assertion,
        // since NaN cast to int is undefined behavior. Reject non-finite coordinates explicitly.
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
        if (p.x < 0 || p.y < 0 || p.x >= gray.cols || p.y >= gray.rows) continue;
        prevOk.push_back(prevPoints_[i]);
        trackedOk.push_back(p);
        idsOk.push_back(prevIds_[i]);
    }

    // RANSAC outlier rejection: drop tracks that are geometrically inconsistent
    // between the two views even though optical flow numerically "succeeded"
    std::vector<cv::Point2f> finalPoints;
    std::vector<int> finalIds;
    if (prevOk.size() >= 8) {
        std::vector<uchar> inlierMask;
        cv::findFundamentalMat(prevOk, trackedOk, cv::FM_RANSAC, 1.0, 0.99, inlierMask);
        for (size_t i = 0; i < trackedOk.size(); i++) {
            if (!inlierMask[i]) continue;
            finalPoints.push_back(trackedOk[i]);
            finalIds.push_back(idsOk[i]);
        }
    } else {
        finalPoints = trackedOk;
        finalIds = idsOk;
    }

    prevPoints_ = finalPoints;
    prevIds_ = finalIds;

    // replenish if the tracked count dropped too low
    if (static_cast<int>(prevPoints_.size()) < kMinFeatures) {
        detectNewFeatures(gray, kMaxFeatures - static_cast<int>(prevPoints_.size()));
    }

    prevGray_ = gray;
    return TrackedFeatures{prevPoints_, prevIds_};
}
