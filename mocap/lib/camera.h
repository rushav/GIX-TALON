// Acquisition from a single OptiTrack camera.
//
// Wraps the SDK behaviors that are easy to get wrong and never optional:
// staggered discovery, asynchronous mode changes, and the frame queue
// returning oldest-first. Callers get a validated grayscale image or
// nothing.

#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace mocap {

struct CameraInfo {
    int serial;
    int revision;
    std::string name;
    bool initialized;
};

// Find every camera on the network. Blocks until the device count
// stops growing, because the SDK reports "ready" after the FIRST
// device initializes, not all of them.
std::vector<CameraInfo> discover();

struct Settings {
    int exposure  = 8133;
    int gain      = 7;       // 0-7
    int intensity = 15;      // IR ring, 0-15
    int frameRate = 30;
    bool irFilter = true;    // 850 nm bandpass in place
};

class Camera {
public:
    // Opens by serial. Throws std::runtime_error if not found.
    explicit Camera(int serial);
    ~Camera();

    void start(const Settings& s);
    void apply(const Settings& s);     // change settings while streaming

    // Newest available grayscale frame, or an empty Mat if none is
    // ready. The Mat is a COPY - safe to hold.
    cv::Mat grab();

    int width()  const;
    int height() const;
    int serial() const { return serial_; }
    std::string name() const;

    // What the camera reports, which may differ from what was asked.
    Settings actual() const;

    // Exposure ceiling is frame-period-derived: floor(1e6/fps) - 200.
    // Re-read after any frame rate change; a cached value is a lie.
    int maxExposure() const;

private:
    struct Impl;                       // SDK types stay out of this header
    std::unique_ptr<Impl> impl_;
    int serial_;
};

}  // namespace mocap