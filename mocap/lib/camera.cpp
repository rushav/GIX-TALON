#include "camera.h"

#include <cameralibrary.h>
#include <chrono>
#include <stdexcept>
#include <thread>

using namespace CameraLibrary;

namespace mocap {

// ---------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------

std::vector<CameraInfo> discover() {
    CameraManager::X().WaitForInitialization();

    // The SDK signals "ready" when the FIRST device initializes, not all
    // of them, so trusting it yields an incomplete list. Poll until the
    // count holds steady instead. Cold enumeration takes 4-8 seconds.
    size_t previous = 0;
    int stable = 0;
    for (int i = 0; i < 40 && stable < 6; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        CameraList list;
        if (list.Count() == previous && list.Count() > 0) ++stable;
        else { stable = 0; previous = list.Count(); }
    }

    // The list can report the same serial more than once. Collapse it so
    // each physical camera appears once.
    std::vector<CameraInfo> found;
    CameraList list;
    for (int i = 0; i < list.Count(); ++i) {
        int serial = list[i].Serial();
        bool seen = false;
        for (auto& c : found) if (c.serial == serial) seen = true;
        if (seen) continue;

        found.push_back({serial,
                         list[i].Revision(),
                         list[i].Name(),
                         list[i].State() == Initialized});
    }
    return found;
}

// ---------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------

struct Camera::Impl {
    std::shared_ptr<CameraLibrary::Camera> cam;
    Settings requested;
};

Camera::Camera(int serial)
    : impl_(std::make_unique<Impl>()), serial_(serial) {

    discover();   // ensure enumeration has completed

    CameraList list;
    for (int i = 0; i < list.Count(); ++i) {
        if (list[i].Serial() == serial) {
            impl_->cam = CameraManager::X().GetCamera(list[i].UID());
            break;
        }
    }
    if (!impl_->cam)
        throw std::runtime_error("camera " + std::to_string(serial) + " not found");
}

Camera::~Camera() = default;

void Camera::start(const Settings& s) {
    impl_->cam->SetVideoType(Core::GrayscaleMode);
    impl_->cam->Start();

    // SetVideoType is asynchronous: the camera reports the new mode at
    // once, but frames already queued still carry the old one. Reassert
    // after Start(); grab() verifies each frame regardless.
    impl_->cam->SetVideoType(Core::GrayscaleMode);

    apply(s);
}

void Camera::apply(const Settings& s) {
    // Frame rate first: it determines the exposure ceiling, so setting
    // exposure before the rate can leave it silently clamped.
    impl_->cam->SetFrameRate(s.frameRate);

    impl_->cam->SetExposure(s.exposure);
    impl_->cam->SetIntensity(s.intensity);
    impl_->cam->SetImagerGain(static_cast<eImagerGain>(s.gain));

    if (impl_->cam->IsFilterSwitchAvailable())
        impl_->cam->SetIRFilter(s.irFilter);

    impl_->requested = s;
}

cv::Mat Camera::grab() {
    // NextFrame() returns the OLDEST queued frame. One call per loop and
    // you fall progressively behind real time, so drain to the newest.
    // Reassignment drops the previous reference, freeing that buffer.
    std::shared_ptr<const Frame> frame;
    while (auto f = impl_->cam->NextFrame()) frame = f;
    if (!frame) return {};

    // Stale frames from before the mode change are still object-mode;
    // GrayscaleData() on one returns null.
    if (frame->FrameType() != Core::GrayscaleMode) return {};

    const unsigned char* pixels = frame->GrayscaleData(*impl_->cam);
    if (!pixels) return {};

    int w = width(), h = height();
    if (w != 1280 || h != 1024)
        throw std::runtime_error("unexpected frame dimensions " +
                                 std::to_string(w) + "x" + std::to_string(h));

    // Copy rather than wrap. Wrapping points at SDK memory that dies
    // with the frame, which is a use-after-free waiting to happen.
    // 1.3 MB at 8 fps is free.
    return cv::Mat(h, w, CV_8UC1, (void*)pixels).clone();
}

// PhysicalPixelWidth/Height are pixel counts. ImagerWidth/Height are
// doubles - sensor size in mm. Width/Height describe an ROI crop.
int Camera::width()  const { return impl_->cam->PhysicalPixelWidth(); }
int Camera::height() const { return impl_->cam->PhysicalPixelHeight(); }

std::string Camera::name() const { return impl_->cam->Name(); }

Settings Camera::actual() const {
    Settings s;
    s.exposure  = impl_->cam->Exposure();
    s.gain      = static_cast<int>(impl_->cam->ImagerGain());
    s.intensity = impl_->cam->Intensity();
    s.frameRate = impl_->cam->FrameRate();
    s.irFilter  = impl_->requested.irFilter;
    return s;
}

int Camera::maxExposure() const {
    // Measured on 33661 across four rates: the ceiling is exactly
    // floor(1e6/fps) - 200. The SDK does re-report it, so ask rather
    // than caching - a value read before a rate change is wrong.
    return impl_->cam->MaximumExposureValue();
}

}  // namespace mocap