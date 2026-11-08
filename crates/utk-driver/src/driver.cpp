// OpenVR driver interface layer, plus the glue to HTC's direct tracker SDK.
//
// - The SDK runs its tracker server inside vrserver and calls ITrackerSubscriber for every pose.
//   The callbacks are forwarded to the Rust layer (lib.rs), and each pose is pushed to SteamVR
//   right away instead of waiting for the next frame.
// - A supervisor thread starts the SDK only while VIVE Hub is closed (both want the dongle) and
//   reports the server state to Rust, which writes it to the status file for the in-VR panel.

#include <openvr_driver.h>

#include "../sdk/ultimate_tracker.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace vr;

extern "C" {
// Must match UtkPose in lib.rs. Quaternions are (w, x, y, z), like HmdQuaternion_t.
struct UtkPose {
    bool connected;
    bool valid;
    double age_s;          // how old the sample is (seconds); poseTimeOffset is its negation
    double pos[3];         // tracker position in the map frame (= driver frame), OpenVR axes
    double rot[4];
    double vel[3];
    double ang_vel[3];     // map frame
    double world_rot[4];   // calibration: map frame -> SteamVR raw frame
    double world_pos[3];
};

// Must match UtkSdkSample in lib.rs.
struct UtkSdkSample {
    float location[3];
    float rotation[4];
    float angular_velocity[3];
    int64_t timestamp_us;
    int64_t sdk_now_us;
    uint32_t id;
    uint32_t flags;
    uint8_t state;
    uint8_t battery;
};

typedef void (*UtkLogFn)(const char *msg);
bool utk_init(UtkLogFn log);
void utk_shutdown();
void utk_set_server_state(int state);  // 0 starting, 1 running, 2 VIVE Hub running, 3 start failed, 4 stopped
bool utk_vive_hub_running();
void utk_sdk_add(uint32_t id, const char *name);
void utk_sdk_remove(uint32_t id);
void utk_sdk_update(const UtkSdkSample *sample);
void utk_sdk_click(uint32_t id, int button, int clicks);
// Next tracker serial waiting to be registered; false if there is none.
bool utk_next_new_device(char *serial, size_t len);
bool utk_get_pose(const char *serial, UtkPose *out);
}

namespace {

void log_line(const char *msg) {
    if (VRDriverLog()) VRDriverLog()->Log(msg);
}

void logf(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    log_line(buf);
}

HmdQuaternion_t quat(const double q[4]) { return {q[0], q[1], q[2], q[3]}; }

class Tracker final : public ITrackedDeviceServerDriver {
public:
    explicit Tracker(std::string serial) : serial_(std::move(serial)) {}

    const std::string &serial() const { return serial_; }
    TrackedDeviceIndex_t index() const { return index_.load(); }

    EVRInitError Activate(uint32_t index) override {
        auto props = VRProperties();
        auto c = props->TrackedDeviceToPropertyContainer(index);
        props->SetStringProperty(c, Prop_TrackingSystemName_String, "utk");
        props->SetStringProperty(c, Prop_ManufacturerName_String, "HTC");
        props->SetStringProperty(c, Prop_ModelNumber_String, "VIVE Ultimate Tracker");
        props->SetStringProperty(c, Prop_SerialNumber_String, serial_.c_str());
        // Vive Tracker 3.0 model (~9 cm, close to the UTK); generic_tracker is a plain placeholder.
        props->SetStringProperty(c, Prop_RenderModelName_String, "{htc}vr_tracker_vive_3_0");
        // Reuse SteamVR's own Vive tracker input profile so "Manage Trackers" can assign roles.
        props->SetStringProperty(c, Prop_ControllerType_String, "vive_tracker");
        props->SetStringProperty(c, Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
        props->SetInt32Property(c, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
        props->SetBoolProperty(c, Prop_WillDriftInYaw_Bool, false);
        props->SetBoolProperty(c, Prop_DeviceIsWireless_Bool, true);
        props->SetBoolProperty(c, Prop_DeviceProvidesBatteryStatus_Bool, false);
        index_.store(index);
        logf("activated tracker %s as device %u", serial_.c_str(), index);
        return VRInitError_None;
    }

    void Deactivate() override { index_.store(k_unTrackedDeviceIndexInvalid); }
    void EnterStandby() override {}
    void *GetComponent(const char *) override { return nullptr; }

    void DebugRequest(const char *, char *response, uint32_t size) override {
        if (size) response[0] = 0;
    }

    DriverPose_t GetPose() override {
        DriverPose_t pose = {};
        pose.qWorldFromDriverRotation = {1, 0, 0, 0};
        pose.qDriverFromHeadRotation = {1, 0, 0, 0};
        pose.qRotation = {1, 0, 0, 0};
        UtkPose p;
        if (!utk_get_pose(serial_.c_str(), &p)) {
            pose.deviceIsConnected = false;
            pose.poseIsValid = false;
            pose.result = TrackingResult_Uninitialized;
            return pose;
        }
        pose.poseTimeOffset = -p.age_s;
        pose.qWorldFromDriverRotation = quat(p.world_rot);
        for (int i = 0; i < 3; i++) {
            pose.vecWorldFromDriverTranslation[i] = p.world_pos[i];
            pose.vecPosition[i] = p.pos[i];
            pose.vecVelocity[i] = p.vel[i];
            pose.vecAngularVelocity[i] = p.ang_vel[i];
        }
        pose.qRotation = quat(p.rot);
        pose.deviceIsConnected = p.connected;
        pose.poseIsValid = p.valid;
        pose.result = p.valid ? TrackingResult_Running_OK : TrackingResult_Running_OutOfRange;
        return pose;
    }

    void SubmitPose() {
        TrackedDeviceIndex_t i = index();
        if (i == k_unTrackedDeviceIndexInvalid) return;
        DriverPose_t pose = GetPose();
        VRServerDriverHost()->TrackedDevicePoseUpdated(i, pose, sizeof(pose));
    }

private:
    std::string serial_;
    std::atomic<TrackedDeviceIndex_t> index_{k_unTrackedDeviceIndexInvalid};
};

// Trackers by SDK id, so the SDK thread can push a pose without touching the provider's list.
constexpr size_t kMaxIds = 16;
std::array<std::atomic<Tracker *>, kMaxIds> g_by_id{};
std::mutex g_names_mu;
std::array<std::string, kMaxIds> g_names;  // SDK id -> serial, for late registration

class Subscriber final : public vive::ITrackerSubscriber {
public:
    void ServerStart() override { log_line("SDK server started"); }
    void ServerStop() override { log_line("SDK server stopped"); }
    void Add(uint32_t id, char const *name) override {
        if (id < kMaxIds) {
            std::lock_guard g(g_names_mu);
            g_names[id] = name ? name : "";
        }
        utk_sdk_add(id, name);
    }
    void Remove(uint32_t id) override { utk_sdk_remove(id); }
    void Click(uint32_t id, int button, int clicks) override { utk_sdk_click(id, button, clicks); }
    void CommandResponse(vive::TrackerCommandResponse const &r) override {
        logf("SDK command %s -> %s", r.Command ? r.Command : "", r.Result ? r.Result : "");
    }
    // SDK thread, ~125 Hz per tracker: hand the pose over and push it to SteamVR immediately.
    void Update(vive::TrackerData const &d) override {
        UtkSdkSample s{};
        std::memcpy(s.location, d.Location, sizeof s.location);
        std::memcpy(s.rotation, d.Rotation, sizeof s.rotation);
        std::memcpy(s.angular_velocity, d.AngularVelocity, sizeof s.angular_velocity);
        s.timestamp_us = d.Timestamp_us;
        s.sdk_now_us = vive::GetTimestamp();
        s.id = d.Id;
        s.flags = d.Flags;
        s.state = d.State;
        s.battery = d.Battery;
        utk_sdk_update(&s);
        if (d.Id < kMaxIds) {
            if (Tracker *t = g_by_id[d.Id].load()) t->SubmitPose();
        }
    }
};

class Provider final : public IServerTrackedDeviceProvider {
public:
    EVRInitError Init(IVRDriverContext *ctx) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
        // steamvr/activateMultipleDrivers must already be on, or SteamVR never loads this driver
        // next to vrlink; scripts/install-driver.sh sets it.
        if (!utk_init(log_line)) return VRInitError_Driver_Failed;
        logf("driver loaded, tracker SDK %s", vive::GetVersion());
        subscriber_ = std::make_unique<Subscriber>();  // must exist before StartTrackerServer
        stop_ = false;
        supervisor_ = std::thread([this] { Supervise(); });
        return VRInitError_None;
    }

    void Cleanup() override {
        {
            std::lock_guard g(mu_);
            stop_ = true;
        }
        cv_.notify_all();
        if (supervisor_.joinable()) supervisor_.join();
        for (auto &p : g_by_id) p.store(nullptr);
        utk_shutdown();
        trackers_.clear();
        subscriber_.reset();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }

    const char *const *GetInterfaceVersions() override { return k_InterfaceVersions; }

    void RunFrame() override {
        char serial[64];
        while (utk_next_new_device(serial, sizeof(serial))) {
            auto t = std::make_unique<Tracker>(serial);
            if (VRServerDriverHost()->TrackedDeviceAdded(serial, TrackedDeviceClass_GenericTracker, t.get())) {
                trackers_.push_back(std::move(t));
            } else {
                logf("failed to add tracker %s", serial);
            }
        }
        // Map SDK ids to activated trackers for the SDK thread's immediate pushes.
        {
            std::lock_guard g(g_names_mu);
            for (size_t id = 0; id < kMaxIds; id++) {
                Tracker *found = nullptr;
                for (auto &t : trackers_) {
                    if (!g_names[id].empty() && t->serial() == g_names[id] && t->index() != k_unTrackedDeviceIndexInvalid) {
                        found = t.get();
                    }
                }
                g_by_id[id].store(found);
            }
        }
        // Also once per frame: keeps validity current when updates stop (tracker lost, VIVE Hub).
        for (auto &t : trackers_) t->SubmitPose();
        VREvent_t e;
        while (VRServerDriverHost()->PollNextEvent(&e, sizeof(e))) {
        }
    }

    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}

private:
    // Starts the SDK server once VIVE Hub is closed; stops it on shutdown.
    void Supervise() {
        bool running = false;
        bool warned = false;
        std::unique_lock lock(mu_);
        while (!stop_) {
            if (!running) {
                if (utk_vive_hub_running()) {
                    utk_set_server_state(2);
                    if (!warned) log_line("VIVE Hub is running and owns the dongle; close it to use the trackers");
                    warned = true;
                } else {
                    utk_set_server_state(0);
                    lock.unlock();
                    running = vive::StartTrackerServer();
                    lock.lock();
                    utk_set_server_state(running ? 1 : 3);
                    warned = false;
                }
            }
            cv_.wait_for(lock, std::chrono::seconds(2), [this] { return stop_; });
        }
        lock.unlock();
        if (running) vive::StopTrackerServer();
        utk_set_server_state(4);
    }

    std::vector<std::unique_ptr<Tracker>> trackers_;
    std::unique_ptr<Subscriber> subscriber_;
    std::thread supervisor_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool stop_ = false;
};

Provider g_provider;

}  // namespace

// Called by the HmdDriverFactory exported from lib.rs.
extern "C" void *utk_cpp_driver_factory(const char *name, int *ret) {
    if (std::strcmp(name, IServerTrackedDeviceProvider_Version) == 0) return &g_provider;
    if (ret) *ret = VRInitError_Init_InterfaceNotFound;
    return nullptr;
}
