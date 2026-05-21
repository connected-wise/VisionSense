// WebRTC video streamer for the VisionSense dashboard.
//
// Per (client, stream) it builds a GStreamer pipeline:
//
//   appsrc(BGR) → videoconvert → nvvidconv (NVMM,NV12) → nvv4l2h264enc (NVENC)
//                → h264parse → rtph264pay → webrtcbin
//
// Signaling (SDP + ICE) is exchanged over the existing libwebsockets text
// channel as JSON: webrtc-request / webrtc-offer / webrtc-answer /
// webrtc-ice / webrtc-stop. WebRTCManager is signaling-transport-agnostic —
// node_dashboard plugs DashboardServer in via SignalingSender.
//
// All public methods are thread-safe: push_frame, on_signaling_message, and
// on_client_disconnected may be invoked from arbitrary rclcpp executor /
// lws service threads. Internally everything that touches GStreamer is
// marshalled onto the GMainLoop thread via g_idle_add.

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/core/mat.hpp>

typedef struct _GstElement GstElement;
typedef struct _GstPromise GstPromise;
typedef struct _GMainLoop GMainLoop;
typedef struct _GMainContext GMainContext;

namespace visionsense {

enum StreamId : uint8_t;  // forward-decl from dashboard_server.h

// Opaque per-client identifier. In practice this is the
// DashboardServer::ClientSession* cast to void*. WebRTCManager never
// dereferences it; it just uses it as a map key + passes it back to the
// SignalingSender so DashboardServer can route the response.
using ClientHandle = void *;

// Caps for each stream — must match what node_dashboard's callbacks
// produce after resize. Streams with width=0 are not WebRTC-eligible
// (variable size) and the manager silently rejects requests for them.
//
// `iframeinterval` is in encoded frames. NVENC injects an IDR keyframe every
// N input buffers; long intervals mean any decoder desync on a low-rate
// stream waits seconds before re-syncing. Tune so each stream hits a
// keyframe roughly every 0.5 s at its expected source rate.
struct StreamCaps {
    int width;
    int height;
    int fps;
    int iframeinterval;
};

const StreamCaps &caps_for(StreamId id);

class WebRTCManager {
public:
    // Called whenever WebRTCManager needs to send a JSON text message to a
    // specific client. Must be thread-safe; will be invoked from the GMainLoop
    // thread.
    using SignalingSender =
        std::function<void(ClientHandle client, const std::string &json)>;

    explicit WebRTCManager(SignalingSender sender);
    ~WebRTCManager();

    WebRTCManager(const WebRTCManager &) = delete;
    WebRTCManager &operator=(const WebRTCManager &) = delete;

    // Initialize GStreamer and start the GMainLoop thread. Returns false if
    // a required element is missing.
    bool start();
    void stop();

    // Hand a fresh BGR frame to all active pipelines for this stream.
    // Caller retains ownership of `frame`; we copy/wrap as needed. The frame
    // dimensions must match caps_for(stream).
    void push_frame(StreamId stream, const cv::Mat &frame_bgr);

    // Handle an inbound signaling JSON message from a client. Returns true if
    // the message was a WebRTC message (handled), false if not (caller may
    // treat as some other type).
    bool on_signaling_message(ClientHandle client, const std::string &json);

    // Called when a client's WebSocket disconnects — tears down all pipelines
    // for that client.
    void on_client_disconnected(ClientHandle client);

    // Returns true if at least one pipeline for stream is currently active
    // across all clients (used by node_dashboard to skip frame work when no
    // one is watching).
    bool has_active_subscribers(StreamId stream);

private:
    struct PipelineCtx;
    using PipelineKey = std::pair<ClientHandle, int>;  // (client, stream)

    // Mainloop-thread entry points.
    void thread_main();
    void mainloop_create_pipeline(ClientHandle client, StreamId stream);
    void mainloop_apply_answer(ClientHandle client, StreamId stream,
                                std::string sdp);
    void mainloop_add_ice(ClientHandle client, StreamId stream,
                          unsigned int mline_index, std::string candidate);
    void mainloop_destroy_pipeline(ClientHandle client, StreamId stream);
    void mainloop_destroy_client_pipelines(ClientHandle client);

    // GStreamer signal handlers (static trampolines, dispatch to PipelineCtx).
    static void s_on_negotiation_needed(GstElement *webrtcbin,
                                         void *user_data);
    static void s_on_ice_candidate(GstElement *webrtcbin,
                                    unsigned int mline_index,
                                    char *candidate,
                                    void *user_data);
    static void s_on_offer_created(GstPromise *promise, void *user_data);

    // Helper: send a JSON text message to a specific client.
    void send_signaling(ClientHandle client, const std::string &json);

    // Lookup helpers (caller must hold pipelines_mutex_).
    PipelineCtx *find_locked(ClientHandle client, StreamId stream);

    SignalingSender sender_;
    std::atomic<bool> running_{false};
    GMainLoop *loop_ = nullptr;
    GMainContext *loop_ctx_ = nullptr;
    std::thread thread_;

    std::mutex pipelines_mutex_;
    std::map<PipelineKey, std::unique_ptr<PipelineCtx>> pipelines_;
};

}  // namespace visionsense
