// Embedded HTTP + WebSocket server for the VisionSense dashboard.
//
// Runs inside node_dashboard. HTTP serves static files (HTML/CSS/JS/assets)
// from launch/dashboard/ in the install share dir; WebSocket /ws pushes the
// latest JPEG frame per stream plus small JSON telemetry messages to every
// connected client.
//
// Wire protocol:
//   Binary frame: byte 0 = stream id (uint8); bytes 1.. = JPEG payload.
//   Text frame:   JSON object with a "t" field (gps, imu, driver, fps,
//                 log, hello).
//
// Threading: libwebsockets event loop runs on its own thread. ROS callbacks
// call push_jpeg / push_json from arbitrary rclcpp executor threads; both are
// safe to call concurrently with the service loop.

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

struct lws;
struct lws_context;
struct lws_protocols;
struct lws_http_mount;

namespace visionsense {

enum StreamId : uint8_t {
    STREAM_FUSED   = 0x01,
    STREAM_DEPTH   = 0x02,
    STREAM_DRIVER  = 0x03,
    STREAM_BEV     = 0x04,
    STREAM_CAMERA  = 0x05,
    STREAM_LANES   = 0x06,
    STREAM_DETECT  = 0x07,
    STREAM_SIGNS   = 0x08,
    STREAM_COUNT   = 9,    // STREAM_FUSED..STREAM_SIGNS + 1 sentinel
};

struct ClientSession {
    struct lws *wsi = nullptr;
    // Latest JPEG per stream id (1..8). slot 0 unused.
    std::array<std::vector<uint8_t>, STREAM_COUNT> latest_jpeg;
    std::array<bool, STREAM_COUNT> jpeg_dirty{};
    // Pending JSON text messages (newest at back). Capped to 64; oldest drop.
    std::deque<std::string> json_queue;
    // Round-robin pointer so we don't starve later streams.
    uint8_t next_stream_to_send = STREAM_FUSED;
    // Accumulator for inbound text frames (lws can deliver them in chunks).
    std::string inbound_text;
};

class DashboardServer {
public:
    // Opaque per-client handle (pointer to ClientSession). Treated as void*
    // by the rest of the system so callers don't have to include this header.
    using ClientHandle = void *;

    // Called when a client sends a complete text WebSocket frame. The handler
    // can decide whether to do something with it (e.g. dispatch as WebRTC
    // signaling). Return value is currently unused but reserved.
    using TextHandler =
        std::function<bool(ClientHandle, const std::string &)>;

    using DisconnectHandler = std::function<void(ClientHandle)>;

    DashboardServer(int port, std::string mount_dir);
    ~DashboardServer();

    DashboardServer(const DashboardServer &) = delete;
    DashboardServer &operator=(const DashboardServer &) = delete;

    // Start the libwebsockets event loop on a background thread. Returns
    // true if the server bound to the configured port.
    bool start();

    // Stop and join the event-loop thread. Safe to call multiple times.
    void stop();

    // Thread-safe: replace the latest JPEG for a given stream and wake all
    // clients so the writable callback flushes the new frame.
    void push_jpeg(StreamId id, std::vector<uint8_t> jpeg);

    // Thread-safe: enqueue a JSON text message for every client.
    void push_json(std::string json);

    // Thread-safe: enqueue a JSON text message for a single specific client.
    // Silently no-ops if the client has disconnected.
    void push_json_to_client(ClientHandle client, std::string json);

    // Register hooks. Both are invoked from the lws service thread, so the
    // handlers must be quick or marshal to another thread themselves.
    void set_text_handler(TextHandler h);
    void set_disconnect_handler(DisconnectHandler h);

    // Called by the WS protocol callback in dashboard_server.cpp to bind
    // libwebsockets per-session storage to our state.
    void on_client_connected(struct lws *wsi, ClientSession *session);
    void on_client_disconnected(ClientSession *session);
    // Returns 0 on success, -1 to close the connection.
    int on_client_writeable(ClientSession *session);
    // Receive a (possibly fragmented) text frame chunk. The accumulated
    // buffer is delivered to the registered TextHandler when the final
    // fragment arrives.
    void on_client_text(ClientSession *session, const char *data, size_t len,
                        bool is_final);

private:
    void run_loop();
    void send_hello(ClientSession *session);

    int port_;
    std::string mount_dir_;
    std::atomic<bool> running_{false};
    std::thread thread_;

    struct lws_context *ctx_ = nullptr;
    // Owned arrays; pointed to by lws_context_creation_info.
    std::unique_ptr<lws_protocols[]> protocols_;
    std::unique_ptr<lws_http_mount> mount_;

    std::mutex clients_mutex_;
    std::set<ClientSession *> clients_;

    TextHandler text_handler_;
    DisconnectHandler disconnect_handler_;
};

}  // namespace visionsense
