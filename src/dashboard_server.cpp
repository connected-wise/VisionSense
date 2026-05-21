#include "dashboard_server.h"

#include <libwebsockets.h>

#include <algorithm>
#include <cstring>

namespace visionsense {

namespace {

// Cap on the JSON queue per client. Telemetry is small; if a client is so
// slow it backs up more than this many messages, drop the oldest.
constexpr size_t kJsonQueueCap = 64;

// Extra mime types not in lws's built-in table.
const struct lws_protocol_vhost_options kMimeWebp = {
    nullptr, nullptr, ".webp", "image/webp"
};
const struct lws_protocol_vhost_options kMimeSvg = {
    &kMimeWebp, nullptr, ".svg", "image/svg+xml"
};

// Bridge: lws hands us the DashboardServer via vhost user-data.
DashboardServer *server_from(struct lws *wsi) {
    return static_cast<DashboardServer *>(
        lws_context_user(lws_get_context(wsi)));
}

// File-scope C-style callbacks with the exact libwebsockets signature.
// Kept out of the header so we don't need to forward-declare
// `enum lws_callback_reasons` there.
int http_cb(struct lws *wsi, enum lws_callback_reasons reason,
            void *user, void *in, size_t len) {
    return lws_callback_http_dummy(wsi, reason, user, in, len);
}

int ws_cb(struct lws *wsi, enum lws_callback_reasons reason,
          void *user, void *in, size_t len) {
    (void)in;
    (void)len;
    auto *session = static_cast<ClientSession *>(user);
    auto *server = server_from(wsi);
    if (!server) return 0;

    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED: {
            new (session) ClientSession();  // placement-new
            server->on_client_connected(wsi, session);
            return 0;
        }
        case LWS_CALLBACK_CLOSED: {
            server->on_client_disconnected(session);
            session->~ClientSession();
            return 0;
        }
        case LWS_CALLBACK_SERVER_WRITEABLE: {
            return server->on_client_writeable(session);
        }
        case LWS_CALLBACK_RECEIVE: {
            // Browser → server. Pings/pongs/control frames are handled by lws
            // automatically; this fires for text/binary payloads only. We
            // care about text (JSON signaling for WebRTC); binary from the
            // client is ignored.
            if (lws_frame_is_binary(wsi)) return 0;
            bool is_final = (lws_is_final_fragment(wsi) != 0);
            server->on_client_text(session,
                                   static_cast<const char *>(in), len,
                                   is_final);
            return 0;
        }
        default:
            return 0;
    }
}

}  // namespace

DashboardServer::DashboardServer(int port, std::string mount_dir)
    : port_(port), mount_dir_(std::move(mount_dir)) {}

DashboardServer::~DashboardServer() {
    stop();
}

bool DashboardServer::start() {
    if (running_.exchange(true)) {
        return true;  // already started
    }

    // Protocols: HTTP (mount-driven file serving) + WS "vc-ws".
    // lws expects a sentinel zero-init entry at the end.
    protocols_.reset(new lws_protocols[3]);
    std::memset(protocols_.get(), 0, sizeof(lws_protocols) * 3);

    protocols_[0].name = "http";
    protocols_[0].callback = &http_cb;
    protocols_[0].per_session_data_size = 0;
    protocols_[0].rx_buffer_size = 0;

    protocols_[1].name = "vc-ws";
    protocols_[1].callback = &ws_cb;
    protocols_[1].per_session_data_size = sizeof(ClientSession);
    protocols_[1].rx_buffer_size = 4096;
    protocols_[1].tx_packet_size = 0;  // no fragmentation hint

    // Mount the dashboard share dir at "/".
    mount_.reset(new lws_http_mount{});
    std::memset(mount_.get(), 0, sizeof(*mount_));
    mount_->mountpoint = "/";
    mount_->mountpoint_len = 1;
    mount_->origin = mount_dir_.c_str();
    mount_->def = "dashboard.html";
    mount_->origin_protocol = LWSMPRO_FILE;
    mount_->extra_mimetypes = &kMimeSvg;

    struct lws_context_creation_info info{};
    info.port = port_;
    info.iface = nullptr;
    info.protocols = protocols_.get();
    info.mounts = mount_.get();
    info.user = this;
    // Intentionally NOT setting LWS_SERVER_OPTION_HTTP_HEADERS_SECURITY_BEST_PRACTICES_ENFORCE.
    // That option adds a strict Content-Security-Policy header
    //   "script-src 'self'; style-src 'self'; ..."
    // which blocks the inline <script> and <style> blocks in dashboard.html.
    // Symptom: file:// works (no CSP) but http://localhost:8080/ is blank
    // because the browser refuses to execute any of the page's JavaScript,
    // including the WebRTC negotiation code. This server only listens on
    // localhost/LAN for an embedded dashboard — the strict CSP buys no
    // meaningful protection in that threat model.
    info.options = LWS_SERVER_OPTION_VALIDATE_UTF8;
    info.ka_time = 30;
    info.ka_probes = 3;
    info.ka_interval = 5;

    // Quiet libwebsockets' own logging in production. Keep ERR/WARN.
    lws_set_log_level(LLL_ERR | LLL_WARN, nullptr);

    ctx_ = lws_create_context(&info);
    if (!ctx_) {
        running_ = false;
        return false;
    }

    thread_ = std::thread([this] { run_loop(); });
    return true;
}

void DashboardServer::stop() {
    if (!running_.exchange(false)) return;
    if (ctx_) {
        // Wake the poll so service returns promptly.
        lws_cancel_service(ctx_);
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    if (ctx_) {
        lws_context_destroy(ctx_);
        ctx_ = nullptr;
    }
}

void DashboardServer::run_loop() {
    while (running_.load()) {
        // Tight poll — lws_cancel_service() also unblocks immediately on push,
        // but a short fallback keeps the loop responsive if the cancel
        // mechanism ever misses an edge.
        lws_service(ctx_, 5);
    }
}

void DashboardServer::push_jpeg(StreamId id, std::vector<uint8_t> jpeg) {
    if (id < STREAM_FUSED || id >= STREAM_COUNT) return;
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (clients_.empty()) return;
    for (auto *client : clients_) {
        client->latest_jpeg[id] = jpeg;  // copy per client; cheap (≤25 KB)
        client->jpeg_dirty[id] = true;
        lws_callback_on_writable(client->wsi);
    }
    if (ctx_) lws_cancel_service(ctx_);
}

void DashboardServer::push_json(std::string json) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (clients_.empty()) return;
    for (auto *client : clients_) {
        if (client->json_queue.size() >= kJsonQueueCap) {
            client->json_queue.pop_front();
        }
        client->json_queue.push_back(json);
        lws_callback_on_writable(client->wsi);
    }
    if (ctx_) lws_cancel_service(ctx_);
}

void DashboardServer::push_json_to_client(ClientHandle handle,
                                           std::string json) {
    auto *client = static_cast<ClientSession *>(handle);
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (clients_.find(client) == clients_.end()) return;  // gone
    if (client->json_queue.size() >= kJsonQueueCap) {
        client->json_queue.pop_front();
    }
    client->json_queue.push_back(std::move(json));
    lws_callback_on_writable(client->wsi);
    if (ctx_) lws_cancel_service(ctx_);
}

void DashboardServer::set_text_handler(TextHandler h) {
    text_handler_ = std::move(h);
}

void DashboardServer::set_disconnect_handler(DisconnectHandler h) {
    disconnect_handler_ = std::move(h);
}

void DashboardServer::on_client_connected(struct lws *wsi, ClientSession *session) {
    session->wsi = wsi;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.insert(session);
    }
    send_hello(session);
}

void DashboardServer::on_client_disconnected(ClientSession *session) {
    // Notify hooks BEFORE we forget about the client so handlers can use the
    // handle as a key one last time.
    if (disconnect_handler_) {
        disconnect_handler_(static_cast<ClientHandle>(session));
    }
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(session);
}

void DashboardServer::on_client_text(ClientSession *session,
                                      const char *data, size_t len,
                                      bool is_final) {
    if (data && len > 0) {
        session->inbound_text.append(data, len);
    }
    if (!is_final) return;
    if (text_handler_ && !session->inbound_text.empty()) {
        text_handler_(static_cast<ClientHandle>(session),
                      session->inbound_text);
    }
    session->inbound_text.clear();
}

void DashboardServer::send_hello(ClientSession *session) {
    // Sent on connect so the client knows the stream-id → name mapping.
    // Tiny string, no JSON escaping needed.
    static const char *hello =
        "{\"t\":\"hello\",\"streams\":["
        "{\"id\":1,\"name\":\"fused\"},"
        "{\"id\":2,\"name\":\"depth\"},"
        "{\"id\":3,\"name\":\"driver\"},"
        "{\"id\":4,\"name\":\"bev\"},"
        "{\"id\":5,\"name\":\"camera\"},"
        "{\"id\":6,\"name\":\"lanes\"},"
        "{\"id\":7,\"name\":\"detect\"},"
        "{\"id\":8,\"name\":\"signs\"}]}";
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        session->json_queue.push_back(hello);
        lws_callback_on_writable(session->wsi);
    }
}

int DashboardServer::on_client_writeable(ClientSession *session) {
    // Drain pending frames as long as the TCP pipe accepts more. The old
    // one-frame-per-callback design forced every frame through a full lws
    // event-loop round-trip; on a fast LAN that's pure latency. We bound the
    // per-tick batch so a slow consumer can't starve other clients.
    constexpr int kMaxBatch = 16;
    for (int sent = 0; sent < kMaxBatch; ++sent) {
        if (lws_send_pipe_choked(session->wsi)) {
            lws_callback_on_writable(session->wsi);
            return 0;
        }

        std::string text_payload;
        std::vector<uint8_t> binary_payload;
        StreamId binary_id = STREAM_FUSED;
        bool have_binary = false;

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            if (!session->json_queue.empty()) {
                text_payload = std::move(session->json_queue.front());
                session->json_queue.pop_front();
            } else {
                // Round-robin through streams starting at next_stream_to_send.
                for (int step = 0; step < STREAM_COUNT - 1; ++step) {
                    uint8_t s = session->next_stream_to_send + step;
                    if (s >= STREAM_COUNT) s = STREAM_FUSED + (s - STREAM_COUNT);
                    if (s < STREAM_FUSED) s = STREAM_FUSED;
                    if (session->jpeg_dirty[s]) {
                        binary_id = static_cast<StreamId>(s);
                        binary_payload = std::move(session->latest_jpeg[s]);
                        session->latest_jpeg[s].clear();
                        session->jpeg_dirty[s] = false;
                        session->next_stream_to_send = (s + 1 < STREAM_COUNT)
                            ? static_cast<uint8_t>(s + 1) : STREAM_FUSED;
                        have_binary = true;
                        break;
                    }
                }
            }
        }

        if (!text_payload.empty()) {
            std::vector<uint8_t> buf(LWS_PRE + text_payload.size());
            std::memcpy(buf.data() + LWS_PRE,
                        text_payload.data(), text_payload.size());
            int n = lws_write(session->wsi, buf.data() + LWS_PRE,
                              text_payload.size(), LWS_WRITE_TEXT);
            if (n < static_cast<int>(text_payload.size())) return -1;
        } else if (have_binary) {
            std::vector<uint8_t> buf(LWS_PRE + 1 + binary_payload.size());
            buf[LWS_PRE] = static_cast<uint8_t>(binary_id);
            std::memcpy(buf.data() + LWS_PRE + 1,
                        binary_payload.data(), binary_payload.size());
            int n = lws_write(session->wsi, buf.data() + LWS_PRE,
                              1 + binary_payload.size(), LWS_WRITE_BINARY);
            if (n < static_cast<int>(1 + binary_payload.size())) return -1;
        } else {
            // Nothing pending — stop draining.
            return 0;
        }
    }
    // Hit the batch cap with more work outstanding — schedule another tick.
    lws_callback_on_writable(session->wsi);
    return 0;
}

}  // namespace visionsense
