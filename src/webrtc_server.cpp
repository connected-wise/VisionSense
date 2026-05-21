#include "webrtc_server.h"

#include "dashboard_server.h"  // for StreamId enum + STREAM_COUNT

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>

#include <cstdio>
#include <cstring>
#include <sstream>
#include <vector>

namespace visionsense {

// ---------------------------------------------------------------------------
// Stream capabilities — must match what node_dashboard's callbacks resize to.
// Streams with width=0 are not WebRTC-eligible (variable size).
// ---------------------------------------------------------------------------
namespace {
// Per-stream encoder caps. iframeinterval tuned so a keyframe lands roughly
// every 0.5 s at each stream's expected source rate:
//   FUSED  ~ 4 Hz (fuse_data is detect-rate bound) → 2-frame interval
//   DEPTH  ~20 Hz                                  → 10
//   DRIVER ~30 Hz                                  → 15
//   BEV    ~10 Hz                                  → 5
//   CAMERA ~25 Hz                                  → 12
//   LANES  ~20 Hz                                  → 10
// Without this, a long IDR gap on a slow stream means the decoder stalls
// (no fresh keyframe to re-sync against) — exactly the "fused freezes,
// others fine" symptom.
constexpr StreamCaps kCaps[] = {
    {0,    0,   0,   0},   // 0 unused
    {1280, 720, 30,  2},   // 1 FUSED  — keyframe every ~0.5 s at 4 Hz
    {720,  450, 30,  10},  // 2 DEPTH
    {640,  480, 30,  15},  // 3 DRIVER
    {480,  360, 30,  5},   // 4 BEV
    {1280, 720, 30,  12},  // 5 CAMERA
    {640,  360, 30,  10},  // 6 LANES
    {0,    0,   30,  15},  // 7 DETECT (variable, not WebRTC)
    {0,    0,   30,  15},  // 8 SIGNS  (variable, not WebRTC)
};
static_assert(sizeof(kCaps) / sizeof(kCaps[0]) == STREAM_COUNT,
              "kCaps must size to STREAM_COUNT");

constexpr int kDefaultBitrateBps = 4'000'000;  // 4 Mbps; LAN-friendly

// Hand-rolled JSON helpers — messages are short, well-known shape; pulling in
// a JSON library is overkill.

std::string json_escape(const std::string &s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\b': o += "\\b";  break;
            case '\f': o += "\\f";  break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<int>(c));
                    o += buf;
                } else {
                    o += c;
                }
        }
    }
    return o;
}

// Find `"<key>"` in json then read the immediately-following value.
// Returns an empty optional-ish (defaults) on miss.

bool json_get_int(const std::string &json, const std::string &key, int *out) {
    std::string needle = "\"" + key + "\"";
    auto p = json.find(needle);
    if (p == std::string::npos) return false;
    p = json.find(':', p + needle.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) ++p;
    bool neg = false;
    if (p < json.size() && json[p] == '-') { neg = true; ++p; }
    int v = 0;
    bool any = false;
    while (p < json.size() && json[p] >= '0' && json[p] <= '9') {
        v = v * 10 + (json[p] - '0');
        ++p;
        any = true;
    }
    if (!any) return false;
    *out = neg ? -v : v;
    return true;
}

bool json_get_string(const std::string &json, const std::string &key,
                     std::string *out) {
    std::string needle = "\"" + key + "\"";
    auto p = json.find(needle);
    if (p == std::string::npos) return false;
    p = json.find(':', p + needle.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t')) ++p;
    if (p >= json.size() || json[p] != '"') return false;
    ++p;
    out->clear();
    while (p < json.size() && json[p] != '"') {
        char c = json[p];
        if (c == '\\' && p + 1 < json.size()) {
            char n = json[p + 1];
            switch (n) {
                case '"':  *out += '"';  p += 2; continue;
                case '\\': *out += '\\'; p += 2; continue;
                case '/':  *out += '/';  p += 2; continue;
                case 'b':  *out += '\b'; p += 2; continue;
                case 'f':  *out += '\f'; p += 2; continue;
                case 'n':  *out += '\n'; p += 2; continue;
                case 'r':  *out += '\r'; p += 2; continue;
                case 't':  *out += '\t'; p += 2; continue;
                case 'u': {
                    if (p + 5 >= json.size()) return false;
                    int code = 0;
                    for (int i = 0; i < 4; ++i) {
                        char h = json[p + 2 + i];
                        code <<= 4;
                        if (h >= '0' && h <= '9') code |= (h - '0');
                        else if (h >= 'a' && h <= 'f') code |= (h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') code |= (h - 'A' + 10);
                        else return false;
                    }
                    if (code < 0x80) *out += static_cast<char>(code);
                    else if (code < 0x800) {
                        *out += static_cast<char>(0xC0 | (code >> 6));
                        *out += static_cast<char>(0x80 | (code & 0x3F));
                    } else {
                        *out += static_cast<char>(0xE0 | (code >> 12));
                        *out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                        *out += static_cast<char>(0x80 | (code & 0x3F));
                    }
                    p += 6;
                    continue;
                }
                default: return false;
            }
        }
        *out += c;
        ++p;
    }
    return p < json.size();
}

}  // namespace

const StreamCaps &caps_for(StreamId id) {
    int i = static_cast<int>(id);
    if (i < 0 || i >= static_cast<int>(STREAM_COUNT)) return kCaps[0];
    return kCaps[i];
}

// ---------------------------------------------------------------------------
// PipelineCtx — owns one GStreamer pipeline + webrtcbin pair for a
// (client, stream) tuple.
// ---------------------------------------------------------------------------
struct WebRTCManager::PipelineCtx {
    WebRTCManager *owner = nullptr;
    ClientHandle client = nullptr;
    StreamId stream{};
    GstElement *pipeline = nullptr;
    GstElement *appsrc = nullptr;
    GstElement *webrtcbin = nullptr;
    int width = 0;
    int height = 0;
    int fps = 30;
};

// ---------------------------------------------------------------------------
// Idle-source bridge: schedule a lambda onto the GMainLoop thread.
// ---------------------------------------------------------------------------
namespace {
struct IdleData {
    std::function<void()> fn;
};
gboolean idle_invoke(gpointer user) {
    auto *d = static_cast<IdleData *>(user);
    d->fn();
    delete d;
    return G_SOURCE_REMOVE;
}
void post_to_loop(GMainContext *ctx, std::function<void()> fn) {
    auto *d = new IdleData{std::move(fn)};
    GSource *src = g_idle_source_new();
    g_source_set_priority(src, G_PRIORITY_DEFAULT);
    g_source_set_callback(src, idle_invoke, d, nullptr);
    g_source_attach(src, ctx);
    g_source_unref(src);
}
}  // namespace

// ---------------------------------------------------------------------------
// WebRTCManager
// ---------------------------------------------------------------------------

WebRTCManager::WebRTCManager(SignalingSender sender)
    : sender_(std::move(sender)) {}

WebRTCManager::~WebRTCManager() { stop(); }

bool WebRTCManager::start() {
    if (running_.exchange(true)) return true;
    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }
    // Verify the elements we need exist.
    for (const char *factory : {"appsrc", "videoconvert", "nvvidconv",
                                "nvv4l2h264enc", "h264parse", "rtph264pay",
                                "webrtcbin"}) {
        GstElementFactory *f = gst_element_factory_find(factory);
        if (!f) {
            g_printerr("[webrtc] required GStreamer element not found: %s\n",
                       factory);
            running_ = false;
            return false;
        }
        gst_object_unref(f);
    }
    loop_ctx_ = g_main_context_new();
    loop_ = g_main_loop_new(loop_ctx_, FALSE);
    thread_ = std::thread([this] { thread_main(); });
    return true;
}

void WebRTCManager::stop() {
    if (!running_.exchange(false)) return;
    if (loop_) {
        // Tear down all pipelines on the loop thread.
        post_to_loop(loop_ctx_, [this]() {
            std::vector<PipelineKey> keys;
            {
                std::lock_guard<std::mutex> lock(pipelines_mutex_);
                for (auto &kv : pipelines_) keys.push_back(kv.first);
            }
            for (auto &k : keys) {
                mainloop_destroy_pipeline(
                    k.first, static_cast<StreamId>(k.second));
            }
            g_main_loop_quit(loop_);
        });
    }
    if (thread_.joinable()) thread_.join();
    if (loop_) { g_main_loop_unref(loop_); loop_ = nullptr; }
    if (loop_ctx_) { g_main_context_unref(loop_ctx_); loop_ctx_ = nullptr; }
}

void WebRTCManager::thread_main() {
    g_main_context_push_thread_default(loop_ctx_);
    g_main_loop_run(loop_);
    g_main_context_pop_thread_default(loop_ctx_);
}

WebRTCManager::PipelineCtx *
WebRTCManager::find_locked(ClientHandle client, StreamId stream) {
    auto it = pipelines_.find({client, static_cast<int>(stream)});
    if (it == pipelines_.end()) return nullptr;
    return it->second.get();
}

bool WebRTCManager::has_active_subscribers(StreamId stream) {
    std::lock_guard<std::mutex> lock(pipelines_mutex_);
    for (const auto &kv : pipelines_) {
        if (kv.first.second == static_cast<int>(stream)) return true;
    }
    return false;
}

void WebRTCManager::push_frame(StreamId stream, const cv::Mat &frame_bgr) {
    if (!running_.load()) return;
    if (frame_bgr.empty()) return;
    const auto &c = caps_for(stream);
    if (c.width == 0) return;  // not WebRTC-eligible
    if (frame_bgr.cols != c.width || frame_bgr.rows != c.height) return;
    if (frame_bgr.type() != CV_8UC3) return;

    // Snapshot list of pipelines for this stream while holding the lock,
    // then push without holding it.
    std::vector<GstElement *> targets;
    {
        std::lock_guard<std::mutex> lock(pipelines_mutex_);
        for (const auto &kv : pipelines_) {
            if (kv.first.second == static_cast<int>(stream) &&
                kv.second->appsrc) {
                gst_object_ref(kv.second->appsrc);  // pin
                targets.push_back(kv.second->appsrc);
            }
        }
    }
    if (targets.empty()) return;

    // Copy the BGR frame into a GstBuffer. nvvidconv accepts BGR via
    // standard memory; it'll move to NVMM internally.
    size_t bytes = static_cast<size_t>(frame_bgr.total()) * frame_bgr.elemSize();
    for (GstElement *src : targets) {
        GstBuffer *buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
        GstMapInfo info;
        if (gst_buffer_map(buf, &info, GST_MAP_WRITE)) {
            // Contiguous copy — cv::Mat is typically contiguous after resize.
            if (frame_bgr.isContinuous()) {
                std::memcpy(info.data, frame_bgr.data, bytes);
            } else {
                size_t row_bytes = frame_bgr.cols * frame_bgr.elemSize();
                for (int r = 0; r < frame_bgr.rows; ++r) {
                    std::memcpy(info.data + r * row_bytes,
                                frame_bgr.ptr(r), row_bytes);
                }
            }
            gst_buffer_unmap(buf, &info);
        }
        GstFlowReturn ret = GST_FLOW_OK;
        g_signal_emit_by_name(src, "push-buffer", buf, &ret);
        gst_buffer_unref(buf);
        gst_object_unref(src);
        (void)ret;  // appsrc absorbs even when downstream is briefly slow
    }
}

void WebRTCManager::send_signaling(ClientHandle client,
                                    const std::string &json) {
    if (sender_) sender_(client, json);
}

bool WebRTCManager::on_signaling_message(ClientHandle client,
                                         const std::string &json) {
    std::string t;
    if (!json_get_string(json, "t", &t)) return false;
    if (t == "webrtc-request") {
        int stream = 0;
        if (!json_get_int(json, "stream", &stream)) return false;
        StreamId sid = static_cast<StreamId>(stream);
        post_to_loop(loop_ctx_,
                     [this, client, sid] {
                         mainloop_create_pipeline(client, sid);
                     });
        return true;
    } else if (t == "webrtc-answer") {
        int stream = 0;
        std::string sdp;
        if (!json_get_int(json, "stream", &stream)) return true;
        if (!json_get_string(json, "sdp", &sdp)) return true;
        StreamId sid = static_cast<StreamId>(stream);
        post_to_loop(loop_ctx_, [this, client, sid, sdp]() mutable {
            mainloop_apply_answer(client, sid, std::move(sdp));
        });
        return true;
    } else if (t == "webrtc-ice") {
        int stream = 0;
        int mline = 0;
        std::string cand;
        if (!json_get_int(json, "stream", &stream)) return true;
        if (!json_get_int(json, "sdpMLineIndex", &mline)) mline = 0;
        if (!json_get_string(json, "candidate", &cand)) return true;
        StreamId sid = static_cast<StreamId>(stream);
        unsigned mline_u = static_cast<unsigned>(mline);
        post_to_loop(loop_ctx_, [this, client, sid, mline_u, cand]() mutable {
            mainloop_add_ice(client, sid, mline_u, std::move(cand));
        });
        return true;
    } else if (t == "webrtc-stop") {
        int stream = 0;
        if (!json_get_int(json, "stream", &stream)) return true;
        StreamId sid = static_cast<StreamId>(stream);
        post_to_loop(loop_ctx_, [this, client, sid] {
            mainloop_destroy_pipeline(client, sid);
        });
        return true;
    }
    return false;
}

void WebRTCManager::on_client_disconnected(ClientHandle client) {
    if (!running_.load() || !loop_ctx_) return;
    post_to_loop(loop_ctx_,
                 [this, client] { mainloop_destroy_client_pipelines(client); });
}

// ---------------------------------------------------------------------------
// GMainLoop-thread implementations.
// ---------------------------------------------------------------------------

void WebRTCManager::mainloop_create_pipeline(ClientHandle client,
                                              StreamId stream) {
    const auto &cap = caps_for(stream);
    if (cap.width == 0) {
        g_printerr("[webrtc] stream %d is not WebRTC-eligible\n",
                   static_cast<int>(stream));
        return;
    }
    {
        std::lock_guard<std::mutex> lock(pipelines_mutex_);
        if (find_locked(client, stream)) {
            // Re-request — tear down and recreate to be safe.
            // Defer the destroy by clearing it here; create new below.
            auto it = pipelines_.find({client, static_cast<int>(stream)});
            it->second.reset();
            pipelines_.erase(it);
        }
    }

    // Build the pipeline. videoconvert before nvvidconv handles the BGR→I420
    // step in regular memory; nvvidconv then moves to NVMM/NV12 for NVENC.
    // iframeinterval is taken from kCaps so low-rate streams (like FUSED)
    // get frequent keyframes and don't stall waiting for IDR recovery.
    char descbuf[1024];
    std::snprintf(descbuf, sizeof(descbuf),
        "appsrc name=src is-live=true do-timestamp=true format=time "
            "max-buffers=4 leaky-type=2 "
            "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 ! "
        "videoconvert ! video/x-raw,format=I420 ! "
        "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        "nvv4l2h264enc maxperf-enable=true insert-sps-pps=true "
            "iframeinterval=%d control-rate=1 bitrate=%d "
            "preset-level=1 profile=0 ! "
        "h264parse config-interval=-1 ! "
        "rtph264pay pt=96 config-interval=-1 ! "
        "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
        "webrtcbin name=webrtc bundle-policy=max-bundle latency=0",
        cap.width, cap.height, cap.fps,
        cap.iframeinterval > 0 ? cap.iframeinterval : 15,
        kDefaultBitrateBps);

    GError *err = nullptr;
    GstElement *pipeline = gst_parse_launch(descbuf, &err);
    if (!pipeline) {
        g_printerr("[webrtc] gst_parse_launch failed: %s\n",
                   err ? err->message : "(unknown)");
        if (err) g_error_free(err);
        return;
    }

    GstElement *appsrc =
        gst_bin_get_by_name(GST_BIN(pipeline), "src");
    GstElement *webrtcbin =
        gst_bin_get_by_name(GST_BIN(pipeline), "webrtc");
    if (!appsrc || !webrtcbin) {
        g_printerr("[webrtc] failed to fetch appsrc/webrtcbin from pipeline\n");
        if (appsrc) gst_object_unref(appsrc);
        if (webrtcbin) gst_object_unref(webrtcbin);
        gst_object_unref(pipeline);
        return;
    }

    auto ctx = std::make_unique<PipelineCtx>();
    ctx->owner = this;
    ctx->client = client;
    ctx->stream = stream;
    ctx->pipeline = pipeline;
    ctx->appsrc = appsrc;
    ctx->webrtcbin = webrtcbin;
    ctx->width = cap.width;
    ctx->height = cap.height;
    ctx->fps = cap.fps;

    g_signal_connect(webrtcbin, "on-negotiation-needed",
                     G_CALLBACK(s_on_negotiation_needed), ctx.get());
    g_signal_connect(webrtcbin, "on-ice-candidate",
                     G_CALLBACK(s_on_ice_candidate), ctx.get());

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING)
        == GST_STATE_CHANGE_FAILURE) {
        g_printerr("[webrtc] failed to set pipeline PLAYING\n");
        gst_object_unref(pipeline);  // also unrefs appsrc/webrtcbin children
        return;
    }

    // Kick the pipeline with one black BGR frame so the caps negotiate all
    // the way down to webrtcbin's sink pad. Without this, on-negotiation-
    // needed only fires once the first real ROS frame arrives — which delays
    // SDP exchange by potentially seconds (or never, if the upstream topic
    // is silent).
    {
        size_t bytes = static_cast<size_t>(cap.width) * cap.height * 3;
        GstBuffer *buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
        if (buf) {
            GstMapInfo info;
            if (gst_buffer_map(buf, &info, GST_MAP_WRITE)) {
                std::memset(info.data, 0, bytes);
                gst_buffer_unmap(buf, &info);
            }
            GstFlowReturn ret = GST_FLOW_OK;
            g_signal_emit_by_name(appsrc, "push-buffer", buf, &ret);
            gst_buffer_unref(buf);
        }
    }

    g_printerr("[webrtc] pipeline started for client=%p stream=%d %dx%d@%d\n",
               client, static_cast<int>(stream),
               cap.width, cap.height, cap.fps);

    std::lock_guard<std::mutex> lock(pipelines_mutex_);
    pipelines_[{client, static_cast<int>(stream)}] = std::move(ctx);
}

void WebRTCManager::mainloop_destroy_pipeline(ClientHandle client,
                                               StreamId stream) {
    std::unique_ptr<PipelineCtx> ctx;
    {
        std::lock_guard<std::mutex> lock(pipelines_mutex_);
        auto it = pipelines_.find({client, static_cast<int>(stream)});
        if (it == pipelines_.end()) return;
        ctx = std::move(it->second);
        pipelines_.erase(it);
    }
    if (ctx->pipeline) {
        gst_element_set_state(ctx->pipeline, GST_STATE_NULL);
        gst_object_unref(ctx->pipeline);
        ctx->pipeline = nullptr;
    }
    if (ctx->appsrc) {
        gst_object_unref(ctx->appsrc);
        ctx->appsrc = nullptr;
    }
    if (ctx->webrtcbin) {
        gst_object_unref(ctx->webrtcbin);
        ctx->webrtcbin = nullptr;
    }
    g_printerr("[webrtc] pipeline stopped for client=%p stream=%d\n",
               client, static_cast<int>(stream));
}

void WebRTCManager::mainloop_destroy_client_pipelines(ClientHandle client) {
    std::vector<int> streams;
    {
        std::lock_guard<std::mutex> lock(pipelines_mutex_);
        for (auto &kv : pipelines_) {
            if (kv.first.first == client) streams.push_back(kv.first.second);
        }
    }
    for (int s : streams) {
        mainloop_destroy_pipeline(client, static_cast<StreamId>(s));
    }
}

void WebRTCManager::mainloop_apply_answer(ClientHandle client, StreamId stream,
                                           std::string sdp) {
    PipelineCtx *ctx = nullptr;
    {
        std::lock_guard<std::mutex> lock(pipelines_mutex_);
        ctx = find_locked(client, stream);
    }
    if (!ctx) return;
    GstSDPMessage *sdp_msg = nullptr;
    if (gst_sdp_message_new(&sdp_msg) != GST_SDP_OK) return;
    if (gst_sdp_message_parse_buffer(
            reinterpret_cast<const guint8 *>(sdp.data()),
            static_cast<guint>(sdp.size()), sdp_msg) != GST_SDP_OK) {
        gst_sdp_message_free(sdp_msg);
        g_printerr("[webrtc] failed to parse answer SDP\n");
        return;
    }
    GstWebRTCSessionDescription *desc =
        gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);
    GstPromise *promise = gst_promise_new();
    g_signal_emit_by_name(ctx->webrtcbin, "set-remote-description",
                          desc, promise);
    gst_promise_interrupt(promise);
    gst_promise_unref(promise);
    gst_webrtc_session_description_free(desc);
}

void WebRTCManager::mainloop_add_ice(ClientHandle client, StreamId stream,
                                      unsigned int mline_index,
                                      std::string candidate) {
    PipelineCtx *ctx = nullptr;
    {
        std::lock_guard<std::mutex> lock(pipelines_mutex_);
        ctx = find_locked(client, stream);
    }
    if (!ctx) return;
    if (candidate.empty()) return;  // end-of-candidates sentinel
    g_signal_emit_by_name(ctx->webrtcbin, "add-ice-candidate",
                          mline_index, candidate.c_str());
}

// ---------------------------------------------------------------------------
// Static GStreamer signal trampolines.
// ---------------------------------------------------------------------------

void WebRTCManager::s_on_negotiation_needed(GstElement *webrtcbin,
                                             void *user_data) {
    (void)webrtcbin;
    auto *ctx = static_cast<PipelineCtx *>(user_data);
    GstPromise *promise = gst_promise_new_with_change_func(
        &WebRTCManager::s_on_offer_created, ctx, nullptr);
    g_signal_emit_by_name(ctx->webrtcbin, "create-offer", nullptr, promise);
}

void WebRTCManager::s_on_offer_created(GstPromise *promise, void *user_data) {
    auto *ctx = static_cast<PipelineCtx *>(user_data);
    const GstStructure *reply = gst_promise_get_reply(promise);
    GstWebRTCSessionDescription *offer = nullptr;
    gst_structure_get(reply, "offer",
                      GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
    gst_promise_unref(promise);
    if (!offer) return;

    GstPromise *p = gst_promise_new();
    g_signal_emit_by_name(ctx->webrtcbin, "set-local-description", offer, p);
    gst_promise_interrupt(p);
    gst_promise_unref(p);

    gchar *sdp_text = gst_sdp_message_as_text(offer->sdp);
    if (sdp_text) {
        std::ostringstream out;
        out << "{\"t\":\"webrtc-offer\",\"stream\":"
            << static_cast<int>(ctx->stream)
            << ",\"sdp\":\"" << json_escape(sdp_text) << "\"}";
        ctx->owner->send_signaling(ctx->client, out.str());
        g_free(sdp_text);
    }
    gst_webrtc_session_description_free(offer);
}

void WebRTCManager::s_on_ice_candidate(GstElement *webrtcbin,
                                        unsigned int mline_index,
                                        char *candidate,
                                        void *user_data) {
    (void)webrtcbin;
    auto *ctx = static_cast<PipelineCtx *>(user_data);
    if (!candidate) return;
    std::ostringstream out;
    out << "{\"t\":\"webrtc-ice\",\"stream\":"
        << static_cast<int>(ctx->stream)
        << ",\"sdpMLineIndex\":" << mline_index
        << ",\"candidate\":\"" << json_escape(candidate) << "\"}";
    ctx->owner->send_signaling(ctx->client, out.str());
}

}  // namespace visionsense
