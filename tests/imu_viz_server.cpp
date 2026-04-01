// =============================================================================
// IMU 3D Visualizer — Web-based
// Usage: sudo ./build/imu_viz_server [config_path] [port]
//
// Reads WIT-Motion IMU and serves a 3D visualization via HTTP.
// Access from your PC:
//   1. ssh -L 8080:localhost:8080 robot
//   2. Open http://localhost:8080 in your browser
//
// No motors involved. Safe to run anytime.
// =============================================================================

#include <iostream>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <memory>
#include <vector>
#include <mutex>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "observations.hpp"
#include "common/robot_config.hpp"
#include "common/math_utils.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// --- Shared IMU state ---
struct IMUState {
    std::mutex mtx;
    float quat[4]{1, 0, 0, 0};   // w, x, y, z (WIT frame)
    float gyro[3]{0, 0, 0};       // rad/s (WIT frame)
    float acc[3]{0, 0, 0};        // m/s^2 (WIT frame)
    // Body frame
    double rpy[3]{0, 0, 0};       // rad
    double grav[3]{0, 0, -1};     // gravity direction in body frame
    double omega[3]{0, 0, 0};     // rad/s body frame
};
static IMUState g_imu;

// --- Embedded HTML page ---
static const char* HTML_PAGE = R"RAWHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>IMU 3D Visualizer</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body { background:#1a1a2e; color:#e0e0e0; font-family:'Courier New',monospace; overflow:hidden; }
  #container { display:flex; height:100vh; }
  #view3d { flex:1; }
  #panel { width:360px; padding:16px; overflow-y:auto; background:#16213e; border-left:1px solid #0f3460; }
  h2 { color:#e94560; margin-bottom:12px; font-size:16px; }
  .section { margin-bottom:16px; }
  .label { color:#888; font-size:12px; }
  .val { font-size:14px; color:#53d8fb; }
  .row { display:flex; justify-content:space-between; margin:2px 0; }
  .check { margin:2px 0; font-size:13px; }
  .ok { color:#4caf50; }
  .bad { color:#e94560; }
  #status { font-size:12px; color:#888; margin-bottom:8px; }
  canvas { display:block; }
</style>
</head>
<body>
<div id="container">
  <div id="view3d"></div>
  <div id="panel">
    <h2>IMU 3D Visualizer</h2>
    <div id="status">Connecting...</div>

    <div class="section">
      <div class="label">Quaternion (WIT) [w, x, y, z]</div>
      <div class="val" id="quat">-</div>
    </div>
    <div class="section">
      <div class="label">Euler (body) [R, P, Y] deg</div>
      <div class="val" id="rpy">-</div>
    </div>
    <div class="section">
      <div class="label">Gyro (body) [x, y, z] rad/s</div>
      <div class="val" id="omega">-</div>
    </div>
    <div class="section">
      <div class="label">Accel (WIT) [x, y, z] m/s^2</div>
      <div class="val" id="acc">-</div>
    </div>
    <div class="section">
      <div class="label">Gravity (body) [x, y, z]</div>
      <div class="val" id="grav">-</div>
    </div>

    <div class="section">
      <h2>Sanity Checks (flat & still)</h2>
      <div id="checks"></div>
    </div>

    <div class="section" style="margin-top:24px; font-size:11px; color:#555;">
      Tilt the robot to see attitude change in real-time.<br>
      Body frame: X=forward(red), Y=left(green), Z=up(blue)
    </div>
  </div>
</div>

<script type="importmap">
{ "imports": { "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js" } }
</script>
<script type="module">
import * as THREE from 'three';

// --- Three.js setup ---
const container = document.getElementById('view3d');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100);
camera.position.set(3, 2, 3);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
container.appendChild(renderer.domElement);

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}
resize();
window.addEventListener('resize', resize);

// Lights
scene.add(new THREE.AmbientLight(0x404040, 2));
const dlight = new THREE.DirectionalLight(0xffffff, 1.5);
dlight.position.set(5, 5, 5);
scene.add(dlight);

// Grid
const grid = new THREE.GridHelper(6, 12, 0x333355, 0x222244);
scene.add(grid);

// World axes (thin)
const makeWorldAxis = (color, dir) => {
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.3 });
  const pts = [new THREE.Vector3(0,0,0), dir.clone().multiplyScalar(3)];
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  return new THREE.Line(geo, mat);
};
scene.add(makeWorldAxis(0xff4444, new THREE.Vector3(1,0,0)));
scene.add(makeWorldAxis(0x44ff44, new THREE.Vector3(0,1,0)));
scene.add(makeWorldAxis(0x4444ff, new THREE.Vector3(0,0,1)));

// Robot body (box representing the quadruped)
const bodyGeo = new THREE.BoxGeometry(0.6, 0.3, 1.0);  // width(Y), height(Z), length(X)
const bodyMat = new THREE.MeshPhongMaterial({ color: 0x336699, transparent: true, opacity: 0.85 });
const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
const robotGroup = new THREE.Group();
robotGroup.add(bodyMesh);

// Body axes (X=forward=red, Y=left=green, Z=up=blue)
const makeArrow = (color, dir) => new THREE.ArrowHelper(dir, new THREE.Vector3(), 1.2, color, 0.15, 0.08);
robotGroup.add(makeArrow(0xff0000, new THREE.Vector3(0, 0, -1)));  // X forward -> -Z in three.js
robotGroup.add(makeArrow(0x00ff00, new THREE.Vector3(-1, 0, 0)));  // Y left -> -X in three.js
robotGroup.add(makeArrow(0x0000ff, new THREE.Vector3(0, 1, 0)));   // Z up -> +Y in three.js

// Legs (simple cylinders at corners)
const legGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.4);
const legMat = new THREE.MeshPhongMaterial({ color: 0x888888 });
const legPositions = [
  [0.2, -0.2, -0.4],  // front-left
  [-0.2, -0.2, -0.4], // front-right
  [0.2, -0.2, 0.4],   // rear-left
  [-0.2, -0.2, 0.4],  // rear-right
];
for (const [x, y, z] of legPositions) {
  const leg = new THREE.Mesh(legGeo, legMat);
  leg.position.set(x, y, z);
  robotGroup.add(leg);
}

// Head indicator (small cone at front)
const headGeo = new THREE.ConeGeometry(0.08, 0.2, 8);
const headMat = new THREE.MeshPhongMaterial({ color: 0xe94560 });
const headMesh = new THREE.Mesh(headGeo, headMat);
headMesh.rotation.x = Math.PI / 2;
headMesh.position.set(0, 0.05, -0.6);
robotGroup.add(headMesh);

scene.add(robotGroup);

// Gravity arrow (yellow, world-space)
const gravArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 2, 0), 1.0, 0xffff00, 0.12, 0.07
);
scene.add(gravArrow);

// --- IMU quaternion (body frame) to Three.js rotation ---
// Body: X=forward, Y=left, Z=up
// Three.js: X=right, Y=up, Z=toward-camera
// Mapping: body_X -> -three_Z, body_Y -> -three_X, body_Z -> three_Y
//
// We receive the body-frame quaternion representing rotation from body to world.
// We need to convert this to Three.js coordinates.

let currentQuat = new THREE.Quaternion();

function bodyQuatToThreeJS(qw, qx, qy, qz) {
  // The IMU quaternion is in WIT frame (X=right, Y=forward, Z=up)
  // Three.js uses (X=right, Y=up, Z=toward-viewer)
  // WIT_X -> Three_X, WIT_Y -> -Three_Z, WIT_Z -> Three_Y
  // For quaternion: q_three = (qw, qx, qz, -qy) after frame mapping
  // WIT(x,y,z) -> Three(x, z, -y)
  return new THREE.Quaternion(qx, qz, -qy, qw);
}

// --- SSE connection ---
const fmt = (v, n=4) => v.toFixed(n);
const fmt3 = (a) => `${fmt(a[0])}  ${fmt(a[1])}  ${fmt(a[2])}`;

function connect() {
  const evtSrc = new EventSource('/events');
  document.getElementById('status').textContent = 'Connected';

  evtSrc.onmessage = (e) => {
    const d = JSON.parse(e.data);

    // Update 3D model
    currentQuat = bodyQuatToThreeJS(d.qw, d.qx, d.qy, d.qz);
    robotGroup.quaternion.copy(currentQuat);

    // Update panel
    document.getElementById('quat').textContent =
      `${fmt(d.qw)} ${fmt(d.qx)} ${fmt(d.qy)} ${fmt(d.qz)}`;
    document.getElementById('rpy').textContent =
      `${fmt(d.roll_deg)} ${fmt(d.pitch_deg)} ${fmt(d.yaw_deg)}`;
    document.getElementById('omega').textContent = fmt3([d.ox, d.oy, d.oz]);
    document.getElementById('acc').textContent = fmt3([d.ax, d.ay, d.az]);
    document.getElementById('grav').textContent = fmt3([d.gx, d.gy, d.gz]);

    // Sanity checks
    const checks = [
      { name: 'roll', val: d.roll_deg, expect: 0, tol: 3.0, unit: 'deg' },
      { name: 'pitch', val: d.pitch_deg, expect: 0, tol: 3.0, unit: 'deg' },
      { name: 'grav_x', val: d.gx, expect: 0, tol: 0.05 },
      { name: 'grav_y', val: d.gy, expect: 0, tol: 0.05 },
      { name: 'grav_z', val: d.gz, expect: -1, tol: 0.05 },
      { name: 'omega_norm', val: Math.sqrt(d.ox*d.ox+d.oy*d.oy+d.oz*d.oz), expect: 0, tol: 0.05 },
      { name: 'quat_norm', val: Math.sqrt(d.qw*d.qw+d.qx*d.qx+d.qy*d.qy+d.qz*d.qz), expect: 1, tol: 0.01 },
    ];
    let html = '';
    for (const c of checks) {
      const ok = Math.abs(c.val - c.expect) < c.tol;
      html += `<div class="check ${ok ? 'ok' : 'bad'}">${ok ? '&#10003;' : '&#10007;'} ${c.name}: ${fmt(c.val)} (expect ${c.expect})</div>`;
    }
    document.getElementById('checks').innerHTML = html;
  };

  evtSrc.onerror = () => {
    document.getElementById('status').textContent = 'Disconnected. Reconnecting...';
    evtSrc.close();
    setTimeout(connect, 2000);
  };
}
connect();

// --- Render loop ---
function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>
)RAWHTML";

// --- Minimal HTTP server ---
static void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static std::string read_request(int client_fd) {
    char buf[4096];
    std::string req;
    for (int i = 0; i < 10; i++) {
        ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        if (n > 0) {
            buf[n] = '\0';
            req += buf;
            if (req.find("\r\n\r\n") != std::string::npos) break;
        } else if (n == 0) {
            break;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            break;
        }
    }
    return req;
}

static std::string get_request_path(const std::string& req) {
    auto sp1 = req.find(' ');
    if (sp1 == std::string::npos) return "/";
    auto sp2 = req.find(' ', sp1 + 1);
    if (sp2 == std::string::npos) return "/";
    return req.substr(sp1 + 1, sp2 - sp1 - 1);
}

static void send_all(int fd, const char* data, size_t len) {
    size_t sent = 0;
    while (sent < len && g_running) {
        ssize_t n = send(fd, data + sent, len - sent, MSG_NOSIGNAL);
        if (n > 0) sent += n;
        else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else break;
    }
}

static void send_string(int fd, const std::string& s) {
    send_all(fd, s.c_str(), s.size());
}

static std::string make_json() {
    std::lock_guard<std::mutex> lk(g_imu.mtx);
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(6);
    ss << "{"
       << "\"qw\":" << g_imu.quat[0] << ","
       << "\"qx\":" << g_imu.quat[1] << ","
       << "\"qy\":" << g_imu.quat[2] << ","
       << "\"qz\":" << g_imu.quat[3] << ","
       << "\"roll_deg\":" << g_imu.rpy[0] * 180.0 / M_PI << ","
       << "\"pitch_deg\":" << g_imu.rpy[1] * 180.0 / M_PI << ","
       << "\"yaw_deg\":" << g_imu.rpy[2] * 180.0 / M_PI << ","
       << "\"ox\":" << g_imu.omega[0] << ","
       << "\"oy\":" << g_imu.omega[1] << ","
       << "\"oz\":" << g_imu.omega[2] << ","
       << "\"ax\":" << g_imu.acc[0] << ","
       << "\"ay\":" << g_imu.acc[1] << ","
       << "\"az\":" << g_imu.acc[2] << ","
       << "\"gx\":" << g_imu.grav[0] << ","
       << "\"gy\":" << g_imu.grav[1] << ","
       << "\"gz\":" << g_imu.grav[2]
       << "}";
    return ss.str();
}

static void handle_sse(int client_fd) {
    std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n";
    send_string(client_fd, header);

    while (g_running) {
        std::string json = make_json();
        std::string msg = "data: " + json + "\n\n";
        ssize_t n = send(client_fd, msg.c_str(), msg.size(), MSG_NOSIGNAL);
        if (n <= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz to browser
    }
    close(client_fd);
}

static void handle_client(int client_fd) {
    set_nonblocking(client_fd);
    std::string req = read_request(client_fd);
    std::string path = get_request_path(req);

    if (path == "/events") {
        // SSE — run in this thread (long-lived)
        handle_sse(client_fd);
        return;
    }

    // Serve HTML page for any other path
    std::string body(HTML_PAGE);
    std::ostringstream resp;
    resp << "HTTP/1.1 200 OK\r\n"
         << "Content-Type: text/html; charset=utf-8\r\n"
         << "Content-Length: " << body.size() << "\r\n"
         << "Connection: close\r\n"
         << "\r\n"
         << body;
    send_string(client_fd, resp.str());
    close(client_fd);
}

// --- IMU reading thread ---
static void imu_thread(std::shared_ptr<IMUComponent> imu) {
    Eigen::Matrix3d R_imu2body;
    R_imu2body << 0,  1,  0,
                 -1,  0,  0,
                  0,  0,  1;

    while (g_running) {
        const float* q_raw = imu->get_quaternion();
        const float* g_raw = imu->get_gyro();
        const float* a_raw = imu->get_acc();

        Eigen::Vector4d q(q_raw[0], q_raw[1], q_raw[2], q_raw[3]);
        Eigen::Matrix3d R_wi = math_utils::quat_to_rotation(q);
        Eigen::Matrix3d R_body = R_imu2body * R_wi;
        Eigen::Vector3d rpy = math_utils::quat_to_euler(q);

        Eigen::Vector3d grav_world(0, 0, -1);
        Eigen::Vector3d grav_body = R_body.transpose() * grav_world;

        Eigen::Vector3d gyro_imu(g_raw[0], g_raw[1], g_raw[2]);
        Eigen::Vector3d gyro_body = R_imu2body * gyro_imu;

        {
            std::lock_guard<std::mutex> lk(g_imu.mtx);
            for (int i = 0; i < 4; i++) g_imu.quat[i] = q_raw[i];
            for (int i = 0; i < 3; i++) {
                g_imu.gyro[i] = g_raw[i];
                g_imu.acc[i] = a_raw[i];
                g_imu.rpy[i] = rpy[i];
                g_imu.grav[i] = grav_body[i];
                g_imu.omega[i] = gyro_body[i];
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGPIPE, SIG_IGN);  // ignore broken pipe from disconnected clients

    std::string config_path = (argc > 1) ? argv[1] : "config/robot_params.yaml";
    int port = (argc > 2) ? std::atoi(argv[2]) : 8080;

    RobotConfig cfg = RobotConfig::from_yaml(config_path);

    std::cout << "=== IMU 3D Visualizer Server ===\n";
    std::cout << "Device: " << cfg.imu_device << "\n";
    std::cout << "Initializing IMU...\n";

    auto imu = std::make_shared<IMUComponent>(cfg.imu_device.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Start IMU reading thread
    std::thread imu_thr(imu_thread, imu);

    // Create server socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "Failed to create socket\n";
        g_running = false;
        imu_thr.join();
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind port " << port << ": " << strerror(errno) << "\n";
        close(server_fd);
        g_running = false;
        imu_thr.join();
        return 1;
    }

    listen(server_fd, 8);
    set_nonblocking(server_fd);

    std::cout << "\nServer running on http://0.0.0.0:" << port << "\n";
    std::cout << "From your PC:\n";
    std::cout << "  ssh -L " << port << ":localhost:" << port << " <robot-host>\n";
    std::cout << "  Then open http://localhost:" << port << "\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    std::vector<std::thread> client_threads;

    while (g_running) {
        struct pollfd pfd{server_fd, POLLIN, 0};
        int ret = poll(&pfd, 1, 200);  // 200ms timeout
        if (ret > 0 && (pfd.revents & POLLIN)) {
            struct sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd >= 0) {
                client_threads.emplace_back(handle_client, client_fd);
            }
        }

        // Clean up finished threads
        for (auto it = client_threads.begin(); it != client_threads.end(); ) {
            if (it->joinable()) {
                // Can't check if thread finished without extra sync, just let them accumulate
                // They'll clean up on exit
                ++it;
            } else {
                it = client_threads.erase(it);
            }
        }
    }

    close(server_fd);
    imu_thr.join();

    for (auto& t : client_threads) {
        if (t.joinable()) t.join();
    }

    std::cout << "\nServer stopped.\n";
    return 0;
}
