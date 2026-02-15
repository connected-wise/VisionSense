/*
 * IMU-GPS Node for VisionSense
 *
 * Publishes IMU (accelerometer, gyroscope, magnetometer) and GPS data from BerryGPS-IMU v4
 * Hardware:
 * - LSM6DSL: Accelerometer + Gyroscope (I2C 0x6A)
 * - LIS3MDL: Magnetometer (I2C 0x1C)
 * - GPS: via gpsd (supports NMEA/UBX protocols transparently)
 *
 * Topics:
 *   /imu/data - sensor_msgs/Imu (accel + gyro)
 *   /imu/mag - sensor_msgs/MagneticField
 *   /gps/fix - sensor_msgs/NavSatFix
 *   /gps/velocity - geometry_msgs/TwistStamped
 */

#include "ros_compat.h"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <gps.h>

// Save gpsd constants before undefining macros that clash with ROS2 NavSatStatus
static constexpr int GPSD_MODE_NOT_SEEN = MODE_NOT_SEEN;
static constexpr int GPSD_MODE_2D = MODE_2D;
#undef STATUS_NO_FIX
#undef STATUS_FIX
#undef STATUS_DGPS_FIX

// LSM6DSL (Accel/Gyro) I2C addresses and registers
#define LSM6DSL_ADDR        0x6A
#define LSM6DSL_WHO_AM_I    0x0F
#define LSM6DSL_CTRL1_XL    0x10  // Accel control
#define LSM6DSL_CTRL2_G     0x11  // Gyro control
#define LSM6DSL_OUTX_L_G    0x22  // Gyro X low byte
#define LSM6DSL_OUTX_L_XL   0x28  // Accel X low byte

// LIS3MDL (Magnetometer) I2C addresses and registers
#define LIS3MDL_ADDR        0x1C
#define LIS3MDL_WHO_AM_I    0x0F
#define LIS3MDL_CTRL_REG1   0x20
#define LIS3MDL_CTRL_REG2   0x21
#define LIS3MDL_CTRL_REG3   0x22
#define LIS3MDL_CTRL_REG4   0x23
#define LIS3MDL_OUT_X_L     0x28

// Sensor scaling factors
#define ACCEL_SCALE         0.122 * 9.81 / 1000.0  // ±4g: 0.122 mg/LSB -> m/s²
#define GYRO_SCALE          8.75 * M_PI / (180.0 * 1000.0)  // 245 dps: 8.75 mdps/LSB -> rad/s
#define MAG_SCALE           1.0 / 6842.0 * 0.0001  // ±4 gauss: 6842 LSB/gauss -> Tesla

// Globals
Publisher<sensor_msgs::msg::Imu> imu_pub = NULL;
Publisher<sensor_msgs::msg::MagneticField> mag_pub = NULL;
Publisher<sensor_msgs::msg::NavSatFix> gps_pub = NULL;
Publisher<geometry_msgs::msg::TwistStamped> vel_pub = NULL;

int i2c_fd = -1;
bool gps_available = false;
struct gps_data_t gpsd_session;

// I2C helper functions
bool i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value)
{
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        return false;
    }

    uint8_t buf[2] = {reg, value};
    if (write(i2c_fd, buf, 2) != 2) {
        return false;
    }
    return true;
}

uint8_t i2c_read_byte(uint8_t addr, uint8_t reg)
{
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        return 0;
    }

    if (write(i2c_fd, &reg, 1) != 1) {
        return 0;
    }

    uint8_t value;
    if (read(i2c_fd, &value, 1) != 1) {
        return 0;
    }

    return value;
}

int16_t i2c_read_word(uint8_t addr, uint8_t reg)
{
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        return 0;
    }

    if (write(i2c_fd, &reg, 1) != 1) {
        return 0;
    }

    uint8_t buf[2];
    if (read(i2c_fd, buf, 2) != 2) {
        return 0;
    }

    return (int16_t)((buf[1] << 8) | buf[0]);
}

// Initialize LSM6DSL (Accel/Gyro)
bool init_lsm6dsl()
{
    // Check WHO_AM_I
    uint8_t who_am_i = i2c_read_byte(LSM6DSL_ADDR, LSM6DSL_WHO_AM_I);
    if (who_am_i != 0x6A) {
        ROS_ERROR("LSM6DSL not found (WHO_AM_I = 0x%02X)", who_am_i);
        return false;
    }

    // Configure accelerometer: 104 Hz, ±4g
    if (!i2c_write_byte(LSM6DSL_ADDR, LSM6DSL_CTRL1_XL, 0x40)) {
        ROS_ERROR("Failed to configure LSM6DSL accelerometer");
        return false;
    }

    // Configure gyroscope: 104 Hz, 245 dps
    if (!i2c_write_byte(LSM6DSL_ADDR, LSM6DSL_CTRL2_G, 0x40)) {
        ROS_ERROR("Failed to configure LSM6DSL gyroscope");
        return false;
    }

    ROS_INFO("LSM6DSL initialized (Accel/Gyro)");
    return true;
}

// Initialize LIS3MDL (Magnetometer)
bool init_lis3mdl()
{
    // Check WHO_AM_I
    uint8_t who_am_i = i2c_read_byte(LIS3MDL_ADDR, LIS3MDL_WHO_AM_I);
    if (who_am_i != 0x3D) {
        ROS_WARN("LIS3MDL not found (WHO_AM_I = 0x%02X), magnetometer disabled", who_am_i);
        return false;
    }

    // Configure: Temperature sensor enabled, high-performance mode, 10 Hz
    i2c_write_byte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x70);

    // Configure: ±4 gauss scale
    i2c_write_byte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x00);

    // Configure: Continuous conversion mode
    i2c_write_byte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00);

    // Configure: Z-axis high-performance mode
    i2c_write_byte(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, 0x0C);

    ROS_INFO("LIS3MDL initialized (Magnetometer)");
    return true;
}

// Read accelerometer data
void read_accelerometer(double& ax, double& ay, double& az)
{
    if (ioctl(i2c_fd, I2C_SLAVE, LSM6DSL_ADDR) < 0) {
        ROS_ERROR("Failed to set I2C slave address for accelerometer");
        return;
    }

    uint8_t reg = LSM6DSL_OUTX_L_XL;
    if (write(i2c_fd, &reg, 1) != 1) {
        ROS_ERROR("Failed to write register address for accelerometer");
        return;
    }

    uint8_t data[6];
    if (read(i2c_fd, data, 6) != 6) {
        ROS_ERROR("Failed to read accelerometer data");
        return;
    }

    // Convert from little-endian bytes to int16
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    ax = raw_x * ACCEL_SCALE;
    ay = raw_y * ACCEL_SCALE;
    az = raw_z * ACCEL_SCALE;

    ROS_DEBUG("Accel raw: [%d, %d, %d] -> scaled: [%.4f, %.4f, %.4f]", raw_x, raw_y, raw_z, ax, ay, az);
}

// Read gyroscope data
void read_gyroscope(double& gx, double& gy, double& gz)
{
    if (ioctl(i2c_fd, I2C_SLAVE, LSM6DSL_ADDR) < 0) {
        return;
    }

    uint8_t reg = LSM6DSL_OUTX_L_G;
    if (write(i2c_fd, &reg, 1) != 1) {
        return;
    }

    uint8_t data[6];
    if (read(i2c_fd, data, 6) != 6) {
        return;
    }

    // Convert from little-endian bytes to int16
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    gx = raw_x * GYRO_SCALE;
    gy = raw_y * GYRO_SCALE;
    gz = raw_z * GYRO_SCALE;
}

// Read magnetometer data
void read_magnetometer(double& mx, double& my, double& mz)
{
    if (ioctl(i2c_fd, I2C_SLAVE, LIS3MDL_ADDR) < 0) {
        return;
    }

    uint8_t reg = LIS3MDL_OUT_X_L;
    if (write(i2c_fd, &reg, 1) != 1) {
        return;
    }

    uint8_t data[6];
    if (read(i2c_fd, data, 6) != 6) {
        return;
    }

    // Convert from little-endian bytes to int16
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    mx = raw_x * MAG_SCALE;
    my = raw_y * MAG_SCALE;
    mz = raw_z * MAG_SCALE;
}

// Initialize gpsd connection
bool init_gps(const std::string& host, const std::string& port)
{
    if (gps_open(host.c_str(), port.c_str(), &gpsd_session) != 0) {
        ROS_WARN("Failed to connect to gpsd at %s:%s", host.c_str(), port.c_str());
        return false;
    }

    gps_stream(&gpsd_session, WATCH_ENABLE | WATCH_JSON, NULL);

    ROS_INFO("Connected to gpsd at %s:%s", host.c_str(), port.c_str());
    return true;
}

// GPS state
double last_lat = 0, last_lon = 0, last_altitude = 0, last_gps_speed = 0;
int last_fix_mode = GPSD_MODE_NOT_SEEN;

// Read GPS data from gpsd
void read_gps()
{
    // Check if data is waiting (0ms timeout = non-blocking)
    while (gps_waiting(&gpsd_session, 0)) {
        if (gps_read(&gpsd_session, NULL, 0) == -1) {
            ROS_WARN("gpsd connection lost, attempting reconnect...");
            gps_close(&gpsd_session);
            gps_available = false;
            return;
        }

        // Only update fix mode from real fix reports (ignore MODE_NOT_SEEN=0)
        if ((gpsd_session.set & MODE_SET) && gpsd_session.fix.mode > GPSD_MODE_NOT_SEEN) {
            last_fix_mode = gpsd_session.fix.mode;
        }

        if (gpsd_session.set & LATLON_SET) {
            last_lat = gpsd_session.fix.latitude;
            last_lon = gpsd_session.fix.longitude;
        }

        if (gpsd_session.set & ALTITUDE_SET) {
            last_altitude = gpsd_session.fix.altMSL;
        }

        if (gpsd_session.set & SPEED_SET) {
            last_gps_speed = gpsd_session.fix.speed;
        }

    }
}

// Global state for publish loop
bool mag_available = false;
bool imu_available_global = false;

// Publish callback
void publish_sensors()
{
    // Read IMU data
    double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (imu_available_global) {
        read_accelerometer(ax, ay, az);
        read_gyroscope(gx, gy, gz);

        // Log raw values
        static int log_counter = 0;
        if (log_counter++ % 50 == 0) {  // Log every 50 samples to reduce spam
            ROS_INFO("IMU Raw Data - Accel: [%.2f, %.2f, %.2f] m/s², Gyro: [%.4f, %.4f, %.4f] rad/s",
                    ax, ay, az, gx, gy, gz);
        }
    } else {
        static bool warned = false;
        if (!warned) {
            ROS_WARN("IMU not available - publishing zero values");
            warned = true;
        }
    }

    // Publish IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = ROS_TIME_NOW();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    // Orientation not available (no fusion)
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation_covariance[0] = -1.0;  // Mark as invalid

    ROS_DEBUG("Publishing IMU message: accel=[%.2f, %.2f, %.2f], gyro=[%.4f, %.4f, %.4f]",
             ax, ay, az, gx, gy, gz);
    imu_pub->publish(imu_msg);

    // Read and publish magnetometer if available
    if (mag_available) {
        double mx, my, mz;
        read_magnetometer(mx, my, mz);

        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = imu_msg.header.stamp;
        mag_msg.header.frame_id = "imu_link";

        mag_msg.magnetic_field.x = mx;
        mag_msg.magnetic_field.y = my;
        mag_msg.magnetic_field.z = mz;

        mag_pub->publish(mag_msg);
    }

    // Read GPS from gpsd
    if (gps_available) {
        read_gps();
    }

    // Publish GPS fix
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = imu_msg.header.stamp;
    gps_msg.header.frame_id = "gps_link";

    gps_msg.latitude = last_lat;
    gps_msg.longitude = last_lon;
    gps_msg.altitude = last_altitude;

    if (last_fix_mode >= GPSD_MODE_2D) {
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    } else {
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    gps_pub->publish(gps_msg);

    // Publish velocity
    geometry_msgs::msg::TwistStamped vel_msg;
    vel_msg.header.stamp = imu_msg.header.stamp;
    vel_msg.header.frame_id = "gps_link";
    vel_msg.twist.linear.x = last_gps_speed;

    vel_pub->publish(vel_msg);
}

int main(int argc, char** argv)
{
    ROS_CREATE_NODE("imu_gps");

    ROS_INFO("IMU-GPS Node - Starting...");

    // Parameters
    int i2c_bus = 7;
    std::string gpsd_host = "localhost";
    std::string gpsd_port = DEFAULT_GPSD_PORT;

    ROS_DECLARE_PARAMETER("i2c_bus", i2c_bus);
    ROS_DECLARE_PARAMETER("gpsd_host", gpsd_host);
    ROS_DECLARE_PARAMETER("gpsd_port", gpsd_port);

    ROS_GET_PARAMETER_OR("i2c_bus", i2c_bus, 7);
    ROS_GET_PARAMETER_OR("gpsd_host", gpsd_host, std::string("localhost"));
    ROS_GET_PARAMETER_OR("gpsd_port", gpsd_port, std::string(DEFAULT_GPSD_PORT));

    // Open I2C bus
    char i2c_device[32];
    snprintf(i2c_device, sizeof(i2c_device), "/dev/i2c-%d", i2c_bus);
    i2c_fd = open(i2c_device, O_RDWR);

    if (i2c_fd < 0) {
        ROS_ERROR("Failed to open I2C bus %d: %s", i2c_bus, strerror(errno));
        return 1;
    }

    ROS_INFO("Opened I2C bus %d", i2c_bus);

    // Initialize IMU sensors
    imu_available_global = init_lsm6dsl();
    if (!imu_available_global) {
        ROS_WARN("LSM6DSL not available - will publish zero IMU values");
    }

    mag_available = init_lis3mdl();  // Magnetometer is optional

    // Initialize GPS via gpsd
    gps_available = init_gps(gpsd_host, gpsd_port);

    // Create publishers
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::Imu, "imu/data", 10, imu_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::MagneticField, "imu/mag", 10, mag_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::msg::NavSatFix, "gps/fix", 10, gps_pub);
    ROS_CREATE_PUBLISHER(geometry_msgs::msg::TwistStamped, "gps/velocity", 10, vel_pub);

    ROS_INFO("IMU-GPS node started");
    ROS_INFO("Publishing to:");
    ROS_INFO("  - /imu/data (sensor_msgs/Imu)");
    ROS_INFO("  - /imu/mag (sensor_msgs/MagneticField)");
    ROS_INFO("  - /gps/fix (sensor_msgs/NavSatFix)");
    ROS_INFO("  - /gps/velocity (geometry_msgs/TwistStamped)");

    // Get publish rate parameter
    int publish_rate = 50;
    ROS_DECLARE_PARAMETER("publish_rate", publish_rate);
    ROS_GET_PARAMETER_OR("publish_rate", publish_rate, 50);

    auto period = std::chrono::milliseconds(1000 / publish_rate);

    // Main loop
    while (ROS_OK()) {
        auto start_time = std::chrono::steady_clock::now();

        // Publish sensor data
        publish_sensors();

        // Sleep to maintain publish rate
        ROS_SPIN_ONCE();
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto sleep_time = period - elapsed;
        if (sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    // Cleanup
    if (i2c_fd >= 0) close(i2c_fd);
    if (gps_available) {
        gps_stream(&gpsd_session, WATCH_DISABLE, NULL);
        gps_close(&gpsd_session);
    }

    ROS_INFO("IMU-GPS node shutting down");
    return 0;
}
