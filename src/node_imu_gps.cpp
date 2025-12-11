/*
 * IMU-GPS Node for VisionSense
 *
 * Publishes IMU (accelerometer, gyroscope, magnetometer) and GPS data from BerryGPS-IMU v4
 * Hardware:
 * - LSM6DSL: Accelerometer + Gyroscope (I2C 0x6A)
 * - LIS3MDL: Magnetometer (I2C 0x1C)
 * - GPS: Serial NMEA sentences on /dev/ttyTHS1
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
#include <termios.h>
#include <cstring>

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
int gps_fd = -1;
bool gps_available = false;

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

// Initialize GPS serial port
bool init_gps(const std::string& port, int baudrate)
{
    gps_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (gps_fd < 0) {
        ROS_WARN("Failed to open GPS port %s: %s", port.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(gps_fd, &tty) != 0) {
        ROS_WARN("Failed to get GPS port attributes");
        close(gps_fd);
        gps_fd = -1;
        return false;
    }

    // Set baud rate
    speed_t speed = B9600;
    if (baudrate == 115200) speed = B115200;
    else if (baudrate == 57600) speed = B57600;
    else if (baudrate == 38400) speed = B38400;
    else if (baudrate == 19200) speed = B19200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(gps_fd, TCSANOW, &tty) != 0) {
        ROS_WARN("Failed to set GPS port attributes");
        close(gps_fd);
        gps_fd = -1;
        return false;
    }

    ROS_INFO("GPS initialized on %s at %d baud", port.c_str(), baudrate);
    return true;
}

// Parse NMEA GPRMC/GNRMC sentence
bool parse_gps_sentence(const std::string& sentence, double& lat, double& lon, double& speed)
{
    // Check if it's a GPRMC or GNRMC sentence
    if (sentence.find("$GPRMC") != 0 && sentence.find("$GNRMC") != 0) {
        return false;
    }

    // Split by comma
    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

    if (fields.size() < 10) {
        return false;
    }

    // Check validity (field 2)
    if (fields[2] != "A") {
        return false;
    }

    // Parse latitude (fields 3, 4)
    if (fields[3].length() < 4 || fields[4].empty()) {
        return false;
    }

    double lat_deg = std::stod(fields[3].substr(0, 2));
    double lat_min = std::stod(fields[3].substr(2));
    lat = lat_deg + lat_min / 60.0;
    if (fields[4] == "S") lat = -lat;

    // Parse longitude (fields 5, 6)
    if (fields[5].length() < 5 || fields[6].empty()) {
        return false;
    }

    double lon_deg = std::stod(fields[5].substr(0, 3));
    double lon_min = std::stod(fields[5].substr(3));
    lon = lon_deg + lon_min / 60.0;
    if (fields[6] == "W") lon = -lon;

    // Parse speed in knots (field 7), convert to m/s
    if (!fields[7].empty()) {
        speed = std::stod(fields[7]) * 0.514444;
    } else {
        speed = 0.0;
    }

    return true;
}

// Read GPS data
bool read_gps(double& lat, double& lon, double& speed)
{
    if (gps_fd < 0) return false;

    static std::string buffer;
    char buf[256];
    int n = read(gps_fd, buf, sizeof(buf) - 1);

    if (n > 0) {
        buf[n] = '\0';
        buffer += buf;

        // Look for complete NMEA sentences
        size_t end = buffer.find('\n');
        while (end != std::string::npos) {
            std::string sentence = buffer.substr(0, end);
            buffer = buffer.substr(end + 1);

            // Try to parse the sentence
            if (parse_gps_sentence(sentence, lat, lon, speed)) {
                return true;
            }

            end = buffer.find('\n');
        }
    }

    return false;
}

// Global state for publish loop
double last_lat = 0, last_lon = 0, last_gps_speed = 0;
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

    // Read and publish GPS
    bool gps_has_fix = gps_available && read_gps(last_lat, last_lon, last_gps_speed);

    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = imu_msg.header.stamp;
    gps_msg.header.frame_id = "gps_link";

    gps_msg.latitude = last_lat;
    gps_msg.longitude = last_lon;
    gps_msg.altitude = 0.0;  // Not available in GPRMC
    gps_msg.status.status = gps_has_fix ?
        sensor_msgs::msg::NavSatStatus::STATUS_FIX :
        sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
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
    std::string gps_port = "/dev/ttyTHS1";
    int gps_baud = 9600;

    ROS_DECLARE_PARAMETER("i2c_bus", i2c_bus);
    ROS_DECLARE_PARAMETER("gps_port", gps_port);
    ROS_DECLARE_PARAMETER("gps_baud", gps_baud);

    ROS_GET_PARAMETER_OR("i2c_bus", i2c_bus, 7);
    ROS_GET_PARAMETER_OR("gps_port", gps_port, std::string("/dev/ttyTHS1"));
    ROS_GET_PARAMETER_OR("gps_baud", gps_baud, 9600);

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

    // Initialize GPS
    gps_available = init_gps(gps_port, gps_baud);

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
    if (gps_fd >= 0) close(gps_fd);

    ROS_INFO("IMU-GPS node shutting down");
    return 0;
}
