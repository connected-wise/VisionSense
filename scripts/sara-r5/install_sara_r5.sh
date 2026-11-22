#!/bin/bash
# Set up the OzzMaker SARA-R5 LTE-M GPS + 10DOF board on the Jetson 40-pin
# UART (/dev/ttyTHS1): CMUX splits the UART into channels, PPP gives an LTE-M
# data link on /dev/gsmtty2 (the NetworkManager connection "LTE") and gpsd
# reads the GNSS stream on /dev/gsmtty4.
# The IMU sensors are on I2C and need nothing here (imu_gps reads them).
#
# Run with: bash scripts/sara-r5/install_sara_r5.sh
# The SIM's APN is stored in the modem (AT+CGDCONT). To set it here:
# VS_LTE_APN=<apn> bash scripts/sara-r5/install_sara_r5.sh
# (Hologram: hologram; Tello: wholesale)
# The modem registers on AT&T (310410) when it can. VS_LTE_OPERATOR=<MCC+MNC>
# picks another network; VS_LTE_OPERATOR= (empty) lets the modem choose.
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "SARA-R5: installing build tools, ppp and gpsd..."
sudo apt-get install -y build-essential curl ppp gpsd gpsd-clients

echo "SARA-R5: n_gsm kernel module..."
bash "${DIR}/build_n_gsm.sh"
echo n_gsm | sudo tee /etc/modules-load.d/n_gsm.conf > /dev/null

echo "SARA-R5: mux daemon and start script..."
gcc -O2 -Wall -o /tmp/sara-r5-mux "${DIR}/sara-r5-mux.c"
sudo install -m 0755 /tmp/sara-r5-mux /usr/local/bin/sara-r5-mux
rm -f /tmp/sara-r5-mux
sudo install -m 0755 "${DIR}/sara-r5-start" /usr/local/bin/sara-r5-start
sudo install -m 0644 "${DIR}/99-gsmtty.rules" /etc/udev/rules.d/99-gsmtty.rules
sudo udevadm control --reload-rules

echo "SARA_R5_OPERATOR=${VS_LTE_OPERATOR-310410}" | sudo tee /etc/default/sara-r5 > /dev/null

echo "SARA-R5: PPP..."
sudo install -m 0640 -g dip "${DIR}/ppp-peer-sara-r5" /etc/ppp/peers/sara-r5
if [ -n "${VS_LTE_APN:-}" ]; then
    sed "s#^\"\" AT\$#\"\" AT\nOK AT+CGDCONT=1,\"IPV4V6\",\"${VS_LTE_APN}\"#" \
        "${DIR}/chat-sara-r5" > /tmp/chat-sara-r5
else
    cp "${DIR}/chat-sara-r5" /tmp/chat-sara-r5
fi
sudo install -D -m 0644 /tmp/chat-sara-r5 /etc/chatscripts/sara-r5
rm -f /tmp/chat-sara-r5

echo "SARA-R5: gpsd on /dev/gsmtty4..."
# -b: read-only, so gpsd never reconfigures the receiver behind the modem.
sudo tee /etc/default/gpsd > /dev/null << 'GPSDCONF'
# GNSS of the SARA-R5, streamed on mux channel 4 (see sara-r5-mux.service).
DEVICES="/dev/gsmtty4"

# -n: poll the receiver without waiting for a client; -b: read-only
GPSD_OPTIONS="-n -b"

USBAUTO="false"
GPSDCONF
sudo install -D -m 0644 "${DIR}/gpsd-sara-r5.conf" /etc/systemd/system/gpsd.service.d/sara-r5.conf
# Ubuntu's gpsd AppArmor profile only allows /dev/tty{S,USB,AMA,ACM}*
GPSD_AA_LOCAL=/etc/apparmor.d/local/usr.sbin.gpsd
if [ -f /etc/apparmor.d/usr.sbin.gpsd ]; then
    if ! sudo grep -qs gsmtty "$GPSD_AA_LOCAL"; then
        echo "/dev/gsmtty[0-9]* rw," | sudo tee -a "$GPSD_AA_LOCAL" > /dev/null
    fi
    sudo apparmor_parser -r /etc/apparmor.d/usr.sbin.gpsd || true
fi

echo "SARA-R5: services..."
sudo install -m 0644 "${DIR}/sara-r5-mux.service" /etc/systemd/system/sara-r5-mux.service
sudo install -m 0644 "${DIR}/sara-r5-ppp.service" /etc/systemd/system/sara-r5-ppp.service
sudo systemctl daemon-reload
sudo systemctl enable sara-r5-mux.service gpsd.service
sudo systemctl restart sara-r5-mux.service
sudo systemctl restart gpsd.service

echo "SARA-R5: NetworkManager LTE connection..."
# NetworkManager owns the link, so it shows in the network menu and counts as
# online when Wi-Fi is off. Its device handler starts sara-r5-ppp. Route
# metric 700 keeps Wi-Fi (600) and Ethernet (100) preferred while they are up.
sudo install -D -m 0755 "${DIR}/nm-device-handler-sara-r5" \
    /etc/NetworkManager/dispatcher.d/device/sara-r5
if ! nmcli -t -f NAME connection show | grep -qx LTE; then
    sudo nmcli connection add type generic con-name LTE ifname ppp0 \
        generic.device-handler sara-r5 \
        ipv4.method manual ipv4.addresses 192.168.201.1/32 \
        ipv4.gateway 192.168.201.2 ipv4.route-metric 700 \
        ipv4.dns "8.8.8.8 1.1.1.1" ipv6.method disabled \
        connection.autoconnect yes connection.autoconnect-retries 0
fi
sudo nmcli connection up LTE || echo "  (LTE did not come up yet — NetworkManager keeps retrying)"
echo "✓ SARA-R5 set up: LTE connection in NetworkManager, GNSS via gpsd"
