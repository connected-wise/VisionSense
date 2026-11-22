// SPDX-License-Identifier: GPL-2.0
/*
 * sara-r5-mux: switch the u-blox SARA-R5 on a serial port into 3GPP 27.010
 * (CMUX) mode and attach the kernel n_gsm line discipline, which creates
 * /dev/gsmtty1..N, one per mux channel (DLCI). Stays in the foreground and
 * holds the port open; closing it (SIGTERM) shuts the mux down.
 *
 * With the SARA-R5 serial variant 0 (AT+USIO=0, the default):
 *   /dev/gsmtty1     AT commands (sara-r5-start)
 *   /dev/gsmtty2     PPP data (sara-r5-ppp)
 *   /dev/gsmtty3     AT commands, unused
 *   /dev/gsmtty4     GNSS NMEA stream (needs AT+UGPRF=2, read by gpsd)
 *
 * Usage: sara-r5-mux [device] [baud]   (default /dev/ttyTHS1 115200)
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/gsmmux.h>
#include <linux/tty.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define MUX_FRAME 127	/* N1, max frame payload; must match AT+CMUX */

static volatile sig_atomic_t stop;

static void on_signal(int sig) { (void)sig; stop = 1; }

static speed_t baud_flag(int baud)
{
	switch (baud) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 230400: return B230400;
	case 460800: return B460800;
	default: return B115200;
	}
}

/* Send cmd and wait up to ms for a line containing "OK". */
static int at(int fd, const char *cmd, int ms)
{
	char buf[512];
	size_t len = 0;

	tcflush(fd, TCIFLUSH);
	if (write(fd, cmd, strlen(cmd)) < 0 || write(fd, "\r", 1) < 0)
		return -1;
	while (ms > 0) {
		struct pollfd p = { .fd = fd, .events = POLLIN };
		ssize_t n;

		if (poll(&p, 1, 100) <= 0) {
			ms -= 100;
			continue;
		}
		n = read(fd, buf + len, sizeof(buf) - 1 - len);
		if (n <= 0)
			continue;
		len += n;
		buf[len] = 0;
		if (strstr(buf, "OK"))
			return 0;
		if (strstr(buf, "ERROR"))
			return -1;
		if (len >= sizeof(buf) - 1)
			len = 0;
	}
	return -1;
}

int main(int argc, char **argv)
{
	const char *dev = argc > 1 ? argv[1] : "/dev/ttyTHS1";
	int baud = argc > 2 ? atoi(argv[2]) : 115200;
	char cmux[64];
	struct termios t;
	struct gsm_config c;
	int ldisc = N_GSM0710;
	int fd, i;

	fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
		return 1;
	}
	tcgetattr(fd, &t);
	cfmakeraw(&t);
	t.c_cflag |= CLOCAL | CREAD;
	t.c_cflag &= ~CRTSCTS;
	cfsetspeed(&t, baud_flag(baud));
	tcsetattr(fd, TCSANOW, &t);

	/*
	 * The modem stays in mux mode after a previous session ends, and does
	 * not answer AT then. Close its mux down: open the control channel
	 * (SABM on DLCI 0), then send the close-down command (CLD).
	 */
	for (i = 0; i < 3 && at(fd, "AT", 1000) != 0; i++) {
		static const unsigned char sabm0[] = { 0xF9, 0x03, 0x3F, 0x01, 0x1C, 0xF9 };
		static const unsigned char cld[] = { 0xF9, 0x03, 0xEF, 0x05, 0xC3, 0x01, 0xF2, 0xF9 };

		printf("no AT response on %s, closing a leftover CMUX session\n", dev);
		fflush(stdout);
		if (write(fd, sabm0, sizeof(sabm0)) < 0)
			break;
		usleep(500000);
		if (write(fd, cld, sizeof(cld)) < 0)
			break;
		sleep(1);
	}
	if (i == 3) {
		fprintf(stderr, "no AT response on %s\n", dev);
		return 1;
	}
	at(fd, "ATE0", 1000);
	snprintf(cmux, sizeof(cmux), "AT+CMUX=0,0,,%d", MUX_FRAME);
	if (at(fd, cmux, 3000) != 0) {
		fprintf(stderr, "%s failed\n", cmux);
		return 1;
	}

	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		fprintf(stderr, "TIOCSETD N_GSM0710: %s (is n_gsm loaded?)\n", strerror(errno));
		return 1;
	}
	if (ioctl(fd, GSMIOC_GETCONF, &c) < 0) {
		fprintf(stderr, "GSMIOC_GETCONF: %s\n", strerror(errno));
		return 1;
	}
	c.initiator = 1;
	c.encapsulation = 0;	/* basic option */
	c.mru = MUX_FRAME;
	c.mtu = MUX_FRAME;
	if (ioctl(fd, GSMIOC_SETCONF, &c) < 0) {
		fprintf(stderr, "GSMIOC_SETCONF: %s\n", strerror(errno));
		return 1;
	}
	/*
	 * Wait until the mux is usable: the device nodes appear shortly after
	 * the mux starts, and opens fail with EL2NSYNC until the control
	 * channel is up.
	 */
	for (i = 0; i < 100; i++) {
		int ch = open("/dev/gsmtty1", O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (ch >= 0) {
			close(ch);
			break;
		}
		if (errno != ENOENT && errno != EL2NSYNC) {
			fprintf(stderr, "open /dev/gsmtty1: %s\n", strerror(errno));
			return 1;
		}
		usleep(100000);
	}

	printf("CMUX up on %s (%d baud, N1=%d)\n", dev, baud, MUX_FRAME);
	fflush(stdout);

	signal(SIGTERM, on_signal);
	signal(SIGINT, on_signal);
	while (!stop)
		pause();

	close(fd);	/* n_gsm sends DISC on DLCI 0, ending mux mode */
	return 0;
}
