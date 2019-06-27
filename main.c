#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <poll.h>
#include <stdbool.h>
#include <getopt.h>
#include <limits.h>
#include "pt-1.4/pt.h"
#include "parser/parser.h"

#define HCI_COMMAND_PKT     0x01
#define HCI_ACLDATA_PKT     0x02
#define HCI_SCODATA_PKT     0x03
#define HCI_EVENT_PKT       0x04
#define HCI_DIAG_PKT        0xf0
#define HCI_VENDOR_PKT      0xff

#define HCI_COMMAND_HDR_SIZE 3
#define HCI_EVENT_HDR_SIZE   2
#define HCI_ACL_HDR_SIZE     4
#define HCI_SCO_HDR_SIZE     3

#define HCI_MAX_ACL_SIZE    1024
#define HCI_MAX_SCO_SIZE    255
#define HCI_MAX_COMMAND_SIZE 260
#define HCI_MAX_EVENT_SIZE   260
#define HCI_MAX_FRAME_SIZE  (HCI_MAX_ACL_SIZE + 4)
static const uint64_t BTSNOOP_EPOCH_DELTA = 0x00dcddb30f2f8000ULL;

static const struct h4_pkt_match {
    uint8_t type;
    uint8_t hlen;
    uint8_t loff;
    uint8_t lsize;
    uint16_t maxlen;
} h4_pkts [] = {
	{0, 0, 0, 0, 0 },   /* 0 */
	{
		.type = HCI_COMMAND_PKT,
		.hlen = HCI_COMMAND_HDR_SIZE,
		.loff = 2,
		.lsize = 1,
		.maxlen = HCI_MAX_COMMAND_SIZE,
	},   /* hci command */

	{
		.type = HCI_ACLDATA_PKT, 
		.hlen = HCI_ACL_HDR_SIZE, 
		.loff = 2, 
		.lsize = 2, 
		.maxlen = HCI_MAX_FRAME_SIZE 
	},  /* acl data*/

	{
		.type = HCI_SCODATA_PKT, 
		.hlen = HCI_SCO_HDR_SIZE, 
		.loff = 2, 
		.lsize = 1, 
		.maxlen = HCI_MAX_SCO_SIZE
	},  /* sco data */

	{
		.type = HCI_EVENT_PKT, 
		.hlen = HCI_EVENT_HDR_SIZE, 
		.loff = 1, 
		.lsize = 1, 
		.maxlen = HCI_MAX_EVENT_SIZE
	}  /* hci event */
};

static int btsnoop_create(const char *file)
{
	int fd;
	if (file == NULL)
		return -1;

	fd = open(file, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
	if (fd < 0) {
		perror(file);
		exit(1);
	}
	write(fd, "btsnoop\0\0\0\0\1\0\0\x3\xea", 16);
	return fd;
}

static uint64_t btsnoop_timestamp(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  // Timestamp is in microseconds.
  uint64_t timestamp = tv.tv_sec * 1000 * 1000LL;
  timestamp += tv.tv_usec;
  timestamp += BTSNOOP_EPOCH_DELTA;
  return timestamp;
}

static void btsnoop_write(int fd, const uint8_t *buf, bool is_received)
{
	int length;
	int flags;
	int drops = 0;
	int length_he = 0;
	int type = buf[0];
	const uint8_t *packet = buf + 1;

	switch (type) {
	case HCI_COMMAND_PKT:
		length_he = packet[2] + 4;
		flags = 2;
	break;
	case HCI_ACLDATA_PKT:
		length_he = (packet[3] << 8) + packet[2] + 5;
		flags = is_received;
	break;
	case HCI_SCODATA_PKT:
		length_he = packet[2] + 4;
		flags = is_received;
	break;
	case HCI_EVENT_PKT:
		length_he = packet[1] + 3;
		flags = 3;
	break;
	default: return ;
	}

	uint64_t timestamp = btsnoop_timestamp();
	uint32_t time_hi = timestamp >> 32;
	uint32_t time_lo = timestamp & 0xFFFFFFFF;

	length = htonl(length_he);
	flags = htonl(flags);
	drops = htonl(drops);
	time_hi = htonl(time_hi);
	time_lo = htonl(time_lo);

	write(fd, &length, 4);
	write(fd, &length, 4);
	write(fd, &flags, 4);
	write(fd, &drops, 4);
	write(fd, &time_hi, 4);
	write(fd, &time_lo, 4);
	write(fd, &type, 1);
	write(fd, packet, length_he - 1);
}

static speed_t tty_get_speed(int speed)
{
#define _(n) case n: return B ##n
	switch (speed) {
		_(9600); break;
		_(38400); break;
		_(115200); break;
		_(230400); break;
		_(460800); break;
		_(576000); break;
		_(921600); break;
		_(1000000); break;
		_(1152000); break;
		_(1500000); break;
		_(2000000); break;
	}
#undef _
	return speed;
}

static int uart_open(const char *dev, int speed)
{
	int fd;
	struct termios ti;

	if (dev == NULL)
		return -1;

	fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		perror(dev);
		exit(1);
	}

	tcflush(fd, TCIOFLUSH);

	if (tcgetattr(fd, &ti) < 0) {
		perror("get port settings");
		exit(1);
	}
	cfmakeraw(&ti);
	ti.c_cflag |= CLOCAL;
	ti.c_cflag &= ~CRTSCTS;
	ti.c_cflag &= ~PARENB;
	ti.c_cflag &= ~CSTOPB;
	ti.c_cflag |= CS8;
	ti.c_cc[VMIN] = 1;
	cfsetispeed(&ti, tty_get_speed(speed));
	cfsetospeed(&ti, tty_get_speed(speed));

	if (tcsetattr(fd, TCSANOW, &ti) < 0) {
		perror("set port settings");
		exit(1);
	}

	tcflush(fd, TCIOFLUSH);

	return fd;
}

struct context
{
	int fd;
	bool in;
	int btsnoop;
	struct pt pt;
	unsigned curn;
	unsigned readn;
	struct frame frm;
	unsigned char buf[1024];
};

#define READ_BYTE(h4, buf, length) \
    do {\
        PT_WAIT_UNTIL(&h4->pt, length > 0);\
        int total = (h4->curn + length < h4->readn) ? length : h4->readn - h4->curn;\
        memcpy(h4->buf + h4->curn, buf, total);\
        buf += total; \
        length -= total; \
        h4->curn += total;\
    } while(h4->curn < h4->readn)

static unsigned short h4_pkt_len(const unsigned char *pkt, const struct h4_pkt_match *match)
{
    switch(match->lsize) {
    case 0:
        return 0;
    break;
    
    case 1: {
        return pkt[match->loff + 1];
    } break;

    case 2: {
        return pkt[match->loff + 1] | pkt[match->loff + 2] << 8;
    } break;

    }
    return USHRT_MAX;
}


static PT_THREAD(h4_process(struct context *c, void *buf, unsigned size))
{
	int rn = 0;
	unsigned dlen;

	PT_BEGIN(&c->pt);

	while (1) {
		c->readn = 1;
		c->curn = 0;
		READ_BYTE(c, buf, size);

		if (c->buf[0] > HCI_EVENT_PKT)
			continue;

		if (c->buf[0] == HCI_EVENT_PKT)
			c->in = true;
		if (c->buf[0] == HCI_COMMAND_PKT)
			c->in = false;

		c->readn += h4_pkts[c->buf[0]].hlen;
		READ_BYTE(c, buf, size);
		dlen = h4_pkt_len(c->buf, h4_pkts + c->buf[0]);
        if (dlen == USHRT_MAX)
            continue;

        c->readn += dlen;
        READ_BYTE(c, buf, size);

		if (c->btsnoop != -1) {
			btsnoop_write(c->btsnoop, c->buf, c->in);
		}

		c->frm.in = c->in;
		c->frm.data_len = c->readn;
		c->frm.ptr = c->frm.data;
		c->frm.len = c->frm.data_len;
		hci_dump(0, &c->frm);
	}

	PT_END(&c->pt);
}

static void context_init(struct context *c, int fd, int btsnoop, bool in)
{
	memset(c, 0, sizeof(*c));
	c->fd = fd;
	c->in = in;
	c->btsnoop = btsnoop;
	c->frm.in = false;
	c->frm.data = c->buf;
	PT_INIT(&c->pt);
}

static int process_frames(int tx, int rx, int btsnoop)
{
	int nfds = 0;
	unsigned char buf[1024];
	struct pollfd fds[2];
	struct context ctx[2];
	struct context *c;

	if (tx != -1) {
		context_init(ctx + 0, tx, btsnoop, false);
		fds[nfds].events = POLLIN;
		fds[nfds].revents = 0;
		fds[nfds].fd = tx;
		nfds++;
	}

	if (rx != -1) {
		context_init(ctx + 1, rx, btsnoop, true);
		fds[nfds].events = POLLIN;
		fds[nfds].revents = 0;
		fds[nfds].fd = rx;
		nfds++;
	}

	while (1) {
		int n = poll(fds, nfds, -1), i;
		if ( n <= 0)
			continue;

		for (i = 0;i < nfds;i++) {
			if (fds[i].revents & (POLLHUP | POLLERR | POLLNVAL)) {
				goto quit;
			} else if (fds[i].revents & POLLIN) {
				int rn;
				int fd = fds[i].fd;
				if (fd == ctx[0].fd) {
					c = ctx + 0;
				} else if (fd == ctx[1].fd) {
					c = ctx + 1;
				}

				rn = read(fd, buf, sizeof(buf));
				if (rn <= 0)
					continue;
				h4_process(c, buf, rn);
			}
		}
	}

quit:
	if (tx >= 0)
		close(tx);
	if (rx >= 0)
		close(rx);
	if (btsnoop >= 0)
		close(btsnoop);

	return 0;
}

static void usage(void)
{
	printf("usage: btuart -w {btsnoop} -r {rx} -t {tx} -b {baudrate} -h\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int c, speed = 115200;
	unsigned long flags = 0, filter = 0;
	const char *rx_file = NULL, *tx_file = NULL, *btsnoop_file = NULL;

	while (-1 != (c = getopt(argc, argv, "r:t:b:w:h"))) {
		switch (c) {
			 case 'r': rx_file = optarg; break;
			 case 't': tx_file = optarg; break;
			 case 'b': speed = strtol(optarg, NULL, 0); break;
			 case 'w': btsnoop_file = optarg; break;
			 case 'h':
			 default:
				usage();
			 break;
		}
	}

	init_parser(0, ~0L, 0, 0, -1, -1);
	if (!tx_file && !rx_file)
		usage();

	process_frames(uart_open(tx_file, speed), uart_open(rx_file, speed), btsnoop_create(btsnoop_file));
	return 0;
}
