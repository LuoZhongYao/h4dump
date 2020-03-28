/*
 * Written by ZhongYao Luo <luozhongyao@gmail.com>
 *
 * Copyright 2020 ZhongYao Luo
 */
#include <sys/ioctl.h>
#include <asm/termbits.h>

int set_baudrate(int fd, int speed)
{
	struct termios2 t;

	if (ioctl(fd, TCGETS2, &t)) {
		return -1;
	}

	t.c_cflag &= ~CBAUD;
	t.c_cflag |= BOTHER;
	t.c_ispeed = speed;
	t.c_ospeed = speed;

	return ioctl(fd, TCSETS2, &t);
}

