/*
 * SPI SPL check tool.
 *
 * Copyright (C) 2013 Ingenic Semiconductor Co.,Ltd
 * Based on: u-boot-1.1.6/tools/spi_checksum/spi_checksum.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <config.h>

#define BUFFER_SIZE 4
#ifdef CONFIG_M200
#define SKIP_SIZE 2048
#endif
#ifdef CONFIG_JZ4780
#define SKIP_SIZE 16
#endif
#define le(a) (((a & 0xff)<<24) | ((a>>8 & 0xff)<< 16) | ((a>>16 & 0xff)<< 8) | ((a>>24 & 0xff)))

int main(int argc, char *argv[])
{
	int fd, count;
	int bytes_read;
	char buffer[BUFFER_SIZE];
	unsigned int check = 0;
	volatile int t = 0;
	
	if (argc != 2) {
		printf("Usage: %s fromfile tofile\n\a",argv[0]);
		return 1;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("Open %s Error\n", argv[1]);
		return 1;
	}

	count = 0;

	while ((bytes_read = read(fd, buffer, BUFFER_SIZE)) > 0) {
		if (t >= SKIP_SIZE)
			check += *((unsigned int *)buffer);
		else 	
			t += BUFFER_SIZE;
		count += bytes_read;
	}

	printf("spi spl count = %d \n", count);
	printf("spi spl check = %#x \n", check);

	lseek( fd, 8, SEEK_SET);
	
	if ((t = write(fd, &count, 4)) != 4) {
		printf("Write %s Error\n",argv[1]);
		return 1;
	}

	check = 0 - check;
	if ((t = write(fd, &check, 4)) != 4) {
		printf("Check: Write %s Error\n",argv[1]);
		return 1;}

#if 0
	lseek( fd, 8, SEEK_SET);

	if ((t = read(fd,buffer,BUFFER_SIZE) < 0)) {
		printf("read %d \n",t);
	}
	printf("%#x\t", *(unsigned int *)buffer);

	if ((t = read(fd,buffer,BUFFER_SIZE) < 0)) {
		printf("read %d \n",t);
	}
	printf("%#x\n", *(unsigned int *)buffer);
	
#endif
	close(fd);

	return 0;
}
