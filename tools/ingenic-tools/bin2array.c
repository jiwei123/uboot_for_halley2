#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>

#define TEST       0
#define bufferLen  1024
char *name_str = "rle_default_logo_addr";
char *data_string = "__attribute__ ((section(\".data\")));";
void help()
{
	printf("binary ----> array\n\n");
	printf
	    ("Interprets any file as plain binary data and dumps to a raw C array.\n");
	printf("usage: bin2array <in-file> <out-file> \n\n");
} int main(int argc, char *argv[])
{
	int fd_in, fd_out, in_fsize;
	char out_buffer[1024] = { 0 };
	int restart = 0;
	int i, j;
	unsigned char in_buffer[1024] = { 0 };
	int chunk;
	int mode = -1;
	printf("bin2array %s %s\n", argv[1], argv[2]);
	if (argc < 3) {
		help();
		return -1;
	}
	fd_in = open(argv[1], O_RDONLY);
	if (fd_in == -1) {
		printf("open input file error\n");
		return -1;
	}
	in_fsize = lseek(fd_in, 0, SEEK_END);
	printf("----------------infilesize = %d\n", in_fsize);
	fd_out =
	    open(argv[2], O_CREAT | O_RDWR | O_TRUNC,
		 S_IRGRP | S_IROTH | S_IRUSR | S_IWUSR);
	if (fd_out == -1) {
		printf
		    ("please use the absolute file path, open ouput file error\n");
		return -1;
	}
	sprintf(out_buffer, "unsigned char %s [ %d ] %s\n", name_str,
		in_fsize, data_string);
	if (write(fd_out, out_buffer, strlen(out_buffer)) != strlen(out_buffer)) {
		printf("write outfile error\n");
		return -1;
	}
	sprintf(out_buffer, "unsigned char %s [ %d ] = {\n\t", name_str,
		in_fsize);
	if (write(fd_out, out_buffer, strlen(out_buffer)) != strlen(out_buffer)) {
		printf("write outfile error\n");
		return -1;
	}
	lseek(fd_in, 0, SEEK_SET);

#if TEST			//for debug
	int test;
	read(fd_in, &test, 1);
	printf("test  = %.2hx\n", test);
	close(fd_in);
	close(fd_out);
	return 0;

#endif /*  */
	for (i = 0; i < in_fsize;) {
		chunk = i + bufferLen < in_fsize ? bufferLen : in_fsize - i;
		read(fd_in, in_buffer, chunk);
		for (j = 0; j < chunk; ++j) {
			char outbuf[128];
			sprintf(outbuf, "0x%.2hX,", in_buffer[j]);
			if (write(fd_out, outbuf, strlen(outbuf)) !=
			    strlen(outbuf)) {
				printf("write data error\n");
				return -1;
			}
			++restart;
			if (restart > 0xf) {
				sprintf(outbuf, "\n\t");
				write(fd_out, outbuf, strlen(outbuf));
				restart = 0;
			}
		}
		i += chunk;
	}
	sprintf(out_buffer, "\n};\n");
	write(fd_out, out_buffer, strlen(out_buffer));
	chmod(argv[2], 0644);
	close(fd_in);
	close(fd_out);
	return 0;
}
