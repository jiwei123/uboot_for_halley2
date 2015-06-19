#include <config.h>
#include <common.h>
#include <jz_lcd/jz_dsim.h>
#include <jz_lcd/jz_lcd_v1_2.h>
#include <jz_lcd/auo_x163.h>
#include <lcd.h>

extern void* lcd_get_fb_base(void);

static struct dsi_device *dsi = &jz_dsi;

static int write_to_mipi(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int l_count = (argc - 1) % 256; /* parameter count low 8bits */
	int h_count = (argc - 1) / 256; /* parameter count high 8bits */
	int i;

	struct dsi_cmd_packet my_data;

	if(argc < 2) {
		debug("your parameter is too few\n");
		return -1;
	}
	if(argv[1][0] >= '0' && argv[1][0] <= '9') {
		my_data.packet_type = 0x39; /* sent long packet */
		my_data.cmd0_or_wc_lsb = l_count;
		my_data.cmd1_or_wc_msb = h_count;
		for(i = 0; i < argc - 1; i++)
			my_data.cmd_data[i] = simple_strtol(argv[i+1], NULL, 16);
		write_command(dsi,my_data);

		debug(" write success \n ");
		return 0;
	}
	else {
		debug(" your command is wrong \n");
		return -1;
	}
}

U_BOOT_CMD( /* now just only can write command ,can't read*/
	mipi_command,	5,	1,	write_to_mipi, /* the max num of command line param argc be set 5 */
	"write long packet to AUO_X163",
	"--eg.  write_lcd 0x2a 0x00 0x05 0x00 0x0f (0x2a is the command to control the SC & EC, 0x00 is the param)\n"
);


static int do_mipi_short_command(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct dsi_cmd_packet my_data;

	if(!(argc == 2 || argc == 3)) {
		printf("your parameter is wrong\n");
		return -1;
	}

	my_data.packet_type =  argc == 2 ? 0x05 : 0x15; /* sent long packet */
	my_data.cmd0_or_wc_lsb = simple_strtol(argv[1], NULL, 16);
	my_data.cmd1_or_wc_msb = argc == 2 ? 0x00 : simple_strtol(argv[2], NULL, 16);

	write_command(dsi,my_data);

	printf(" write success \n ");

	return 0;
}

U_BOOT_CMD( /* now just only can write command ,can't read*/
	mipi_short_command,	3,	1,	do_mipi_short_command,
	"write short packet to AUO_X163",
	"--eg.  write_lcd 0x29 0x00\n"
);

static int do_mipi_power(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct dsi_cmd_packet my_data;

	if(argc != 2) {
		printf("your parameter is wrong\n");
		return -1;
	}

	if (!strcmp(argv[1], "enable")) {

	} else if (!strcmp(argv[1], "clock")) {
		jz_dsih_dphy_clock_en(dsi, 0);
	} else if (!strcmp(argv[1], "shutdown")) {
		jz_dsih_dphy_shutdown(dsi, 0);
	} else if (!strcmp(argv[1], "gate")) {
		*(volatile unsigned int *)0xb0000020 |= (1<<26); //close gate for clk
	} else if (!strcmp(argv[1], "power")) {
        mipi_dsih_hal_power(dsi, 0);
	} else if (!strcmp(argv[1], "lp")) {
		mipi_dsih_dphy_enable_hs_clk(dsi, 0);
	}

	return 0;
}

U_BOOT_CMD( /* now just only can write command ,can't read*/
	mipi_power,	3,	1,	do_mipi_power,
	"write short packet to AUO_X163",
	"--eg.  write_lcd 0x29 0x00\n"
);

static int do_mipi_reg_read(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct dsi_cmd_packet my_data;

	if(argc != 4) {
		printf("your parameter is wrong\n");
		return -1;
	}

	unsigned int reg_address = simple_strtoul(argv[1], NULL, 16);
	unsigned int start = simple_strtoul(argv[2], NULL, 10);
	unsigned int end = simple_strtoul(argv[3], NULL, 10);

	unsigned int data = mipi_dsih_read_part(dsi, reg_address, start, end - start + 1);

	printf ("data: 0x%x\n", data);

	return 0;
}

U_BOOT_CMD( /* now just only can write command ,can't read*/
	mipi_reg_read,	10,	1,	do_mipi_reg_read,
	"write short packet to AUO_X163",
	"--eg.  write_lcd 0x29 0x00\n"
);

static int do_mipi_reg_write(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct dsi_cmd_packet my_data;

	if(argc != 5) {
		printf("your parameter is wrong\n");
		return -1;
	}

	unsigned int reg_address = simple_strtoul(argv[1], NULL, 16);
	unsigned int start = simple_strtoul(argv[2], NULL, 10);
	unsigned int end = simple_strtoul(argv[3], NULL, 10);
	unsigned int data = simple_strtoul(argv[4], NULL, 16);

	mipi_dsih_write_part(dsi, reg_address, data, start, end - start + 1);

	return 0;
}

U_BOOT_CMD( /* now just only can write command ,can't read*/
	mipi_reg_write,	10,	1,	do_mipi_reg_write,
	"write short packet to AUO_X163",
	"--eg.  write_lcd 0x29 0x00\n"
);
