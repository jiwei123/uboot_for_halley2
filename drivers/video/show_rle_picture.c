#include <common.h>
#include <malloc.h>
#include <lcd.h>

DECLARE_GLOBAL_DATA_PTR;
extern unsigned char rle_default_logo_addr[];
extern int lcd_line_length;

static inline unsigned int color16_to_32(unsigned short color) {
	unsigned int r, g, b;

	r = color >> 11;
	r = (r << 3) | 0x7;

	g = (color >> 5) & 0x003F;
	g = (g << 2) | 0x3;

	b = color & 0x001F;
	b = (b << 3) | 0x7;

	return r << 16 | g << 8 | b;
}

static inline unsigned short color32_to_16(unsigned int color) {
	unsigned short r, g, b;

	r = (color >> 16) & 0xff;
	r = r >> 3;

	g = (color >> 8) & 0xff;
	g = g >> 2;

	b = color & 0xff;
	b = b >> 3;

	return r << 11 | g << 5 | b;
}

static inline void fb_set_bpp16(unsigned short *dst, unsigned short color, unsigned int count) {
	while (count--) {
		*dst++ = color;
	}
}

static inline void fb_set_bpp32(unsigned int *dst, unsigned int color, unsigned int count) {
	while (count--) {
		*dst++ = color;
	}
}

/**
 *  create_fb_from_rle:	通过rle数组所表示的图片和显存每像素所需要的位数创建一片显存,并返回显存的宽度和高度
 *  @rle: rle数组的起始地址
 *  @rle_width: 返回显存宽度
 *  @rle_height: 返回显存高度
 *  @bpp: 显存每像素所需要的位数
 *  @return: 返回显存的地址
 */

void *create_fb_from_rle(unsigned short *rle, unsigned int *rle_width, unsigned int *rle_height, int bpp)
{
	int is_32_bpp = (bpp == 32 || bpp == 24 || bpp == 18);
	unsigned short width, height;
	unsigned short count;
	unsigned short value;
	unsigned int i, j;
	unsigned short *dst, *mem;

	width = rle[0];
	height = rle[1];
	rle += 4;

	mem = malloc(height * ((width * 2) << is_32_bpp));
	if (!mem) {
		printf ("%s: failed to allocate memory for rle\n", __func__);
		return NULL;
	}

	dst = mem;

	for (i = 0; i < height; ++i) {
		for (j = 0; j < width;) {
			count = *rle++;
			value = *rle++;
			if (is_32_bpp)
				fb_set_bpp32((unsigned int *)dst, color16_to_32(value), count);
			else
				fb_set_bpp16(dst, value, count);
			dst += (count << is_32_bpp);
			j += count;
		}
	}

	*rle_width = width;
	*rle_height = height;

	return mem;
}

/**
 *  write_mem: 把base1地址内存中宽width和高height的内容拷贝到base0地址内存中
 *  @base0: 源内存起始地址
 *  @base1: 目标内存起始地址
 *  @width: 源内存内容的宽度
 *  @height: 源内存内容的高度
 *  @pixels_line0: 源内存行大小
 *  @pixels_line1: 目标内存行大小
 *  @size: 位大小
 */

void write_mem(void *base0, void *base1, int width, int height,
               int pixels_line0, int pixels_line1, int size) {
	int i;
	width = max(width, 0);
	height = max(height, 0);

	for (i = 0; i < height; ++i) {
		memcpy(base0, base1, width * size);
		base0 += pixels_line0 * size;
		base1 += pixels_line1 * size;
	}
}

/**
 * clear_mem: 设置fb的全部像素为同一个颜色值
 * @base: fb 内存起始地址
 * @width: fb的宽度
 * @height: fb的高度
 * @pixels_line: fb每行的像素个数，pixels_line >= width
 * @bpp: 每像素所需的位数
 * @color32: 需要清零的颜色，以32位颜色表示，如果目标fb是16位，那么会自动转换为16的颜色
 */
void clear_mem(void *base, int width, int height, int pixels_line, int bpp, unsigned int color32) {
	int i;
	unsigned short color16;
	unsigned short *p16;
	unsigned int *p32;

	if (bpp == 32 || bpp == 24 || bpp == 18) {
		p32 = base;
		for (i = 0; i < height; ++i) {
			fb_set_bpp32(p32, color32, width);
			p32 += pixels_line;
		}
	} else {
		p16 = base;
		color16 = color32_to_16(color32);
		for (i = 0; i < height; ++i) {
			fb_set_bpp16(p16, color16, width);
			p16 += pixels_line;
		}
	}
}

/**
 * clear_fb: 把当前fb的全部像素设置为同一个颜色值
 * @color32: 需要清零的颜色，以32位颜色表示，如果目标fb是16位，那么会自动转换为16的颜色
 */
void clear_fb(unsigned int color32)
{
	int bpp = lcd_get_pixel_bpp();
	void *fb_base = lcd_get_fb_base();
	int fb_width = lcd_get_pixel_width();
	int fb_height = lcd_get_pixel_height();
	int fb_pixels_line_length = lcd_get_pixels_line_length();

	clear_mem(fb_base, fb_width, fb_height, fb_pixels_line_length, bpp, color32);
}

/**
 *  write_fb: 把每像素所需的位数相同的源显存src拷贝到目标显存dst的(x,y)位置,注意每像素所需的位数是相同的
 *  @dst: 目标显存起始地址
 *  @dst_xres: 目标显存宽度
 *  @dst_yres: 目标显存高度
 *  @dst_pixels_line_length: 目标显存行大小
 *  @src: 源显存起始地址
 *  @src_xres: 源显存宽度
 *  @src_yres: 源显存高度
 *  @src_pixels_line_length: 源显存行大小
 *  @x: x坐标
 *  @y: y坐标
 *  @bpp: 每像素所需的位数
 */

void write_fb(void *dst, int dst_xres, int dst_yres, int dst_pixels_line_length,
              void *src, int src_xres, int src_yres, int src_pixels_line_length,
              int x, int y, int bpp) {
	int x0, y0, x1, y1, x_off, y_off;
	int x_len, y_len;
	int size = 0;
	int is_32_bpp = (bpp == 32 || bpp == 24 || bpp == 18);
	if(is_32_bpp)
		size = 4;
	else
		size = 2;

	x0 = max(0, x);
	y0 = max(0, y);

	x_off = x0 - x;
	y_off = y0 - y;

	if (x >= dst_xres || y >= dst_yres)
		return;

	x1 = x + src_xres;
	y1 = y + src_yres;

	if (x1 <= 0 || y1 <= 0)
		return;

	x_len = x > 0 ? src_xres : x1;
	y_len = y > 0 ? src_yres : y1;

	if (x1 > dst_xres)
		x_len -= (x1 - dst_xres);

	if (y1 > dst_yres)
		y_len -= (y1 - dst_yres);

	write_mem(dst + (y0 * dst_pixels_line_length + x0) * size,
              src + (y_off * src_pixels_line_length + x_off) * size,
              x_len, y_len,
              dst_pixels_line_length, src_pixels_line_length, size);
}

/**
 *  show_rle_picture: 将rle存储格式的图片以宽为dst_xres高为dst_yres显示在目标显存dst的(x,y)位置
 *  @src_picture_addr: 源图片rle数组起始地址
 *  @dst: 目标显存起始地址
 *  @dst_xres: 目标显存的宽度
 *  @dst_yres: 目标显存的高度
 *  @dst_pixels_line_length: 目标显存行大小
 *  @x: x坐标
 *  @y: y坐标
 *  @bpp: 目标显存每像素所需的位数
 */

int show_rle_picture(unsigned short *src_picture_addr, unsigned int *dst,
                     int dst_xres, int dst_yres, int dst_pixels_line_length,
                     int x, int y, int bpp) {
	unsigned int width, height;
	unsigned int *mem = NULL;
	mem = create_fb_from_rle(src_picture_addr, &width, &height, bpp);
	if (!mem)
		return -1;
	write_fb(dst, dst_xres, dst_yres, dst_pixels_line_length,
             mem, width, height, width,
             x, y, bpp);
	free(mem);
	lcd_sync();
	return 0;
}

/**
 *  show_rle_picture_in_middle: 将rle存储格式的图片以宽为dst_xres高为dst_yres居中显示在目标显存dst上
 *  @src_picture_addr: 源图片rle数组起始地址
 *  @dst: 目标显存起始地址
 *  @dst_xres: 目标显存的宽度
 *  @dst_yres: 目标显存的高度
 *  @dst_pixels_line_length: 目标显存行大小
 *  @bpp: 目标显存每像素所需的位数
 */

int show_rle_picture_in_middle(unsigned short *src_picture_addr, unsigned int *dst,
                               int dst_xres, int dst_yres,
                               int dst_pixels_line_length, int bpp) {
	int x, y;
	unsigned int width, height;
	unsigned int *mem = NULL;
	mem = create_fb_from_rle(src_picture_addr, &width, &height, bpp);
	if (!mem)
		return -1;
	x = (int)(panel_info.vl_col - width) / 2;
	y = (int)(panel_info.vl_row - height) / 2;
	write_fb(dst, dst_xres, dst_yres, dst_pixels_line_length,
             mem, width, height, width,
             x, y, bpp);
	free(mem);
	lcd_sync();
	return 0;
}

/**
 *  show_rle_picture_in_fb_middle: 居中显示rle存储格式的图片在屏幕上
 *  @src_picture_addr: 源图片rle数组起始地址
 */

int show_rle_picture_in_fb_middle(unsigned short *src_picture_addr)
{
	int ret;
	int bpp = lcd_get_pixel_bpp();
	void *fb_base = lcd_get_fb_base();
	int fb_width = lcd_get_pixel_width();
	int fb_height = lcd_get_pixel_height();
	int fb_pixels_line_length = lcd_get_pixels_line_length();

	ret = show_rle_picture_in_middle(src_picture_addr, fb_base,
                                     fb_width, fb_height,
                                     fb_pixels_line_length, bpp);
	return ret;
}

/*test to show the lcd_logo*/
static int
do_lcd_logo_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int width, height;
	unsigned int *mem = NULL;
	int x = 0, y = 0;

	if (!((argc == 1) || (argc == 3)))
		return CMD_RET_USAGE;

	if (argc == 3 ) {
		x = simple_strtol(argv[1], NULL, 10);
		y = simple_strtol(argv[2], NULL, 10);
	}

	printf("mem:%p\n", rle_default_logo_addr);
	mem = create_fb_from_rle((unsigned int short *)rle_default_logo_addr, &width, &height, 32);
	printf("mem:%p width:%d height:%d %d %d \n", mem, width, height, x, y);

	if (!mem)
		return CMD_RET_SUCCESS;

	write_fb(gd->fb_base, panel_info.vl_col, panel_info.vl_row, panel_info.vl_col,
             mem, width, height, width,
             x, y, 32);

	free(mem);

	lcd_sync();

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(lcd_logo_test, 3, 0, do_lcd_logo_test,
	"test lcd_logo",
	"lcd_logo_test [x, y]");
