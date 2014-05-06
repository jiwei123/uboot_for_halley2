#ifndef _RLE_LOGO_H
#define _RLE_LOGO_H

//#include <jz_logo_file.h>
//#define RLE_LOGO_BASE_ADDR (0x00000000)    //need to fixed!
//#define RLE_LOGO_DEFAULT_ADDR_OFFSET  (0x00000000)   //need to fixed!	
#if !defined(CONFIG_LCD_INFO_BELOW_LOGO)
#define  BMP_LOGO_HEIGHT  panel_info.vl_row 
#define  BMP_LOGO_WIDTH   panel_info.vl_col
#else
//#define  BMP_LOGO_HEIGHT  panel_info.vl_row 
//#define  BMP_LOGO_WIDTH   panel_info.vl_col
#define  BMP_LOGO_HEIGHT  0
#define  BMP_LOGO_WIDTH   0
#endif

#define BMP_LOGO_COLORS		0

unsigned short bmp_logo_palette[] = {};
unsigned int bmp_logo_bitmap [] = {};
#endif
