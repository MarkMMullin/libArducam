/********************************************
START Program,OV2640_1024x768_JPEG		Generated,Sun Sep  4 16:08:21 2016		Level:C++
********************************************/

const struct sensor_reg OV2640_1024x768_JPEG[] =
{
	{RB0_RA_DLMT,0x01},                              /*1	[ff := 01]		Register Bank Select	 SENS(01)/SENS */
	{RB1_CLKRC,0x01},                                /*2	[11 := 01]		Internal clock	0x01*/
	{RB1_COM7,0x00},                                 /*3	[12 := 00]		Common control 7	 Resolution selectors for UXGA/RES_UXGA */
	{RB1_HREFST,0x11},                               /*4	[17 := 11]		Horizontal Window start MSB 8 bit = named HSTART	0x11*/
	{RB1_HREFEND,0x75},                              /*5	[18 := 75]		Horizontal Window end MSB 8 bit = named HEND	0x75*/
	{RB1_REG32,0x36},                                /*6	[32 := 36]		Common Control 32	0x36*/
	{RB1_VSTRT,0x01},                                /*7	[19 := 01]		Vertical Window start MSB 8 bit	0x01*/
	{RB1_VEND,0x97},                                 /*8	[1a := 97]		Vertical Window end MSB 8 bit	0x97*/
	{RB1_COM1,0x0f},                                 /*9	[03 := 0f]		Common control 1	 VWIN_LSB_UXGA(0f)/VWIN_LSB_UXGA */
	{RB1_RSVD_37,0x40},                              /*10	[37 := 40]		RB1_RSVD_37	0x40*/
	{RB1_BD50,0xbb},                                 /*11	[4f := bb]		50Hz Banding AEC 8 LSBs	0xbb*/
	{RB1_BD60,0x9c},                                 /*12	[50 := 9c]		60Hz Banding AEC 8 LSBs	0x9c*/
	{RB1_RSVD_5a,0x57},                              /*13	[5a := 57]		RB1_RSVD_5a	0x57*/
	{RB1_RSVD_6d,0x80},                              /*14	[6d := 80]		RB1_RSVD_6d	0x80*/
	{RB1_RSVD_3d,0x34},                              /*15	[3d := 34]		RB1_RSVD_3d	0x34*/
	{RB1_RSVD_39,0x02},                              /*16	[39 := 02]		RB1_RSVD_39	0x02*/
	{RB1_RSVD_35,0x88},                              /*17	[35 := 88]		RB1_RSVD_35	0x88*/
	{RB1_RSVD_22,0x0a},                              /*18	[22 := 0a]		RB1_RSVD_22	0x0a*/
	{RB1_RSVD_37,0x40},                              /*19	[37 := 40]		RB1_RSVD_37	0x40*/
	{RB1_ARCOM2,0xa0},                               /*20	[34 := a0]		Zoom: Horizontal start point	0xa0*/
	{RB1_RSVD_06,0x02},                              /*21	[06 := 02]		RB1_RSVD_06	0x02*/
	{RB1_COM4,0xb7},                                 /*22	[0d := b7]		RB1_COM4	0xb7*/
	{RB1_RSVD_0e,0x01},                              /*23	[0e := 01]		RB1_RSVD_0e	0x01*/
	{RB0_RA_DLMT,0x00},                              /*24	[ff := 00]		Register Bank Select	 DSP(00)/DSP */
	{RB0_HSIZE8,0xc8},                               /*25	[c0 := c8]		Image Horizontal Size HSIZE[10:3]	 SET(c8)/SET(200) */
	{RB0_VSIZE8,0x96},                               /*26	[c1 := 96]		Image Vertical Size VSIZE[10:3]	 SET(96)/SET(150) */
	{RB0_SIZEL,0x00},                                /*27	[8c := 00]		Image Size Completion	 HSIZE8_SET(00)/HSIZE8_SET(0)  |  VSIZE8_SET(00)/VSIZE8_SET(0) */
	{RB0_CTRL2,0x3d},                                /*28	[86 := 3d]		DSP Module enable 2	 DCW_EN(20)/DCW_EN  |  SDE_EN(10)/SDE_EN  |  UV_ADJ_EN(08)/UV_ADJ_EN  |  UV_AVG_EN(04)/UV_AVG_EN  |  CMX_EN(01)/CMX_EN */
	{RB0_CTRL,0x00},                                 /*29	[50 := 00]		RB0_CTRL	 V_DIV_SET(00)/V_DIV_SET(0)  |  H_DIV_SET(00)/H_DIV_SET(0) */
	{RB0_HSIZE,0x90},                                /*30	[51 := 90]		H_SIZE[7:0] (real/4)	0x90*/
	{RB0_VSIZE,0x2c},                                /*31	[52 := 2c]		V_SIZE[7:0] (real/4)	0x2c*/
	{RB0_XOFFL,0x00},                                /*32	[53 := 00]		OFFSET_X[7:0]SET(x)	0x00*/
	{RB0_YOFFL,0x00},                                /*33	[54 := 00]		OFFSET_Y[7:0]	0x00*/
	{RB0_VHYX,0x88},                                 /*34	[55 := 88]		Offset and size completion	 VSIZE_SET(80)/VSIZE_SET  |  YOFF_SET(00)/YOFF_SET(0)  |  XOFF_SET(00)/XOFF_SET(0)  ??08?? */
	{RB0_ZMOW,0x00},                                 /*35	[5a := 00]		Zoom: Out Width  OUTW[7:0] (real/4)	 OUTW_SET(00)/OUTW_SET(0) */
	{RB0_ZMOH,0xc0},                                 /*36	[5b := c0]		Zoom: Out Height OUTH[7:0] (real/4)	 OUTH_SET(c0)/OUTH_SET(192) */
	{RB0_ZMHH,0x01},                                 /*37	[5c := 01]		Zoom: Speed and H&W completion	 ZSPEED_SET(00)/ZSPEED_SET(0)  |  OUTW_SET(01)/OUTW_SET(1) */
	{RB0_R_DVP_SP,0x02},                             /*38	[d3 := 02]		DVP output speed control	0x02*/
	{0xFF,0xFF}
};

