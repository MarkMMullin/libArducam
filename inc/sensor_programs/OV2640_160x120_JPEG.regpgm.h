/********************************************
START Program,OV2640_160x120_JPEG		Generated,Sun Sep  4 16:08:21 2016		Level:C++
********************************************/

const struct sensor_reg OV2640_160x120_JPEG[] =
{
	{RB0_RA_DLMT,0x01},                              /*1	[ff := 01]		Register Bank Select	 SENS(01)/SENS */
	{RB1_COM7,0x40},                                 /*2	[12 := 40]		Common control 7	 SVGA/RES_SVGA */
	{RB1_HREFST,0x11},                               /*3	[17 := 11]		Horizontal Window start MSB 8 bit = named HSTART	0x11*/
	{RB1_HREFEND,0x43},                              /*4	[18 := 43]		Horizontal Window end MSB 8 bit = named HEND	0x43*/
	{RB1_VSTRT,0x00},                                /*5	[19 := 00]		Vertical Window start MSB 8 bit	0x00*/
	{RB1_VEND,0x4b},                                 /*6	[1a := 4b]		Vertical Window end MSB 8 bit	0x4b*/
	{RB1_REG32,0x09},                                /*7	[32 := 09]		Common Control 32	0x09*/
	{RB1_BD50,0xca},                                 /*8	[4f := ca]		50Hz Banding AEC 8 LSBs	0xca*/
	{RB1_BD60,0xa8},                                 /*9	[50 := a8]		60Hz Banding AEC 8 LSBs	0xa8*/
	{RB1_RSVD_5a,0x23},                              /*10	[5a := 23]		RB1_RSVD_5a	0x23*/
	{RB1_RSVD_6d,0x00},                              /*11	[6d := 00]		RB1_RSVD_6d	0x00*/
	{RB1_RSVD_39,0x12},                              /*12	[39 := 12]		RB1_RSVD_39	0x12*/
	{RB1_RSVD_35,0xda},                              /*13	[35 := da]		RB1_RSVD_35	0xda*/
	{RB1_RSVD_22,0x1a},                              /*14	[22 := 1a]		RB1_RSVD_22	0x1a*/
	{RB1_RSVD_37,0xc3},                              /*15	[37 := c3]		RB1_RSVD_37	0xc3*/
	{RB1_RSVD_23,0x00},                              /*16	[23 := 00]		RB1_RSVD_23	0x00*/
	{RB1_ARCOM2,0xc0},                               /*17	[34 := c0]		Zoom: Horizontal start point	0xc0*/
	{RB1_RSVD_36,0x1a},                              /*18	[36 := 1a]		RB1_RSVD_36	0x1a*/
	{RB1_RSVD_06,0x88},                              /*19	[06 := 88]		RB1_RSVD_06	0x88*/
	{RB1_RSVD_07,0xc0},                              /*20	[07 := c0]		RB1_RSVD_07	0xc0*/
	{RB1_COM4,0x87},                                 /*21	[0d := 87]		RB1_COM4	0x87*/
	{RB1_RSVD_0e,0x41},                              /*22	[0e := 41]		RB1_RSVD_0e	0x41*/
	{RB1_RSVD_4c,0x00},                              /*23	[4c := 00]		RB1_RSVD_4c	0x00*/
	{RB0_RA_DLMT,0x00},                              /*24	[ff := 00]		Register Bank Select	 DSP(00)/DSP */
	{RB0_RESET,0x04},                                /*25	[e0 := 04]		Reset	 DVP(04)/DVP */
	{RB0_HSIZE8,0x64},                               /*26	[c0 := 64]		Image Horizontal Size HSIZE[10:3]	 SET(64)/SET(100) */
	{RB0_VSIZE8,0x4b},                               /*27	[c1 := 4b]		Image Vertical Size VSIZE[10:3]	 SET(4b)/SET(75) */
	{RB0_CTRL2,0x35},                                /*28	[86 := 35]		DSP Module enable 2	 DCW_EN(20)/DCW_EN  |  SDE_EN(10)/SDE_EN  |  UV_AVG_EN(04)/UV_AVG_EN  |  CMX_EN(01)/CMX_EN */
	{RB0_CTRL,0x92},                                 /*29	[50 := 92]		RB0_CTRL	 LP_DP(80)/LP_DP  |  V_DIV_SET(02)/V_DIV_SET(2)  |  H_DIV_SET(02)/H_DIV_SET(2) */
	{RB0_HSIZE,0xc8},                                /*30	[51 := c8]		H_SIZE[7:0] (real/4)	0xc8*/
	{RB0_VSIZE,0x96},                                /*31	[52 := 96]		V_SIZE[7:0] (real/4)	0x96*/
	{RB0_XOFFL,0x00},                                /*32	[53 := 00]		OFFSET_X[7:0]SET(x)	0x00*/
	{RB0_YOFFL,0x00},                                /*33	[54 := 00]		OFFSET_Y[7:0]	0x00*/
	{RB0_VHYX,0x00},                                 /*34	[55 := 00]		Offset and size completion	 YOFF_SET(00)/YOFF_SET(0)  |  XOFF_SET(00)/XOFF_SET(0) */
	{RB0_TEST,0x00},                                 /*35	[57 := 00]		Horizontal size completion	0x00*/
	{RB0_ZMOW,0x28},                                 /*36	[5a := 28]		Zoom: Out Width  OUTW[7:0] (real/4)	 OUTW_SET(28)/OUTW_SET(40) */
	{RB0_ZMOH,0x1e},                                 /*37	[5b := 1e]		Zoom: Out Height OUTH[7:0] (real/4)	 OUTH_SET(1e)/OUTH_SET(30) */
	{RB0_ZMHH,0x00},                                 /*38	[5c := 00]		Zoom: Speed and H&W completion	 ZSPEED_SET(00)/ZSPEED_SET(0)  |  OUTW_SET(00)/OUTW_SET(0) */
	{RB0_RESET,0x00},                                /*39	[e0 := 00]		Reset	0x00*/
	{0xFF,0xFF}
};

