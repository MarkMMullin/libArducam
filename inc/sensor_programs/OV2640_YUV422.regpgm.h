/********************************************
START Program,OV2640_YUV422		Generated,Sun Sep  4 16:08:21 2016		Level:C++
********************************************/

const struct sensor_reg OV2640_YUV422[] =
{
	{RB0_RA_DLMT,0x00},                              /*1	[ff := 00]		Register Bank Select	 DSP(00)/DSP */
	{RB0_R_BYPASS,0x00},                             /*2	[05 := 00]		Bypass DSP	 Use the internal DSP/USE_DSP */
	{RB0_IMAGE_MODE,0x10},                           /*3	[da := 10]		Image Output Format Select	 JPEG_EN(10)/JPEG_EN  |  YUV422(00)/YUV422 */
	{RB0_RSVD_d7,0x03},                              /*4	[d7 := 03]		RB0_RSVD_d7	0x03*/
	{RB0_RSVD_df,0x00},                              /*5	[df := 00]		RB0_RSVD_df	0x00*/
	{RB0_RSVD_33,0x80},                              /*6	[33 := 80]		RB0_RSVD_33	0x80*/
	{RB0_RSVD_3c,0x40},                              /*7	[3c := 40]		RB0_RSVD_3c	0x40*/
	{RB0_RSVD_e1,0x77},                              /*8	[e1 := 77]		RB0_RSVD_e1	0x77*/
	{RB0_RSVD_00,0x00},                              /*9	[00 := 00]		RB0_RSVD_00	0x00*/
	{0xFF,0xFF}
};

