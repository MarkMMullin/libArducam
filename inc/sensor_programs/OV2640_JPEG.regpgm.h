/********************************************
START Program,OV2640_JPEG		Generated,Sun Sep  4 16:08:21 2016		Level:C++
********************************************/

const struct sensor_reg OV2640_JPEG[] =
{
	{RB1_RSVD_e0,0x14},                              /*1	[e0 := 14]		Undocumented	0x14*/
	{RB1_RSVD_e1,0x77},                              /*2	[e1 := 77]		Undocumented	0x77*/
	{RB1_RSVD_e5,0x1f},                              /*3	[e5 := 1f]		Undocumented	0x1f*/
	{RB1_RSVD_d7,0x03},                              /*4	[d7 := 03]		Undocumented	0x03*/
	{RB1_RSVD_da,0x10},                              /*5	[da := 10]		Undocumented	0x10*/
	{RB1_RSVD_e0,0x00},                              /*6	[e0 := 00]		Undocumented	0x00*/
	{RB0_RA_DLMT,0x01},                              /*7	[ff := 01]		Register Bank Select	 SENS(01)/SENS */
	{RB1_REG04,0x08},                                /*8	[04 := 08]		Register 04	 HREF_EN(08)/HREF_EN */
	{0xFF,0xFF}
};

