/********************************************
START Program,OV2640_JPEG_INIT		Generated,Sun Sep  4 16:08:21 2016		Level:C++
********************************************/

const struct sensor_reg OV2640_JPEG_INIT[] =
{
	{RB0_RA_DLMT,0x00},                              /*1	[ff := 00]		Register Bank Select	 DSP(00)/DSP */
	{RB0_RSVD_2c,0xff},                              /*2	[2c := ff]		RB0_RSVD_2c	0xff*/
	{RB0_RSVD_2e,0xdf},                              /*3	[2e := df]		RB0_RSVD_2e	0xdf*/
	{RB0_RA_DLMT,0x01},                              /*4	[ff := 01]		Register Bank Select	 SENS(01)/SENS */
	{RB1_RSVD_3c,0x32},                              /*5	[3c := 32]		RB1_RSVD_3c	0x32*/
	{RB1_CLKRC,0x04},                                /*6	[11 := 04]		Internal clock	MMM (BBB 0x04, Ard = 00)*/
	{RB1_COM2,0x02},                                 /*7	[09 := 02]		Common control 2	0x02*/
	{RB1_REG04,0x28},                                /*8	[04 := 28]		Register 04	 Always set/DEF  |  HREF_EN(08)/HREF_EN */
	{RB1_COM8,0xe5},                                 /*9	[13 := e5]		Common control 8	 Banding filter1/DEF  |  Banding filter2/BNDF_EN  |  AGC Auto/Manual/AGC_EN  |  Auto/Manual Exposure/AEC_EN */
	{RB1_COM9,0x48},                                 /*10	[14 := 48]		Common control 9 Automatic gain ceiling - maximum AGC value [7:5]	 AGC_GAIN_8x(40)/AGC_GAIN_8x  ??08?? */
	{RB1_RSVD_2c,0x0c},                              /*11	[2c := 0c]		RB1_RSVD_2c	0x0c*/
	{RB1_RSVD_33,0x78},                              /*12	[33 := 78]		RB1_RSVD_33	0x78*/
	{RB1_RSVD_3a,0x33},                              /*13	[3a := 33]		RB1_RSVD_3a	0x33*/
	{RB1_RSVD_3b,0xfb},                              /*14	[3b := fb]		RB1_RSVD_3b	0xfb*/
	{RB1_RSVD_3e,0x00},                              /*15	[3e := 00]		RB1_RSVD_3e	0x00*/
	{RB1_RSVD_43,0x11},                              /*16	[43 := 11]		RB1_RSVD_43	0x11*/
	{RB1_RSVD_16,0x10},                              /*17	[16 := 10]		RB1_RSVD_16	0x10*/
	{RB1_RSVD_39,0x92},                              /*18	[39 := 92]		RB1_RSVD_39	0x92*/
	{RB1_RSVD_35,0xda},                              /*19	[35 := da]		RB1_RSVD_35	0xda*/
	{RB1_RSVD_22,0x1a},                              /*20	[22 := 1a]		RB1_RSVD_22	0x1a*/
	{RB1_RSVD_37,0xc3},                              /*21	[37 := c3]		RB1_RSVD_37	0xc3*/
	{RB1_RSVD_23,0x00},                              /*22	[23 := 00]		RB1_RSVD_23	0x00*/
	{RB1_ARCOM2,0xc0},                               /*23	[34 := c0]		Zoom: Horizontal start point	0xc0*/
	{RB1_RSVD_36,0x1a},                              /*24	[36 := 1a]		RB1_RSVD_36	0x1a*/
	{RB1_RSVD_06,0x88},                              /*25	[06 := 88]		RB1_RSVD_06	0x88*/
	{RB1_RSVD_07,0xc0},                              /*26	[07 := c0]		RB1_RSVD_07	0xc0*/
	{RB1_COM4,0x87},                                 /*27	[0d := 87]		RB1_COM4	0x87*/
	{RB1_RSVD_0e,0x41},                              /*28	[0e := 41]		RB1_RSVD_0e	0x41*/
	{RB1_RSVD_4c,0x00},                              /*29	[4c := 00]		RB1_RSVD_4c	0x00*/
	{RB1_COM19,0x00},                                /*30	[48 := 00]		RB1_COM19	0x00*/
	{RB1_RSVD_5b,0x00},                              /*31	[5b := 00]		RB1_RSVD_5b	0x00*/
	{RB1_RSVD_42,0x03},                              /*32	[42 := 03]		RB1_RSVD_42	0x03*/
	{RB1_RSVD_4a,0x81},                              /*33	[4a := 81]		RB1_RSVD_4a	0x81*/
	{RB1_RSVD_21,0x99},                              /*34	[21 := 99]		RB1_RSVD_21	0x99*/
	{RB1_AEW,0x40},                                  /*35	[24 := 40]		AGC/AEC - Stable operating region (upper limit)	0x40*/
	{RB1_AEB,0x38},                                  /*36	[25 := 38]		AGC/AEC - Stable operating region (lower limit)	0x38*/
	{RB1_VV,0x82},                                   /*37	[26 := 82]		AGC/AEC Fast mode operating region	 HIGH_TH_SET(08)/HIGH_TH_SET(8)  |  LOW_TH_SET(02)/LOW_TH_SET(2) */
	{RB1_RSVD_5c,0x00},                              /*38	[5c := 00]		RB1_RSVD_5c	0x00*/
	{RB1_RSVD_63,0x00},                              /*39	[63 := 00]		RB1_RSVD_63	0x00*/
	{RB1_HISTO_LOW,0x70},                            /*40	[61 := 70]		Histogram Algorithm Low Level	0x70*/
	{RB1_HISTO_HIGH,0x80},                           /*41	[62 := 80]		Histogram Algorithm High Level	0x80*/
	{RB1_RSVD_7c,0x05},                              /*42	[7c := 05]		RB1_RSVD_7c	0x05*/
	{RB1_RSVD_20,0x80},                              /*43	[20 := 80]		RB1_RSVD_20	0x80*/
	{RB1_RSVD_28,0x30},                              /*44	[28 := 30]		RB1_RSVD_28	0x30*/
	{RB1_RSVD_6c,0x00},                              /*45	[6c := 00]		RB1_RSVD_6c	0x00*/
	{RB1_RSVD_6d,0x80},                              /*46	[6d := 80]		RB1_RSVD_6d	0x80*/
	{RB1_RSVD_6e,0x00},                              /*47	[6e := 00]		RB1_RSVD_6e	0x00*/
	{RB1_RSVD_70,0x02},                              /*48	[70 := 02]		RB1_RSVD_70	0x02*/
	{RB1_RSVD_71,0x94},                              /*49	[71 := 94]		RB1_RSVD_71	0x94*/
	{RB1_RSVD_73,0xc1},                              /*50	[73 := c1]		RB1_RSVD_73	0xc1*/
	{RB1_COM7,0x40},                                 /*51	[12 := 40]		Common control 7	 SVGA/RES_SVGA */
	{RB1_HREFST,0x11},                               /*52	[17 := 11]		Horizontal Window start MSB 8 bit = named HSTART	0x11*/
	{RB1_HREFEND,0x43},                              /*53	[18 := 43]		Horizontal Window end MSB 8 bit = named HEND	0x43*/
	{RB1_VSTRT,0x00},                                /*54	[19 := 00]		Vertical Window start MSB 8 bit	0x00*/
	{RB1_VEND,0x4b},                                 /*55	[1a := 4b]		Vertical Window end MSB 8 bit	0x4b*/
	{RB1_REG32,0x09},                                /*56	[32 := 09]		Common Control 32	0x09*/
	{RB1_RSVD_37,0xc0},                              /*57	[37 := c0]		RB1_RSVD_37	0xc0*/
	{RB1_BD50,0x60},                                 /*58	[4f := 60]		50Hz Banding AEC 8 LSBs	0x60*/
	{RB1_BD60,0xa8},                                 /*59	[50 := a8]		60Hz Banding AEC 8 LSBs	0xa8*/
	{RB1_RSVD_6d,0x00},                              /*60	[6d := 00]		RB1_RSVD_6d	0x00*/
	{RB1_RSVD_3d,0x38},                              /*61	[3d := 38]		RB1_RSVD_3d	0x38*/
	{RB1_FLL,0x3f},                                  /*62	[46 := 3f]		Frame Length Adjustment LSBs	0x3f*/
	{RB1_BD50,0x60},                                 /*63	[4f := 60]		50Hz Banding AEC 8 LSBs	0x60*/
	{RB1_COM3,0x3c},                                 /*64	[0c := 3c]		Common control 3	 0 For Banding at 60H/BAND_50H  ??38?? */
	{RB0_RA_DLMT,0x00},                              /*65	[ff := 00]		Register Bank Select	 DSP(00)/DSP */
	{RB0_RSVD_e5,0x7f},                              /*66	[e5 := 7f]		RB0_RSVD_e5	0x7f*/
	{RB0_MC_BIST,0xc0},                              /*67	[f9 := c0]		Microcontroller misc register	 RESET(80)/RESET  |  BOOT_ROM_SEL(40)/BOOT_ROM_SEL */
	{RB0_RSVD_41,0x24},                              /*68	[41 := 24]		RB0_RSVD_41	0x24*/
	{RB0_RESET,0x14},                                /*69	[e0 := 14]		Reset	 JPEG(10)/JPEG  |  DVP(04)/DVP */
	{RB0_RSVD_76,0xff},                              /*70	[76 := ff]		RB0_RSVD_76	0xff*/
	{RB0_RSVD_33,0xa0},                              /*71	[33 := a0]		RB0_RSVD_33	0xa0*/
	{RB0_RSVD_42,0x20},                              /*72	[42 := 20]		RB0_RSVD_42	0x20*/
	{RB0_RSVD_43,0x18},                              /*73	[43 := 18]		RB0_RSVD_43	0x18*/
	{RB0_RSVD_4c,0x00},                              /*74	[4c := 00]		RB0_RSVD_4c	0x00*/
	{RB0_CTRL3,0xd5},                                /*75	[87 := d5]		DSP Module enable 3	 BPC_EN(80)/BPC_EN  |  WPC_EN(40)/WPC_EN  ??15?? */
	{RB0_RSVD_88,0x3f},                              /*76	[88 := 3f]		RB0_RSVD_88	0x3f*/
	{RB0_RSVD_d7,0x03},                              /*77	[d7 := 03]		RB0_RSVD_d7	0x03*/
	{RB0_RSVD_d9,0x10},                              /*78	[d9 := 10]		RB0_RSVD_d9	0x10*/
	{RB0_R_DVP_SP,0x82},                             /*79	[d3 := 82]		DVP output speed control	 SP_AUTO_MODE(80)/SP_AUTO_MODE */
	{RB0_RSVD_c8,0x08},                              /*80	[c8 := 08]		RB0_RSVD_c8	0x08*/
	{RB0_RSVD_c9,0x80},                              /*81	[c9 := 80]		RB0_RSVD_c9	0x80*/
	{RB0_BPADDR,0x00},                               /*82	[7c := 00]		SDE Indirect Register Access: Address	0x00*/
	{RB0_BPDATA,0x00},                               /*83	[7d := 00]		SDE Indirect Register Access: Data	0x00*/
	{RB0_BPADDR,0x03},                               /*84	[7c := 03]		SDE Indirect Register Access: Address	0x03*/
	{RB0_BPDATA,0x48},                               /*85	[7d := 48]		SDE Indirect Register Access: Data	0x48*/
	{RB0_BPDATA,0x48},                               /*86	[7d := 48]		SDE Indirect Register Access: Data	0x48*/
	{RB0_BPADDR,0x08},                               /*87	[7c := 08]		SDE Indirect Register Access: Address	0x08*/
	{RB0_BPDATA,0x20},                               /*88	[7d := 20]		SDE Indirect Register Access: Data	0x20*/
	{RB0_BPDATA,0x10},                               /*89	[7d := 10]		SDE Indirect Register Access: Data	0x10*/
	{RB0_BPDATA,0x0e},                               /*90	[7d := 0e]		SDE Indirect Register Access: Data	0x0e*/
	{RB0_RSVD_90,0x00},                              /*91	[90 := 00]		RB0_RSVD_90	0x00*/
	{RB0_RSVD_91,0x0e},                              /*92	[91 := 0e]		RB0_RSVD_91	0x0e*/
	{RB0_RSVD_91,0x1a},                              /*93	[91 := 1a]		RB0_RSVD_91	0x1a*/
	{RB0_RSVD_91,0x31},                              /*94	[91 := 31]		RB0_RSVD_91	0x31*/
	{RB0_RSVD_91,0x5a},                              /*95	[91 := 5a]		RB0_RSVD_91	0x5a*/
	{RB0_RSVD_91,0x69},                              /*96	[91 := 69]		RB0_RSVD_91	0x69*/
	{RB0_RSVD_91,0x75},                              /*97	[91 := 75]		RB0_RSVD_91	0x75*/
	{RB0_RSVD_91,0x7e},                              /*98	[91 := 7e]		RB0_RSVD_91	0x7e*/
	{RB0_RSVD_91,0x88},                              /*99	[91 := 88]		RB0_RSVD_91	0x88*/
	{RB0_RSVD_91,0x8f},                              /*100	[91 := 8f]		RB0_RSVD_91	0x8f*/
	{RB0_RSVD_91,0x96},                              /*101	[91 := 96]		RB0_RSVD_91	0x96*/
	{RB0_RSVD_91,0xa3},                              /*102	[91 := a3]		RB0_RSVD_91	0xa3*/
	{RB0_RSVD_91,0xaf},                              /*103	[91 := af]		RB0_RSVD_91	0xaf*/
	{RB0_RSVD_91,0xc4},                              /*104	[91 := c4]		RB0_RSVD_91	0xc4*/
	{RB0_RSVD_91,0xd7},                              /*105	[91 := d7]		RB0_RSVD_91	0xd7*/
	{RB0_RSVD_91,0xe8},                              /*106	[91 := e8]		RB0_RSVD_91	0xe8*/
	{RB0_RSVD_91,0x20},                              /*107	[91 := 20]		RB0_RSVD_91	0x20*/
	{RB0_RSVD_92,0x00},                              /*108	[92 := 00]		RB0_RSVD_92	0x00*/
	{RB0_RSVD_93,0x06},                              /*109	[93 := 06]		RB0_RSVD_93	0x06*/
	{RB0_RSVD_93,0xe3},                              /*110	[93 := e3]		RB0_RSVD_93	0xe3*/
	{RB0_RSVD_93,0x05},                              /*111	[93 := 05]		RB0_RSVD_93	0x05*/
	{RB0_RSVD_93,0x05},                              /*112	[93 := 05]		RB0_RSVD_93	0x05*/
	{RB0_RSVD_93,0x00},                              /*113	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x04},                              /*114	[93 := 04]		RB0_RSVD_93	0x04*/
	{RB0_RSVD_93,0x00},                              /*115	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x00},                              /*116	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x00},                              /*117	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x00},                              /*118	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x00},                              /*119	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x00},                              /*120	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_93,0x00},                              /*121	[93 := 00]		RB0_RSVD_93	0x00*/
	{RB0_RSVD_96,0x00},                              /*122	[96 := 00]		RB0_RSVD_96	0x00*/
	{RB0_RSVD_97,0x08},                              /*123	[97 := 08]		RB0_RSVD_97	0x08*/
	{RB0_RSVD_97,0x19},                              /*124	[97 := 19]		RB0_RSVD_97	0x19*/
	{RB0_RSVD_97,0x02},                              /*125	[97 := 02]		RB0_RSVD_97	0x02*/
	{RB0_RSVD_97,0x0c},                              /*126	[97 := 0c]		RB0_RSVD_97	0x0c*/
	{RB0_RSVD_97,0x24},                              /*127	[97 := 24]		RB0_RSVD_97	0x24*/
	{RB0_RSVD_97,0x30},                              /*128	[97 := 30]		RB0_RSVD_97	0x30*/
	{RB0_RSVD_97,0x28},                              /*129	[97 := 28]		RB0_RSVD_97	0x28*/
	{RB0_RSVD_97,0x26},                              /*130	[97 := 26]		RB0_RSVD_97	0x26*/
	{RB0_RSVD_97,0x02},                              /*131	[97 := 02]		RB0_RSVD_97	0x02*/
	{RB0_RSVD_97,0x98},                              /*132	[97 := 98]		RB0_RSVD_97	0x98*/
	{RB0_RSVD_97,0x80},                              /*133	[97 := 80]		RB0_RSVD_97	0x80*/
	{RB0_RSVD_97,0x00},                              /*134	[97 := 00]		RB0_RSVD_97	0x00*/
	{RB0_RSVD_97,0x00},                              /*135	[97 := 00]		RB0_RSVD_97	0x00*/
	{RB0_CTRL1,0xed},                                /*136	[c3 := ed]		DSP Module enable 1	 CIP(80)/CIP  |  DMY(40)/DMY  |  RAW_GMA(20)/RAW_GMA  |  AWB(08)/AWB  |  AWB_GAIN(04)/AWB_GAIN  |  PRE(01)/PRE */
	{RB0_RSVD_a4,0x00},                              /*137	[a4 := 00]		RB0_RSVD_a4	0x00*/
	{RB0_RSVD_a8,0x00},                              /*138	[a8 := 00]		RB0_RSVD_a8	0x00*/
	{RB0_RSVD_c5,0x11},                              /*139	[c5 := 11]		RB0_RSVD_c5	0x11*/
	{RB0_RSVD_c6,0x51},                              /*140	[c6 := 51]		RB0_RSVD_c6	0x51*/
	{RB0_RSVD_bf,0x80},                              /*141	[bf := 80]		RB0_RSVD_bf	0x80*/
	{RB0_RSVD_c7,0x10},                              /*142	[c7 := 10]		RB0_RSVD_c7	0x10*/
	{RB0_RSVD_b6,0x66},                              /*143	[b6 := 66]		RB0_RSVD_b6	0x66*/
	{RB0_RSVD_b8,0xa5},                              /*144	[b8 := a5]		RB0_RSVD_b8	0xa5*/
	{RB0_RSVD_b7,0x64},                              /*145	[b7 := 64]		RB0_RSVD_b7	0x64*/
	{RB0_RSVD_b9,0x7c},                              /*146	[b9 := 7c]		RB0_RSVD_b9	0x7c*/
	{RB0_RSVD_b3,0xaf},                              /*147	[b3 := af]		RB0_RSVD_b3	0xaf*/
	{RB0_RSVD_b4,0x97},                              /*148	[b4 := 97]		RB0_RSVD_b4	0x97*/
	{RB0_RSVD_b5,0xff},                              /*149	[b5 := ff]		RB0_RSVD_b5	0xff*/
	{RB0_RSVD_b0,0xc5},                              /*150	[b0 := c5]		RB0_RSVD_b0	0xc5*/
	{RB0_RSVD_b1,0x94},                              /*151	[b1 := 94]		RB0_RSVD_b1	0x94*/
	{RB0_RSVD_b2,0x0f},                              /*152	[b2 := 0f]		RB0_RSVD_b2	0x0f*/
	{RB0_RSVD_c4,0x5c},                              /*153	[c4 := 5c]		RB0_RSVD_c4	0x5c*/
	{RB0_HSIZE8,0x64},                               /*154	[c0 := 64]		Image Horizontal Size HSIZE[10:3]	 SET(64)/SET(100) */
	{RB0_VSIZE8,0x4b},                               /*155	[c1 := 4b]		Image Vertical Size VSIZE[10:3]	 SET(4b)/SET(75) */
	{RB0_SIZEL,0x00},                                /*156	[8c := 00]		Image Size Completion	 HSIZE8_SET(00)/HSIZE8_SET(0)  |  VSIZE8_SET(00)/VSIZE8_SET(0) */
	{RB0_CTRL2,0x3d},                                /*157	[86 := 3d]		DSP Module enable 2	 DCW_EN(20)/DCW_EN  |  SDE_EN(10)/SDE_EN  |  UV_ADJ_EN(08)/UV_ADJ_EN  |  UV_AVG_EN(04)/UV_AVG_EN  |  CMX_EN(01)/CMX_EN */
	{RB0_CTRL,0x00},                                 /*158	[50 := 00]		RB0_CTRL	 V_DIV_SET(00)/V_DIV_SET(0)  |  H_DIV_SET(00)/H_DIV_SET(0) */
	{RB0_HSIZE,0xc8},                                /*159	[51 := c8]		H_SIZE[7:0] (real/4)	0xc8*/
	{RB0_VSIZE,0x96},                                /*160	[52 := 96]		V_SIZE[7:0] (real/4)	0x96*/
	{RB0_XOFFL,0x00},                                /*161	[53 := 00]		OFFSET_X[7:0]SET(x)	0x00*/
	{RB0_YOFFL,0x00},                                /*162	[54 := 00]		OFFSET_Y[7:0]	0x00*/
	{RB0_VHYX,0x00},                                 /*163	[55 := 00]		Offset and size completion	 YOFF_SET(00)/YOFF_SET(0)  |  XOFF_SET(00)/XOFF_SET(0) */
	{RB0_ZMOW,0xc8},                                 /*164	[5a := c8]		Zoom: Out Width  OUTW[7:0] (real/4)	 OUTW_SET(c8)/OUTW_SET(200) */
	{RB0_ZMOH,0x96},                                 /*165	[5b := 96]		Zoom: Out Height OUTH[7:0] (real/4)	 OUTH_SET(96)/OUTH_SET(150) */
	{RB0_ZMHH,0x00},                                 /*166	[5c := 00]		Zoom: Speed and H&W completion	 ZSPEED_SET(00)/ZSPEED_SET(0)  |  OUTW_SET(00)/OUTW_SET(0) */
	{RB0_R_DVP_SP,0x00},                             /*167	[d3 := 00]		DVP output speed control	0x00*/
	{RB0_CTRL1,0xed},                                /*168	[c3 := ed]		DSP Module enable 1	 CIP(80)/CIP  |  DMY(40)/DMY  |  RAW_GMA(20)/RAW_GMA  |  AWB(08)/AWB  |  AWB_GAIN(04)/AWB_GAIN  |  PRE(01)/PRE */
	{RB0_RSVD_7f,0x00},                              /*169	[7f := 00]		RB0_RSVD_7f	0x00*/
	{RB0_IMAGE_MODE,0x00},                           /*170	[da := 00]		Image Output Format Select	 YUV422(00)/YUV422 */
	{RB0_RSVD_e5,0x1f},                              /*171	[e5 := 1f]		RB0_RSVD_e5	0x1f*/
	{RB0_RSVD_e1,0x67},                              /*172	[e1 := 67]		RB0_RSVD_e1	0x67*/
	{RB0_RESET,0x00},                                /*173	[e0 := 00]		Reset	0x00*/
	{RB0_RSVD_dd,0x7f},                              /*174	[dd := 7f]		RB0_RSVD_dd	0x7f*/
	{RB0_R_BYPASS,0x00},                             /*175	[05 := 00]		Bypass DSP	 Use the internal DSP/USE_DSP */
	{RB0_RSVD_12,0x40},                              /*176	[12 := 40]		RB0_RSVD_12	0x40*/
	{RB0_R_DVP_SP,0x04},                             /*177	[d3 := 04]		DVP output speed control	0x04*/
	{RB0_HSIZE8,0x16},                               /*178	[c0 := 16]		Image Horizontal Size HSIZE[10:3]	 SET(16)/SET(22) */
	{RB0_VSIZE8,0x12},                               /*179	[c1 := 12]		Image Vertical Size VSIZE[10:3]	 SET(12)/SET(18) */
	{RB0_SIZEL,0x00},                                /*180	[8c := 00]		Image Size Completion	 HSIZE8_SET(00)/HSIZE8_SET(0)  |  VSIZE8_SET(00)/VSIZE8_SET(0) */
	{RB0_CTRL2,0x3d},                                /*181	[86 := 3d]		DSP Module enable 2	 DCW_EN(20)/DCW_EN  |  SDE_EN(10)/SDE_EN  |  UV_ADJ_EN(08)/UV_ADJ_EN  |  UV_AVG_EN(04)/UV_AVG_EN  |  CMX_EN(01)/CMX_EN */
	{RB0_CTRL,0x00},                                 /*182	[50 := 00]		RB0_CTRL	 V_DIV_SET(00)/V_DIV_SET(0)  |  H_DIV_SET(00)/H_DIV_SET(0) */
	{RB0_HSIZE,0x2c},                                /*183	[51 := 2c]		H_SIZE[7:0] (real/4)	0x2c*/
	{RB0_VSIZE,0x24},                                /*184	[52 := 24]		V_SIZE[7:0] (real/4)	0x24*/
	{RB0_XOFFL,0x00},                                /*185	[53 := 00]		OFFSET_X[7:0]SET(x)	0x00*/
	{RB0_YOFFL,0x00},                                /*186	[54 := 00]		OFFSET_Y[7:0]	0x00*/
	{RB0_VHYX,0x00},                                 /*187	[55 := 00]		Offset and size completion	 YOFF_SET(00)/YOFF_SET(0)  |  XOFF_SET(00)/XOFF_SET(0) */
	{RB0_ZMOW,0x2c},                                 /*188	[5a := 2c]		Zoom: Out Width  OUTW[7:0] (real/4)	 OUTW_SET(2c)/OUTW_SET(44) */
	{RB0_ZMOH,0x24},                                 /*189	[5b := 24]		Zoom: Out Height OUTH[7:0] (real/4)	 OUTH_SET(24)/OUTH_SET(36) */
	{RB0_ZMHH,0x00},                                 /*190	[5c := 00]		Zoom: Speed and H&W completion	 ZSPEED_SET(00)/ZSPEED_SET(0)  |  OUTW_SET(00)/OUTW_SET(0) */
	{0xFF,0xFF}
};

