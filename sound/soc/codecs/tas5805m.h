// SPDX-License-Identifier: GPL-2.0
//
// Driver for the TAS5805M Audio Amplifier
//
// Author: Andy Liu <andy-liu@ti.com>
// Author: Daniel Beer <daniel.beer@igorinstitute.com>
// Author: Felix Kaechele <felix@kaechele.ca> - TAS5872M support

#ifndef __TAS5805M_H__
#define __TAS5805M_H__

/* TAS5805M
 *
 * Datasheet-defined registers on page 0, book 0
 */
#define REG_PAGE		0x00
#define REG_DEVICE_CTRL_1	0x02
#define REG_DEVICE_CTRL_2	0x03
#define REG_SIG_CH_CTRL		0x28
#define REG_SAP_CTRL_1		0x33
#define REG_FS_MON		0x37
#define REG_BCK_MON		0x38
#define REG_CLKDET_STATUS	0x39
#define REG_VOL_CTL		0x4c
#define REG_AGAIN		0x54
#define REG_ADR_PIN_CTRL	0x60
#define REG_ADR_PIN_CONFIG	0x61
#define REG_CHAN_FAULT		0x70
#define REG_GLOBAL_FAULT1	0x71
#define REG_GLOBAL_FAULT2	0x72
#define REG_FAULT		0x78
#define REG_BOOK		0x7f

/* DEVICE_CTRL_2 register values */
#define DCTRL2_MODE_DEEP_SLEEP	0x00
#define DCTRL2_MODE_SLEEP	0x01
#define DCTRL2_MODE_HIZ		0x02
#define DCTRL2_MODE_PLAY	0x03

#define DCTRL2_MUTE		0x08
#define DCTRL2_DIS_DSP		0x10

/* This sequence of register writes must always be sent, prior to the
 * 5ms delay while we wait for the DSP to boot.
 */
static const uint8_t tas5805m_dsp_init[] = {
	0x00, 0x00, /* Page 0 */
	0x7f, 0x00, /* Book 0 */
	0x03, 0x02, /* DSP power up / DAC Hi-Z */
	0x01, 0x11, /* Reset registers and modules */
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00, /* Page 0 */
	0x7f, 0x00, /* Book 0 */
	0x03, 0x02, /* DSP power up / DAC Hi-Z */
};

static const uint32_t tas5805m_volume[] = {
	0x0000001B, /*   0, -110dB */ 0x0000001E, /*   1, -109dB */
	0x00000021, /*   2, -108dB */ 0x00000025, /*   3, -107dB */
	0x0000002A, /*   4, -106dB */ 0x0000002F, /*   5, -105dB */
	0x00000035, /*   6, -104dB */ 0x0000003B, /*   7, -103dB */
	0x00000043, /*   8, -102dB */ 0x0000004B, /*   9, -101dB */
	0x00000054, /*  10, -100dB */ 0x0000005E, /*  11,  -99dB */
	0x0000006A, /*  12,  -98dB */ 0x00000076, /*  13,  -97dB */
	0x00000085, /*  14,  -96dB */ 0x00000095, /*  15,  -95dB */
	0x000000A7, /*  16,  -94dB */ 0x000000BC, /*  17,  -93dB */
	0x000000D3, /*  18,  -92dB */ 0x000000EC, /*  19,  -91dB */
	0x00000109, /*  20,  -90dB */ 0x0000012A, /*  21,  -89dB */
	0x0000014E, /*  22,  -88dB */ 0x00000177, /*  23,  -87dB */
	0x000001A4, /*  24,  -86dB */ 0x000001D8, /*  25,  -85dB */
	0x00000211, /*  26,  -84dB */ 0x00000252, /*  27,  -83dB */
	0x0000029A, /*  28,  -82dB */ 0x000002EC, /*  29,  -81dB */
	0x00000347, /*  30,  -80dB */ 0x000003AD, /*  31,  -79dB */
	0x00000420, /*  32,  -78dB */ 0x000004A1, /*  33,  -77dB */
	0x00000532, /*  34,  -76dB */ 0x000005D4, /*  35,  -75dB */
	0x0000068A, /*  36,  -74dB */ 0x00000756, /*  37,  -73dB */
	0x0000083B, /*  38,  -72dB */ 0x0000093C, /*  39,  -71dB */
	0x00000A5D, /*  40,  -70dB */ 0x00000BA0, /*  41,  -69dB */
	0x00000D0C, /*  42,  -68dB */ 0x00000EA3, /*  43,  -67dB */
	0x0000106C, /*  44,  -66dB */ 0x0000126D, /*  45,  -65dB */
	0x000014AD, /*  46,  -64dB */ 0x00001733, /*  47,  -63dB */
	0x00001A07, /*  48,  -62dB */ 0x00001D34, /*  49,  -61dB */
	0x000020C5, /*  50,  -60dB */ 0x000024C4, /*  51,  -59dB */
	0x00002941, /*  52,  -58dB */ 0x00002E49, /*  53,  -57dB */
	0x000033EF, /*  54,  -56dB */ 0x00003A45, /*  55,  -55dB */
	0x00004161, /*  56,  -54dB */ 0x0000495C, /*  57,  -53dB */
	0x0000524F, /*  58,  -52dB */ 0x00005C5A, /*  59,  -51dB */
	0x0000679F, /*  60,  -50dB */ 0x00007444, /*  61,  -49dB */
	0x00008274, /*  62,  -48dB */ 0x0000925F, /*  63,  -47dB */
	0x0000A43B, /*  64,  -46dB */ 0x0000B845, /*  65,  -45dB */
	0x0000CEC1, /*  66,  -44dB */ 0x0000E7FB, /*  67,  -43dB */
	0x00010449, /*  68,  -42dB */ 0x0001240C, /*  69,  -41dB */
	0x000147AE, /*  70,  -40dB */ 0x00016FAA, /*  71,  -39dB */
	0x00019C86, /*  72,  -38dB */ 0x0001CEDC, /*  73,  -37dB */
	0x00020756, /*  74,  -36dB */ 0x000246B5, /*  75,  -35dB */
	0x00028DCF, /*  76,  -34dB */ 0x0002DD96, /*  77,  -33dB */
	0x00033718, /*  78,  -32dB */ 0x00039B87, /*  79,  -31dB */
	0x00040C37, /*  80,  -30dB */ 0x00048AA7, /*  81,  -29dB */
	0x00051884, /*  82,  -28dB */ 0x0005B7B1, /*  83,  -27dB */
	0x00066A4A, /*  84,  -26dB */ 0x000732AE, /*  85,  -25dB */
	0x00081385, /*  86,  -24dB */ 0x00090FCC, /*  87,  -23dB */
	0x000A2ADB, /*  88,  -22dB */ 0x000B6873, /*  89,  -21dB */
	0x000CCCCD, /*  90,  -20dB */ 0x000E5CA1, /*  91,  -19dB */
	0x00101D3F, /*  92,  -18dB */ 0x0012149A, /*  93,  -17dB */
	0x00144961, /*  94,  -16dB */ 0x0016C311, /*  95,  -15dB */
	0x00198A13, /*  96,  -14dB */ 0x001CA7D7, /*  97,  -13dB */
	0x002026F3, /*  98,  -12dB */ 0x00241347, /*  99,  -11dB */
	0x00287A27, /* 100,  -10dB */ 0x002D6A86, /* 101,  -9dB */
	0x0032F52D, /* 102,   -8dB */ 0x00392CEE, /* 103,   -7dB */
	0x004026E7, /* 104,   -6dB */ 0x0047FACD, /* 105,   -5dB */
	0x0050C336, /* 106,   -4dB */ 0x005A9DF8, /* 107,   -3dB */
	0x0065AC8C, /* 108,   -2dB */ 0x00721483, /* 109,   -1dB */
	0x00800000, /* 110,    0dB */ 0x008F9E4D, /* 111,    1dB */
	0x00A12478, /* 112,    2dB */ 0x00B4CE08, /* 113,    3dB */
	0x00CADDC8, /* 114,    4dB */ 0x00E39EA9, /* 115,    5dB */
	0x00FF64C1, /* 116,    6dB */ 0x011E8E6A, /* 117,    7dB */
	0x0141857F, /* 118,    8dB */ 0x0168C0C6, /* 119,    9dB */
	0x0194C584, /* 120,   10dB */ 0x01C62940, /* 121,   11dB */
	0x01FD93C2, /* 122,   12dB */ 0x023BC148, /* 123,   13dB */
	0x02818508, /* 124,   14dB */ 0x02CFCC01, /* 125,   15dB */
	0x0327A01A, /* 126,   16dB */ 0x038A2BAD, /* 127,   17dB */
	0x03F8BD7A, /* 128,   18dB */ 0x0474CD1B, /* 129,   19dB */
	0x05000000, /* 130,   20dB */ 0x059C2F02, /* 131,   21dB */
	0x064B6CAE, /* 132,   22dB */ 0x07100C4D, /* 133,   23dB */
	0x07ECA9CD, /* 134,   24dB */ 0x08E43299, /* 135,   25dB */
	0x09F9EF8E, /* 136,   26dB */ 0x0B319025, /* 137,   27dB */
	0x0C8F36F2, /* 138,   28dB */ 0x0E1787B8, /* 139,   29dB */
	0x0FCFB725, /* 140,   30dB */ 0x11BD9C84, /* 141,   31dB */
	0x13E7C594, /* 142,   32dB */ 0x16558CCB, /* 143,   33dB */
	0x190F3254, /* 144,   34dB */ 0x1C1DF80E, /* 145,   35dB */
	0x1F8C4107, /* 146,   36dB */ 0x2365B4BF, /* 147,   37dB */
	0x27B766C2, /* 148,   38dB */ 0x2C900313, /* 149,   39dB */
	0x32000000, /* 150,   40dB */ 0x3819D612, /* 151,   41dB */
	0x3EF23ECA, /* 152,   42dB */ 0x46A07B07, /* 153,   43dB */
	0x4F3EA203, /* 154,   44dB */ 0x58E9F9F9, /* 155,   45dB */
	0x63C35B8E, /* 156,   46dB */ 0x6FEFA16D, /* 157,   47dB */
	0x7D982575, /* 158,   48dB */
};

#define TAS5805M_VOLUME_MAX	((int)ARRAY_SIZE(tas5805m_volume) - 1)
#define TAS5805M_VOLUME_MIN	0

/* TAS5782M
 *
 * Unfortunately, the TAS5782M datasheet doesn't have
 * neatly named registers
 */
#define TAS5782M_REG_1	0x01
#define TAS5782M_REG_2	0x02
#define TAS5782M_REG_3	0x03
#define TAS5782M_REG_0D	0x0d
#define TAS5782M_REG_25	0x25
#define TAS5782M_REG_2A	0x2a

#define TAS5782M_REG_1_RESET_REGISTERS	0x01
#define TAS5782M_REG_1_RESET_MODULES	0x10

#define TAS5782M_REG_2_MODE_POWER_ON	0x00
#define TAS5782M_REG_2_MODE_POWER_DOWN	0x01
#define TAS5782M_REG_2_MODE_STANDBY	0x10

#define TAS5782M_REG_3_NORMAL_VOLUME	0x00
#define TAS5782M_REG_3_MUTE_L		0x10
#define TAS5782M_REG_3_MUTE_R		0x01
#define TAS5782M_REG_3_MUTE		(TAS5782M_REG_3_MUTE_L | TAS5782M_REG_3_MUTE_R)

#define TAS5782M_REG_0D_SREF_SCLK	0x10

#define TAS5782M_REG_25_IGNORE_SCLK	        0x10
#define TAS5782M_REG_25_IGNORE_SCLK_HALT	0x08

#define TAS5782M_REG_2A_DAC_L_CH_L	0x10
#define TAS5782M_REG_2A_DAC_L_CH_R	0x20
#define TAS5782M_REG_2A_DAC_R_CH_L	0x02
#define TAS5782M_REG_2A_DAC_R_CH_R	0x01
#define TAS5782M_REG_2A_DAC_MUTE	0x00

static const uint8_t tas5782m_dsp_init[] = {
	REG_PAGE, 0x00, /* Page 0 */
	REG_BOOK, 0x00, /* Book 0 */
	TAS5782M_REG_2, (TAS5782M_REG_2_MODE_POWER_DOWN | TAS5782M_REG_2_MODE_STANDBY),
	TAS5782M_REG_1, (TAS5782M_REG_1_RESET_REGISTERS | TAS5782M_REG_1_RESET_MODULES),
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00,
	0x00, 0x00,
	TAS5782M_REG_3, TAS5782M_REG_3_MUTE, /* Mute L and R channel */
	TAS5782M_REG_2A, TAS5782M_REG_2A_DAC_MUTE, /* Mute L and R DAC data path */
	TAS5782M_REG_25, 0x18, /* Ignore MCLK and MCLK halt detection */
	0x0d, 0x10, /* PLL reference clock is SCLK */
	TAS5782M_REG_2, TAS5782M_REG_2_MODE_POWER_ON,
};

#endif /* __TAS5805M_H__ */
