
#include "hideep.h"

#define TEST_MODE_COMMAND_ADDR 0x0804
#define TX_NUM 35
#define RX_NUM 58

const unsigned char CMP_TYPICAL_SPEC[TX_NUM*RX_NUM] = {
	65,	66,	65,	65,	64,	64,	63,	63,	62,	61,	61,	61,	60,	60,	60,	59,	59,	59,	59,	58,	58,	58,	58,	58,	57,	57,	58,	57,	57,	58,	57,	57,	58,	58,	58,	58,	58,	59,	58,	59,	59,	59,	59,	59,	60,	60,	60,	61,	61,	61,	62,	62,	63,	63,	64,	64,	65,	64,
	65,	66,	65,	65,	64,	64,	63,	63,	62,	62,	62,	61,	61,	61,	60,	60,	60,	60,	59,	59,	59,	59,	59,	59,	58,	58,	59,	58,	58,	58,	58,	58,	58,	59,	58,	58,	59,	59,	59,	59,	59,	59,	60,	60,	61,	60,	60,	62,	61,	62,	62,	62,	63,	64,	64,	64,	65,	64,
	65,	66,	65,	65,	64,	64,	63,	63,	63,	62,	62,	61,	61,	61,	61,	60,	60,	60,	59,	59,	59,	59,	59,	59,	58,	58,	59,	58,	58,	58,	58,	58,	58,	59,	58,	58,	59,	59,	59,	59,	60,	60,	60,	60,	61,	60,	61,	62,	61,	62,	62,	63,	63,	64,	64,	64,	65,	64,
	66,	66,	66,	65,	65,	64,	64,	63,	63,	62,	62,	61,	61,	61,	61,	60,	60,	60,	60,	59,	59,	59,	59,	59,	58,	58,	59,	58,	58,	59,	58,	58,	58,	59,	59,	59,	59,	59,	59,	59,	60,	60,	60,	60,	61,	61,	61,	62,	62,	62,	63,	63,	63,	64,	65,	64,	65,	64,
	66,	67,	66,	66,	65,	65,	64,	64,	63,	62,	62,	62,	61,	61,	61,	60,	60,	60,	60,	60,	59,	59,	59,	59,	59,	59,	59,	59,	59,	59,	59,	58,	59,	59,	59,	59,	60,	60,	59,	60,	60,	60,	60,	61,	61,	61,	61,	62,	62,	62,	63,	63,	64,	64,	65,	65,	66,	65,
	66,	67,	66,	66,	65,	65,	64,	64,	63,	63,	63,	62,	62,	62,	62,	61,	61,	61,	60,	60,	60,	60,	60,	60,	59,	59,	60,	59,	59,	59,	59,	59,	59,	60,	59,	59,	60,	60,	60,	60,	60,	60,	61,	61,	62,	61,	61,	63,	62,	63,	63,	63,	64,	65,	65,	65,	66,	65,
	67,	67,	66,	66,	65,	65,	65,	64,	64,	63,	63,	62,	62,	62,	62,	61,	61,	61,	61,	60,	60,	60,	60,	60,	59,	59,	60,	59,	59,	60,	59,	59,	59,	60,	60,	60,	60,	61,	60,	60,	61,	61,	61,	61,	62,	62,	62,	63,	63,	63,	64,	64,	64,	65,	65,	65,	66,	65,
	67,	68,	67,	67,	66,	66,	65,	65,	64,	64,	63,	63,	62,	63,	62,	62,	62,	62,	61,	61,	61,	61,	61,	61,	60,	60,	61,	60,	60,	60,	60,	60,	60,	60,	60,	60,	61,	61,	61,	61,	61,	61,	62,	62,	62,	62,	62,	63,	63,	63,	64,	64,	65,	65,	66,	66,	67,	66,
	67,	68,	67,	67,	66,	66,	65,	65,	65,	64,	64,	63,	63,	63,	63,	62,	62,	62,	62,	61,	61,	61,	61,	61,	60,	60,	61,	60,	60,	61,	60,	60,	61,	61,	61,	61,	61,	61,	61,	61,	62,	62,	62,	62,	63,	63,	63,	64,	64,	64,	65,	65,	65,	66,	66,	66,	67,	66,
	68,	69,	68,	68,	67,	67,	66,	66,	65,	65,	65,	64,	64,	64,	64,	63,	63,	63,	62,	62,	62,	62,	62,	62,	61,	61,	62,	61,	61,	61,	61,	61,	61,	62,	61,	61,	62,	62,	62,	62,	62,	62,	63,	63,	63,	63,	63,	64,	64,	64,	65,	65,	66,	66,	67,	67,	68,	67,
	69,	69,	69,	68,	68,	67,	67,	66,	66,	65,	65,	64,	64,	64,	64,	63,	63,	63,	63,	63,	62,	63,	63,	62,	62,	62,	62,	62,	62,	62,	62,	62,	62,	62,	62,	62,	63,	63,	63,	63,	63,	63,	63,	64,	64,	64,	64,	65,	65,	65,	66,	66,	66,	67,	68,	68,	68,	68,
	69,	70,	69,	69,	68,	68,	67,	67,	67,	66,	66,	65,	65,	65,	65,	64,	64,	64,	64,	63,	63,	63,	63,	63,	62,	62,	63,	62,	62,	63,	62,	62,	63,	63,	63,	63,	63,	64,	63,	63,	64,	64,	64,	64,	65,	64,	65,	66,	66,	66,	66,	67,	67,	68,	68,	68,	69,	68,
	70,	70,	70,	70,	69,	69,	68,	68,	67,	66,	66,	66,	65,	66,	65,	65,	65,	65,	64,	64,	64,	64,	64,	64,	63,	63,	64,	63,	63,	63,	63,	63,	63,	64,	64,	63,	64,	64,	64,	64,	65,	65,	65,	65,	66,	65,	65,	66,	66,	66,	67,	67,	68,	68,	69,	69,	69,	69,
	71,	71,	71,	70,	70,	69,	69,	69,	68,	67,	67,	67,	66,	66,	66,	65,	66,	66,	65,	65,	65,	65,	65,	65,	64,	64,	65,	64,	64,	64,	64,	64,	64,	64,	64,	64,	65,	65,	65,	65,	65,	65,	66,	66,	66,	66,	66,	67,	67,	67,	68,	68,	68,	69,	69,	69,	70,	69,
	71,	72,	71,	71,	70,	70,	70,	69,	69,	68,	68,	67,	67,	67,	67,	66,	67,	67,	66,	66,	65,	66,	66,	66,	65,	65,	66,	65,	65,	65,	65,	65,	65,	65,	65,	65,	66,	66,	66,	66,	66,	66,	67,	67,	67,	67,	67,	68,	68,	68,	69,	69,	69,	70,	70,	70,	71,	70,
	72,	73,	72,	72,	71,	71,	70,	70,	70,	69,	69,	68,	68,	68,	68,	67,	67,	68,	67,	67,	66,	67,	67,	67,	66,	66,	67,	66,	66,	66,	66,	66,	66,	67,	66,	66,	67,	67,	67,	67,	67,	67,	68,	68,	68,	68,	68,	69,	69,	69,	70,	70,	70,	71,	71,	71,	72,	71,
	73,	74,	74,	73,	72,	72,	71,	71,	71,	70,	70,	69,	69,	69,	69,	68,	68,	69,	68,	68,	67,	68,	68,	68,	67,	67,	67,	67,	67,	67,	67,	67,	67,	68,	67,	67,	68,	68,	68,	68,	68,	68,	68,	68,	69,	69,	69,	70,	70,	70,	71,	71,	71,	72,	72,	72,	73,	72,
	74,	74,	74,	74,	73,	73,	72,	72,	72,	71,	71,	70,	70,	70,	70,	69,	69,	69,	69,	69,	68,	69,	69,	68,	68,	68,	68,	68,	68,	68,	68,	68,	68,	68,	68,	68,	69,	69,	69,	69,	69,	69,	69,	69,	70,	70,	70,	71,	71,	71,	71,	72,	72,	73,	73,	73,	73,	73,
	75,	75,	75,	75,	74,	74,	73,	73,	73,	72,	72,	71,	71,	71,	71,	70,	70,	70,	70,	70,	69,	70,	70,	69,	69,	69,	70,	69,	69,	69,	69,	69,	69,	70,	69,	69,	70,	70,	70,	70,	70,	70,	70,	70,	71,	71,	71,	72,	72,	72,	72,	73,	73,	74,	74,	74,	74,	74,
	76,	76,	76,	76,	75,	75,	74,	74,	74,	73,	73,	72,	72,	72,	72,	72,	72,	72,	71,	71,	71,	71,	71,	71,	71,	70,	71,	70,	70,	71,	70,	70,	70,	71,	71,	71,	71,	71,	71,	71,	71,	71,	72,	72,	72,	72,	72,	73,	73,	73,	74,	74,	74,	75,	75,	75,	76,	75,
	77,	77,	77,	77,	76,	76,	75,	75,	75,	74,	74,	74,	73,	74,	73,	73,	73,	73,	72,	72,	72,	72,	72,	72,	72,	72,	72,	72,	72,	72,	72,	71,	72,	72,	72,	72,	72,	72,	72,	72,	73,	73,	73,	73,	73,	73,	73,	74,	74,	74,	75,	75,	75,	76,	76,	76,	76,	76,
	78,	78,	78,	78,	77,	77,	76,	76,	76,	75,	75,	75,	75,	75,	75,	74,	74,	74,	74,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	73,	74,	74,	74,	74,	74,	74,	74,	75,	74,	74,	75,	75,	75,	76,	76,	76,	77,	77,	77,	77,	77,
	79,	80,	79,	79,	78,	78,	78,	78,	77,	77,	77,	76,	76,	76,	76,	75,	76,	76,	75,	75,	75,	75,	75,	75,	74,	74,	75,	74,	74,	75,	74,	74,	74,	75,	75,	75,	75,	75,	75,	75,	75,	75,	76,	75,	76,	76,	76,	77,	76,	77,	77,	77,	77,	78,	78,	78,	79,	78,
	80,	81,	80,	80,	80,	79,	79,	79,	79,	78,	78,	77,	77,	78,	77,	77,	77,	77,	76,	76,	76,	76,	76,	76,	76,	76,	76,	76,	76,	76,	76,	75,	76,	76,	76,	76,	76,	76,	76,	76,	77,	77,	77,	77,	77,	77,	77,	78,	78,	78,	78,	78,	79,	79,	79,	80,	80,	80,
	82,	82,	81,	81,	81,	81,	80,	80,	80,	79,	79,	79,	79,	79,	79,	78,	78,	78,	78,	78,	77,	78,	78,	78,	77,	77,	77,	77,	77,	77,	77,	77,	77,	77,	77,	77,	78,	78,	78,	78,	78,	78,	78,	78,	79,	79,	78,	79,	79,	79,	80,	80,	80,	80,	81,	81,	81,	81,
	83,	83,	83,	82,	82,	82,	81,	81,	81,	81,	81,	80,	80,	80,	80,	79,	80,	80,	79,	79,	79,	79,	79,	79,	79,	79,	79,	79,	79,	79,	79,	78,	79,	79,	79,	79,	79,	79,	79,	79,	80,	80,	80,	79,	80,	80,	80,	81,	80,	80,	81,	81,	81,	82,	82,	82,	82,	82,
	84,	84,	84,	84,	83,	83,	82,	83,	82,	82,	82,	81,	82,	82,	82,	81,	81,	81,	81,	81,	81,	81,	81,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	81,	81,	81,	81,	81,	81,	81,	81,	82,	81,	81,	82,	82,	82,	82,	82,	82,	83,	83,	83,	84,	83,
	85,	86,	85,	85,	85,	85,	84,	84,	84,	84,	84,	83,	83,	83,	83,	83,	83,	83,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	82,	83,	83,	83,	82,	83,	83,	83,	84,	83,	83,	84,	84,	84,	85,	85,	85,	85,	85,
	87,	87,	87,	86,	86,	86,	85,	86,	85,	85,	85,	85,	85,	85,	85,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	83,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	84,	85,	85,	84,	85,	85,	85,	85,	85,	85,	86,	86,	86,	86,	86,
	88,	88,	88,	88,	87,	88,	87,	87,	87,	86,	87,	86,	86,	86,	86,	86,	86,	86,	86,	86,	85,	85,	86,	85,	85,	85,	85,	85,	85,	85,	85,	85,	85,	85,	85,	85,	86,	86,	86,	86,	86,	86,	86,	86,	86,	86,	86,	87,	86,	86,	87,	87,	87,	87,	87,	87,	88,	87,
	89,	89,	89,	89,	89,	89,	88,	89,	88,	88,	88,	88,	88,	88,	88,	87,	88,	88,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	86,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	87,	88,	87,	87,	88,	88,	88,	88,	88,	88,	89,	89,	89,	89,	89,
	91,	91,	91,	90,	90,	90,	90,	90,	90,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	88,	88,	89,	88,	88,	88,	88,	88,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	89,	90,	89,	90,	90,	90,	90,	90,	90,
	92,	92,	92,	92,	91,	91,	91,	91,	91,	91,	91,	91,	91,	91,	91,	90,	91,	91,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	90,	91,	91,	90,	91,	91,	90,	91,	91,	91,	91,	91,	91,	92,	91,	91,	92,	92,
	93,	93,	93,	93,	93,	93,	92,	93,	93,	93,	93,	92,	92,	92,	93,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	92,	93,	92,	92,	93,	92,	93,	93,	93,	93,	93,	93,
	95,	95,	95,	95,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	94,	95,	94,	94,	94,	95
};

const unsigned short HIDEEP_4RCF_TYPICAL_SPEC[TX_NUM] = {
	1263,	1406,	1436,	1443,	1442,	1437,	1438,	1433,	1426,	1431,	1432,	1435,	1436,	1439,	1416,	1416,	1409,	1427,	1424,	1420,	1417,	1417,	1414,	1412,	1407,	1403,	1397,	1388,	1378,	1368,	1365,	1352,	1337,	1304,	1146
};

const unsigned short HIDEEP_EOP_TYPICAL_SPEC[RX_NUM] = {
	1005,	1064,	1056,	1059,	1053,	1052,	1046,	1043,	1035,	1033,	1028,	1024,	1021,	1018,	1016,	1013,	1005,	1001,	1003,	1012,	1015,	1017,	1019,	1028,	1024,	1031,	1031,	1035,	1046,	1048,	1053,	1052,	1056,	1061,	1070,	1071,	1072,	1073,	1082,	1083,	1084,	1088,	1096,	1095,	1101,	1103,	1103,	1113,	1113,	1114,	1120,	1118,	1123,	1123,	1127,	1125,	1141,	1055
};

const unsigned short HIDEEP_4RC_TYPICAL = 940;

void hideep_test_mode(struct hideep_data *ts)
{
    unsigned char *buf;
    unsigned short *framebuf;
    unsigned short *tempframe;
	unsigned char r,t;
    int ret = 0;
	unsigned int resultCNT;
	int retry = 0;


    buf = kmalloc(sizeof(buf)*TX_NUM*RX_NUM*2, GFP_KERNEL);
  if(buf ==NULL)
    {
        dbg_err("can't not memmory alloc");
        goto exit_buf_alloc_hideep_test_mode;
    }
    
    framebuf = kmalloc(sizeof(framebuf)*TX_NUM*RX_NUM, GFP_KERNEL);
  if(framebuf ==NULL)
    {
        dbg_err("can't not memmory alloc");
        goto exit_framebuf_alloc_hideep_test_mode;
    }
  	

    tempframe = kmalloc(sizeof(tempframe)*TX_NUM*RX_NUM, GFP_KERNEL);
  if(tempframe ==NULL)
    {
        dbg_err("can't not memmory alloc");
        goto exit_tempframe_alloc_hideep_test_mode;
    }
  	

	//4RCF
	hideep_reset();
	mdelay(HIDEEP_RESET_DELAY);
	buf[0] = 0x03;
	//buf[1] = 0x03;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 1, buf);
	dbg_log("ret = %d", ret);
	mdelay(HIDEEP_IMAGE_DELAY);
	/*
	ret = hideep_i2c_read(ts->client, 0x1008, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	*/
	retry =1;
	do{
		ret = hideep_i2c_read(ts->client, 0x1008, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);
		if(*((unsigned char *)framebuf)=='G')
		    break;
		mdelay(HIDEEP_IMAGE_DELAY);
		retry--;
		dbg_err("retry = %d", retry);
	}while(retry);

	//Inspection
	for(t=0; t<TX_NUM; t++)
	{
		resultCNT = 0;
		for (r=0; r<RX_NUM; r++)
		{
            dbg_log("4RC: index = %d, framebuf = %d, HIDEEP_4RCF_TYPICAL_SPEC = %d", 
                r+t*RX_NUM,
                framebuf[r+t*RX_NUM],
                HIDEEP_4RCF_TYPICAL_SPEC[t]);
			
			if (HIDEEP_4RCF_TYPICAL_SPEC[t]*13 < framebuf[r+t*RX_NUM]*10) // SPEC : +30% // ??
			{
				resultCNT++;
			}
		}

		if (resultCNT > 0)
		{
			ts->TXshortResult[t] = 1;
			dbg_log("TX Channel Short = %d", t);
		}
		else
		{
			ts->TXshortResult[t] = 0;
			dbg_log("4RCF Pass = %d", t);
		}
	}


	//EOP
	hideep_reset();
	mdelay(HIDEEP_RESET_DELAY);
	buf[0] = 0x02;
	//buf[1] = 0x02;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 1, buf);
	dbg_log("ret = %d", ret);

	mdelay(HIDEEP_IMAGE_DELAY);
	/*
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	*/
	retry =1;
	do{
		ret = hideep_i2c_read(ts->client, 0x1008, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);
		if(*((unsigned char *)framebuf)=='G')
		    break;
		mdelay(HIDEEP_IMAGE_DELAY);
		retry--;
		dbg_err("retry = %d", retry);
	}while(retry);

	//Inspection
	for(r=0; r<RX_NUM; r++)
	{
		resultCNT = 0;
		for (t=0; t<TX_NUM; t++)
		{
            dbg_log("EOP: index = %d, framebuf = %d, HIDEEP_EOP_TYPICAL_SPEC = %d", 
                r+t*RX_NUM,
                framebuf[r+t*RX_NUM],
                HIDEEP_EOP_TYPICAL_SPEC[r]);

			if (HIDEEP_EOP_TYPICAL_SPEC[r]*12 < framebuf[r+t*RX_NUM]*10) // SPEC : +20%
			{
				resultCNT++;
			}
		}

		if (resultCNT > 0)
		{
			ts->RXshortResult[r] = 1;
			dbg_log("RX Channel Short = %d", r);
		}
		else
		{
			ts->RXshortResult[r] = 0;
			dbg_log("EOP Pass = %d", r);
		}
	}



	//4RC
	hideep_reset();
	mdelay(HIDEEP_RESET_DELAY);
    buf[0] = 0x00;
    //buf[1] = 0x00;
    ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 1, buf);
    dbg_log("ret = %d", ret);

	mdelay(HIDEEP_IMAGE_DELAY);
	/*
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	*/
	retry =1;
	do{
		ret = hideep_i2c_read(ts->client, 0x1008, (TX_NUM*RX_NUM)*2, (unsigned char *)tempframe);
		if(*((unsigned char *)tempframe)=='G')
		    break;
		mdelay(HIDEEP_IMAGE_DELAY);
		retry--;
		dbg_err("retry = %d", retry);
	}while(retry);
	//1RC
	hideep_reset();
	mdelay(HIDEEP_RESET_DELAY);
	buf[0] = 0x01;
	//buf[1] = 0x01;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 1, buf);
	dbg_log("ret = %d", ret);

	mdelay(HIDEEP_IMAGE_DELAY);
	/*
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	*/
	//ret = hideep_i2c_read(ts->client, 0x1008, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);
	retry =1;
	do{
		ret = hideep_i2c_read(ts->client, 0x1008, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);
		if(*((unsigned char *)framebuf)=='G')
		    break;
		mdelay(HIDEEP_IMAGE_DELAY);
		retry--;
		dbg_err("retry = %d", retry);
	}while(retry);

	//Calculation CMP 
	for(t=0; t<TX_NUM; t++)
	{
		for (r=0; r<RX_NUM; r++)
		{
			tempframe[r+t*RX_NUM] = tempframe[r+t*RX_NUM]>0?tempframe[r+t*RX_NUM]:1;
			framebuf[r+t*RX_NUM] = framebuf[r+t*RX_NUM]*100 / tempframe[r+t*RX_NUM];

            dbg_log("CMP: index = %d, framebuf(CMP) = %d, tempframe(4RC) = %d", 
                r+t*RX_NUM,
                framebuf[r+t*RX_NUM],
                tempframe[r+t*RX_NUM]);
		}
	}	

	//Inspection
	for (r=0; r<RX_NUM; r++)
	{
		if (ts->RXshortResult[r] == 0)
		{
			resultCNT = 0;
			for(t=0; t<TX_NUM; t++)
			{
				if (HIDEEP_4RC_TYPICAL*7 > tempframe[r+t*RX_NUM]*10) // SPEC : -30%
				{
					resultCNT++;
				}
				else
				{
					resultCNT = 0;
				}
			}
			if (resultCNT > 1)
			{
				ts->RXopenResult[r] = 1;
				dbg_log("RX Channel Open = %d", r);
			}
			else
			{
				ts->RXopenResult[r] = 0;
				dbg_log("RX 4RC Pass = %d", r);
			}
		}
		else
		{
			ts->RXopenResult[r] = 0;
			dbg_log("Skip to check 4RC RX = %d", r);
		}
	}
	

	for(t=0; t<TX_NUM; t++)
	{
		if (ts->TXshortResult[t] == 0)
		{
			resultCNT = 0;
			for (r=0; r<RX_NUM; r++)
			{
				if ((CMP_TYPICAL_SPEC[r+t*RX_NUM]*8 > framebuf[r+t*RX_NUM]*10)&&(ts->RXopenResult[r] == 0)) // SPEC : -20%
				{
					resultCNT++;
				}
			}

			if (resultCNT > 0)
			{
				ts->TXopenResult[t] = 1;
				dbg_log("TX Channel Open = %d", t);
			}
			else
			{
				ts->TXopenResult[t] = 0;
				dbg_log("TX CMP Pass = %d", t);
			}
		}
		else
		{
			ts->TXopenResult[t] = 0;
			dbg_log("Skip to check CMP TX = %d", t);
		}
	}
	hideep_reset();
	mdelay(100);
	kfree(tempframe);
	kfree(framebuf);
	kfree(buf);
	return;
	
exit_tempframe_alloc_hideep_test_mode:
	kfree(framebuf);
exit_framebuf_alloc_hideep_test_mode:
	kfree(framebuf);
exit_buf_alloc_hideep_test_mode:
	return;
}

