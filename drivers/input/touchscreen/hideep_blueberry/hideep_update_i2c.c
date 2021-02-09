/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************/

#include "hideep.h"

/*******************************************************************************
 * basic operation to access touch ic
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *mount -t vfat32 /vendor /dev/block/mmcblk1p1
 *-----------------------------------------------------------------------------*/
#define PGM_BURST_WR
#define PGM_VERIFY
/*------------------------------------------------------------------------------
 * register map
 *-----------------------------------------------------------------------------*/
#define YRAM_BASE                    (0x40000000)
#define PERIPHERAL_BASE              (0x50000000)
#define ESI_BASE                     (PERIPHERAL_BASE + 0x00000000)
#define FLASH_BASE                   (PERIPHERAL_BASE + 0x01000000)
#define SYSCON_BASE                  (PERIPHERAL_BASE + 0x02000000)

#define SYSCON_MOD_CON               (SYSCON_BASE + 0x0000)
#define SYSCON_SPC_CON               (SYSCON_BASE + 0x0004)
#define SYSCON_CLK_CON               (SYSCON_BASE + 0x0008)
#define SYSCON_CLK_ENA               (SYSCON_BASE + 0x000C)
#define SYSCON_RST_CON               (SYSCON_BASE + 0x0010)
#define SYSCON_WDT_CON               (SYSCON_BASE + 0x0014)
#define SYSCON_WDT_CNT               (SYSCON_BASE + 0x0018)
#define SYSCON_PWR_CON               (SYSCON_BASE + 0x0020)
#define SYSCON_CHIP_ID               (SYSCON_BASE + 0x007C)
#define SYSCON_PGM_ID                (SYSCON_BASE + 0x00F4)

#define FLASH_CON                    (FLASH_BASE  + 0x0000)
#define FLASH_STA                    (FLASH_BASE  + 0x0004)
#define FLASH_CFG                    (FLASH_BASE  + 0x0008)
#define FLASH_TIM                    (FLASH_BASE  + 0x000C)
#define FLASH_CACHE_CFG              (FLASH_BASE  + 0x0010)

#define ESI_TX_INVALID				 (ESI_BASE	  + 0x0008)

/*------------------------------------------------------------------------*//**
 * flash commands
 *//*--------------------------------------------------------------------------*/
#define MERASE                       (0x00010000)
#define SERASE                       (0x00020000)
#define PERASE                       (0x00040000)
#define PROG                         (0x00080000)
#define WRONLY                       (0x00100000)
#define INF                          (0x00200000)

#define NVM_PAGE_SIZE                (128)

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define SET_FLASH_PIO(CE)        hideep_pgm_w_reg(client, FLASH_CON                    , 0x01 | ((CE) << 1))
#define SET_PIO_SIG(X, Y)        hideep_pgm_w_reg(client, (FLASH_BASE + 0x400000) + (X),                  Y)
#define	SET_FLASH_HWCONTROL		 hideep_pgm_w_reg(client, FLASH_CON, 0x00)
#define NVM_DEFAULT_PAGE         0
#define NVM_SFR_WPAGE            1
#define NVM_SFR_RPAGE            2



#define HIDEEP_ROM_MAX_SIZE								0xc000
#define HIDEEP_WHOLE_START_ADDR 				0
#define HIDEEP_WHOLE_LENGHT 						HIDEEP_ROM_MAX_SIZE
#define HIDEEP_BOOTLOADER_START_ADDR 		0
#define HIDEEP_BOOTLOADER_LENGHT 				0x300
#define HIDEEP_CODE_START_ADDR 					0x2c0
#define HIDEEP_CODE_LENGTH 						(HIDEEP_BOOTLOADER_START_ADDR-HIDEEP_CODE_START_ADDR)
#define HIDEEP_CRC_START_ADDR 					0x2c8
#define HIDEEP_CRC_LENGTH								8
#define HIDEEP_CUSTOM_INFO_START_ADDR 	0x2d8
#define HIDEEP_CUSTOM_INFO_LENGHT 			8
#define HIDEEP_CUSTOM_START_ADDR 				0x0
#define HIDEEP_CUSTOM_LENGTH 						HIDEEP_ROM_MAX_SIZE
#define HIDEEP_VR_INFO_START_ADDR 			0x2d8
#define HIDEEP_VR_INFO_LENGHT 					8
#define HIDEEP_VR_START_ADDR 						0x300
#define HIDEEP_VR_LENGTH								HIDEEP_ROM_MAX_SIZE


#define HIDEEP_BOOTLOADER_VERSION_ADDR_BASE     0x2e8
#define HIDEEP_BOOTLOADER_MAJOR_VERSION			(HIDEEP_BOOTLOADER_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_BOOTLOADER_MINOR_VERSION			(HIDEEP_BOOTLOADER_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_CODE_VERSION_ADDR_BASE           0x2ea
#define HIDEEP_CODE_MAJOR_VERSION			    (HIDEEP_CODE_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_CODE_MINOR_VERSION			    (HIDEEP_CODE_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_CUSTOM_VERSION_ADDR_BASE         0x2ec
#define HIDEEP_CUSTOM_MAJOR_VERSION			    (HIDEEP_CUSTOM_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_CUSTOM_MINOR_VERSION			    (HIDEEP_CUSTOM_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_VR_VERSION_ADDR_BASE             0x2ee
#define HIDEEP_VR_MAJOR_VERSION				    (HIDEEP_VR_VERSION_ADDR_BASE+__HIDEEP_MAJOR_VERSION__)
#define HIDEEP_VR_MINOR_VERSION				    (HIDEEP_VR_VERSION_ADDR_BASE+__HIDEEP_MINOR_VERSION__)
#define HIDEEP_FACTORY_ID		  				0x2f0
#define HIDEEP_UPDATE_ENTER_PGM_RETRY_TIME		10
#define HIDEEP_UPDATE_READ_ID_RETRY_TIME		5
#define HIDEEP_VERIFY_ENABLE
#define HIDEEP_AUTO_UPDATE_FROM_BIN
//#define HIDEEP_CHECK_INT_AFTER_REBOOT
int hideep_swd_reset(void);

#define HIDEEP_CHECK_VER_FROM_VR	3

struct s_hideep_being_updated
{
	unsigned int start;
	unsigned int length;
};
const struct s_hideep_being_updated hideep_bootloader_block[]=
{
		{HIDEEP_WHOLE_START_ADDR,HIDEEP_WHOLE_LENGHT},
};
const struct s_hideep_being_updated hideep_code_block[]=
{
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
		{HIDEEP_CODE_START_ADDR,HIDEEP_CODE_LENGTH},
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
};
const struct s_hideep_being_updated hideep_custom_block[]=
{
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CRC_START_ADDR,HIDEEP_CRC_LENGTH},
		{HIDEEP_CUSTOM_INFO_START_ADDR,HIDEEP_CUSTOM_INFO_LENGHT},
		{HIDEEP_CUSTOM_START_ADDR,HIDEEP_CUSTOM_LENGTH},
};
const struct s_hideep_being_updated hideep_vr_block[]=
{
		{HIDEEP_VR_START_ADDR,HIDEEP_VR_LENGTH},
		{HIDEEP_BOOTLOADER_START_ADDR,HIDEEP_BOOTLOADER_LENGHT},
};
struct s_hideep_fw_update_block_info
{
	int block_count;
	struct s_hideep_being_updated *pblock;
};

const struct s_hideep_fw_update_block_info hideep_fw_update_blocks_info[]=
{
		{
				sizeof(hideep_bootloader_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_bootloader_block,
		},
		{
				sizeof(hideep_code_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_code_block,
		},
		{
				sizeof(hideep_custom_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_custom_block,
		},
		{
				sizeof(hideep_vr_block)/sizeof(struct s_hideep_being_updated),
				(struct s_hideep_being_updated *)hideep_vr_block,
		},
};

#if 0
static unsigned short hideep_version_addr[]={
	TOUCH_READ_BL_VER_ADDR,
	TOUCH_READ_CODE_VER_ADDR,
	TOUCH_READ_CUST_VER_ADDR,
	TOUCH_READ_VR_VER_ADDR,
	TOUCH_READ_FACTORY_ID_ADDR
};
#endif

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
typedef struct pgm_packet
{
    union
    {
        u8   b[8];
        u32  w[2];
    }header;

    u32 payload[NVM_PAGE_SIZE/sizeof(u32)];
}PGM_PACKET;
extern struct hideep_data *private_ts;
int read_trim_data(struct i2c_client *client);
#define NVM_W_SFR(x,y)  { SET_FLASH_PIO(1);  SET_PIO_SIG(x, y); SET_FLASH_PIO(0); }

/*******************************************************************************
 * basic operation to access crimson
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
hideep_pgm_r_reg(struct i2c_client *client, u32 addr, u32 *val)
{
    int ret = 0;
    u32 packet[3];
    u8* bulk = (u8*)packet + 3;

    packet[0] = htonl(0x00);
    packet[1] = htonl(addr);

    ret = i2c_master_send(client, bulk,              5);             //write address
    if (ret < 0)
    {
        goto err;
    }
    mdelay(1);
    ret = i2c_master_recv(client, (u8*)&(packet[2]), 4);
    if (ret < 0)
    {
        goto err;
    }

    *val = ntohl(packet[2]);

err:
    return ret;

}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
hideep_pgm_w_reg(struct i2c_client *client, u32 addr, u32 data)
{
    int ret = 0;
    u32 packet[3];
    u8* bulk = (u8*)packet + 3;

    packet[0] = htonl(0x80);
    packet[1] = htonl(addr);
    packet[2] = htonl(data);

    //i2c_master_send
    ret = i2c_master_send(client, bulk+0, 5);
    if (ret < 0)
    {
        goto err;
    }

    ret = i2c_master_send(client, bulk+5, 4);
    if (ret < 0)
    {
        goto err;
    }

 err:
    return ret;
}

#ifdef PGM_BURST_WR
/*------------------------------------------------------------------------------
 * burst write
 *-----------------------------------------------------------------------------*/
static int
hideep_pgm_w_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
    int ret   = 0;
    int i;

    if(len % 4 != 0)
        return -1;

    //---------------------------------
    // header
    packet->header.w[0] = htonl( (0x80|(len/4-1)));
    packet->header.w[1] = htonl(addr);

    for(i=0; i<NVM_PAGE_SIZE/sizeof(u32); i++)
        packet->payload[i] = htonl(packet->payload[i]);

    //i2c_master_send
    ret = i2c_master_send(client, (u8*)&packet->header.b[3],(len+5));
    if (ret < 0)
    {
        goto err;
    }

 err:
    return ret;
}

/*------------------------------------------------------------------------------
 * burst read
 *-----------------------------------------------------------------------------*/
static int
hideep_pgm_r_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
    int ret   = 0;
    int i;

    if(len % 4 != 0)
        return -1;

    //---------------------------------
    // header
    packet->header.w[0] = htonl( (0x00|(len/4-1)));
    packet->header.w[1] = htonl(addr);

    ret = i2c_master_send(client, (u8*)&packet->header.b[3], 5);             //write address
    if (ret < 0)
    {
        goto err;
    }
    //i2c_master_send
    ret = i2c_master_recv(client, (u8*)packet->payload,    len);
    if (ret < 0)
    {
        goto err;
    }

    for(i=0; i<NVM_PAGE_SIZE/sizeof(u32); i++)
        packet->payload[i] = htonl(packet->payload[i]);

 err:
    return ret;
}

#endif

/*******************************************************************************
 *
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static void
hideep_sw_reset(struct i2c_client *client, u32 food)
{
    hideep_pgm_w_reg(client, SYSCON_WDT_CNT, food);
    hideep_pgm_w_reg(client, SYSCON_WDT_CON, 0x03);
    hideep_pgm_w_reg(client, SYSCON_WDT_CON, 0x01);

    dbg_fun("sw reset");
    return;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int hideep_nvm_unlock(struct i2c_client *client, unsigned int uc0, 
 unsigned int uc1,  unsigned int uc2,  unsigned int uc3)
{
    int ret = 0;
    u8    trim_data = 0x08;

    trim_data =   read_trim_data(client);

    hideep_pgm_w_reg(client, FLASH_TIM, 3);
    hideep_pgm_w_reg(client, FLASH_CFG, NVM_SFR_WPAGE);
    NVM_W_SFR(0,  uc0 | ((trim_data & 0x0f)<<12));
    NVM_W_SFR(4,  uc1);
    NVM_W_SFR(8,  uc2);
    NVM_W_SFR(12, uc3);
    SET_FLASH_HWCONTROL;
    hideep_pgm_w_reg(client, FLASH_CFG, NVM_DEFAULT_PAGE);
    return ret;
}
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
hideep_enter_pgm(struct i2c_client *client)
{
    s32 ret = 0;
    u32 status;
    u32 pattern = 0xDF9DAF39;

    //--------------------------------
    // pgm pattern
    i2c_master_send(client, (u8*)&pattern, 4);
    mdelay(1);

	// flush invalid Tx load register
	hideep_pgm_w_reg(client, ESI_TX_INVALID, 0x01);

    hideep_pgm_r_reg(client, SYSCON_PGM_ID, &status);

    if( status != htonl(pattern))
    {
        dbg_err("enter_pgm : error(%08x):\n", status);
        return -1;
    }

    //--------------------------------
    hideep_pgm_w_reg(client, SYSCON_WDT_CON, 0x00);
    hideep_pgm_w_reg(client, SYSCON_SPC_CON, 0x00);                    // remap
    hideep_pgm_w_reg(client, SYSCON_CLK_ENA, 0xFF);                    // Clock Enable for YRAM/DEMOD/ACU/MSP/CM0
    hideep_pgm_w_reg(client, SYSCON_CLK_CON, 0x01);                    // Select MOSC
    hideep_pgm_w_reg(client, SYSCON_PWR_CON, 0x01);                    //
    hideep_pgm_w_reg(client, FLASH_TIM,      0x03);
    hideep_pgm_w_reg(client, FLASH_CACHE_CFG,0x00);                    // cache disable
    hideep_pgm_w_reg(client, FLASH_CACHE_CFG,0x02);                    // cache flush..


    mdelay(1);

    return ret;
}


/*******************************************************************************
 *
 *******************************************************************************/

/*-----------------------------------------------------------------------------
 *  Program Page in Flash Memory
 *-----------------------------------------------------------------------------*/
static s32
hideep_program_page( struct i2c_client *client, u16 addr, struct pgm_packet *packet_w)
{
    u32  pio_cmd = PROG;
    u32  status;
#ifndef PGM_BURST_WR
    s32   i;
#endif

    hideep_pgm_r_reg(client, FLASH_STA, &status);                  // flash status
    if(status == 0)
        return -1;

    addr = addr & ~(NVM_PAGE_SIZE-1);                   // address mask

    //-------------------------------------------
    // fusing begin
    SET_FLASH_PIO(0);
    SET_FLASH_PIO(1);
    #if 0
    if((addr>0xc000)&&(addr<0xc400))
    {
        pio_cmd |= INF;
	    addr -= 0xc000;
    }
		#endif
    SET_PIO_SIG(pio_cmd+addr, htonl(packet_w->payload[0]));
#ifdef PGM_BURST_WR
    hideep_pgm_w_mem(client, (FLASH_BASE + 0x400000) + pio_cmd, packet_w, NVM_PAGE_SIZE);
#else
    for (i = 0; i < NVM_PAGE_SIZE/4; i++)
    {
        SET_PIO_SIG(pio_cmd + (i<<2), packet_w->payload[i]);
    }
#endif
    #if 0
    if (pio_cmd & INF)
        SET_PIO_SIG(124 | INF,          htonl(packet_w->payload[31]));
    else
    #endif
        SET_PIO_SIG(124,          htonl(packet_w->payload[31]));

    SET_FLASH_PIO(0);

    mdelay(1);                                         //optimize.......
    while(1)
    {
        hideep_pgm_r_reg(client, FLASH_STA, &status);
        if( (status) != 0)
            break;
    }

    hideep_pgm_w_reg(client, FLASH_CON, 0);
    // fusing end..
    //-------------------------------------------


    return 0;
}

/*******************************************************************************
 * chip specific fuse
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static struct pgm_packet packet_w;
static struct pgm_packet packet_r;

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
unsigned int check_chipid(struct i2c_client *client)
{
    unsigned int status;
    unsigned int ID = SYSCON_CHIP_ID;
    hideep_pgm_r_reg(client, ID, &status);

    return status;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/

int read_trim_data(struct i2c_client *client)
{
    unsigned int tmp;
	
	dbg_fun();
//    if (!(IS_FLASH_READY))
//        return -1;

    hideep_pgm_w_reg(client, 0x51000008, NVM_SFR_RPAGE);

    hideep_pgm_r_reg(client, 0x00000000, (unsigned int *)&tmp);
    tmp = ( tmp & 0x0000F000 ) >> 12;

    hideep_pgm_w_reg(client, 0x51000008, NVM_DEFAULT_PAGE);

    return tmp;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
#define CHIPID_MASK_CRIMSON     	0x43524900
#define CHIPID_MASK_LIME        	0x4C494D00
#define NVMSFR_CRIMSON_MASK0       	0x27270698
#define NVMSFR_CRIMSON_MASK1       	0x0E5203FF
#define NVMSFR_CRIMSON_MASK2       	0xFC623800
#define NVMSFR_CRIMSON_MASK3       	0x00310000
#define NVMSFR_LIME_MASK0          	0x28170EA0
#define NVMSFR_LIME_MASK1          	0x0A0E03FF
#define NVMSFR_LIME_MASK2          	0x8C203D0C
#define NVMSFR_LIME_MASK3          	0x0030027B

static int
hideep_program_nvm(struct i2c_client *client, const u8 *ucode, size_t len, int offset)
{
    size_t i;
    s32 ret;
    u32 pages ;
    u32 addr;
    s32 retry = 3;
    s32 len_r;
    s32 len_w;
	unsigned int chipid;

    //------------------------------------------
    // enter pgm
    while(retry--)
    {
        dbg_fun("enter_pgm : %d\n", retry);
        ret = hideep_enter_pgm(client);
        if(ret >= 0)
            break;

        hideep_sw_reset(client, 10);
    }
    if(retry < 0)
    {
        dbg_err("enter_pgm : failed");
        return -1;
    }
    //------------------------------------------
    // checking id & nvm unlock
    chipid = check_chipid(client);
	if((chipid & CHIPID_MASK_CRIMSON) == CHIPID_MASK_CRIMSON)
	{
		hideep_nvm_unlock(client,NVMSFR_CRIMSON_MASK0, NVMSFR_CRIMSON_MASK1, NVMSFR_CRIMSON_MASK2, NVMSFR_CRIMSON_MASK3);
	}
	else if((chipid & CHIPID_MASK_LIME) == CHIPID_MASK_LIME)
	{
		hideep_nvm_unlock(client,NVMSFR_LIME_MASK0, NVMSFR_LIME_MASK1, NVMSFR_LIME_MASK2, NVMSFR_LIME_MASK3);
	}
	else
	{
		dbg_err( "nvm_unlock : failed.");
		return -1;
	}
    //------------------------------------------
    // fusing nvm
    pages      = (len + NVM_PAGE_SIZE - 1)/ NVM_PAGE_SIZE;
    addr       = offset;
    len_r      = len;
    len_w      = len_r;

    for(i=0; i<pages; i++)
    {
        if(len_r >= NVM_PAGE_SIZE)
            len_w = NVM_PAGE_SIZE;

        memcpy(packet_w.payload, &(ucode[addr]), len_w);

        ret = hideep_program_page(client, i * NVM_PAGE_SIZE+offset, &packet_w);
        if(ret < 0)
        {
            dbg_err("hideep_program_nvm : error(%08x):\n", addr);
        }
        addr   += NVM_PAGE_SIZE;
        len_r  -= NVM_PAGE_SIZE;
        len_w   = len_r        ;

    }

    return ret;
}

/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
static int
hideep_verify_nvm(struct i2c_client *client, const u8 *ucode, size_t len, int offset)
{
    s32 i;
    s32 j;
    s32 ret = 0;
    u32 addr  = offset;
    u32 pages = (len + NVM_PAGE_SIZE - 1)/ NVM_PAGE_SIZE;
    s32 len_r = len;
    s32 len_v = len_r;

    for(i=0; i<pages; i++)
    {
        if(len_r >= NVM_PAGE_SIZE)
            len_v = NVM_PAGE_SIZE;

#ifdef PGM_BURST_WR
        hideep_pgm_r_mem(client, 0x00000000 + addr, &packet_r ,NVM_PAGE_SIZE);
#else
        for (j = 0; j < NVM_PAGE_SIZE/4; j++)
        {
            hideep_pgm_r_reg(client, addr+(j<<2), &(packet_r.payload[j]));
        }
#endif
        ret  = memcmp(&(ucode[addr]), packet_r.payload, len_v        );
        if(ret != 0)
        {
            u8 *read = (u8*) packet_r.payload;

            for(j=0; j<NVM_PAGE_SIZE; j++)
                dbg_fun("%02x : %02x", ucode[addr+j], read[j]);

            dbg_err("verify : error(addr : %d)", addr);

            ret = -1;
        }
        addr   += NVM_PAGE_SIZE;
        len_r  -= NVM_PAGE_SIZE;
        len_v  = len_r        ;
    }

  return ret;
}

/*******************************************************************************
 * fusing flash
 *******************************************************************************/
/*------------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------------*/
int
hideep_fuse_ucode(struct i2c_client *client, const u8 *code, size_t len, int offset)
{
    s32 ret;
    s32 retry = 3;

    //-------------------------------------------
	private_ts->pdata->int_enable(private_ts, 0);
	private_ts->pdata->reset();
	msleep(20);
    //-------------------------------------------
    // chip specific code for flash fuse
    ret = hideep_program_nvm(client, code, len, offset);
    if(ret != 0)
        goto out;

#ifdef PGM_VERIFY
    while(retry--)
    {
        ret = hideep_verify_nvm (client, code, len, offset);
        if(ret == 0)
            break;

        dbg_fun(" download uc failed(%d)", retry);
    }

    if(retry != 0)
    {
        dbg_fun("download uc success");
    }

#endif


    //--------------------------------
    // reset..
    hideep_sw_reset(client, 1000);                                 // enable wdr reset

    //-------------------------------------------
out:
	private_ts->pdata->int_enable(private_ts, 1);
    return ret;
}



/*------------------------------------------------------------------------------
 * flash programming sequence
 *-----------------------------------------------------------------------------*/
int
hideep_load_ucode(struct device *dev, const char *fn)
{
    struct hideep_data *ts_drv = private_ts;
    const struct firmware *fw_entry;
    s32 ret;

    //-------------------------------------------
    // load binary from user space
    ret = request_firmware(&fw_entry, fn, dev);
    if(ret != 0)
    {
        dev_err(dev, "request_firmware : fail(%d)\n", ret);
        return ret;
    }
	if(private_ts->suspend)
		return ret;
    //-------------------------------------------
    // chip specific code for flash fuse
    mutex_lock(&private_ts->io_lock); // Mutex lock - front of function

    ret = hideep_fuse_ucode(ts_drv->client, fw_entry->data, fw_entry->size,0);       //reboot ic

    mutex_unlock(&private_ts->io_lock); // Mutex lock - front of function
    //-------------------------------------------
    release_firmware(fw_entry);
    return ret;
}
int hideep_update(struct device *dev, const char *fn)
{
	return hideep_load_ucode(dev,fn);
}


int check_version(struct i2c_client *client, u8 *val)
{
    int ret = 0;
	
    ret = hideep_load_dwz(private_ts,(u16*)val);
    return ret;
}


void get_fw_version(unsigned char *pres, unsigned short *pver)
{
	pver[0] = (pres[HIDEEP_BOOTLOADER_MAJOR_VERSION]<<8) | pres[HIDEEP_BOOTLOADER_MINOR_VERSION];
	pver[1] = (pres[HIDEEP_CODE_MAJOR_VERSION]<<8)       | pres[HIDEEP_CODE_MINOR_VERSION];
	pver[2] = (pres[HIDEEP_CUSTOM_MAJOR_VERSION]<<8)     | pres[HIDEEP_CUSTOM_MINOR_VERSION];
	pver[3] = (pres[HIDEEP_VR_MAJOR_VERSION]<<8)         | pres[HIDEEP_VR_MINOR_VERSION];
	pver[4] = pres[HIDEEP_FACTORY_ID];
}


int hideep_check_which_part_be_updated(unsigned char *pres, int *index)
{
	unsigned short cur_ver[5];
	unsigned short res_ver[5];
	int ret;
	int update_index;

	//check which part is need to be updated.
	private_ts->pdata->int_enable(private_ts, 0);
	private_ts->pdata->reset();
	msleep(20);
	ret = check_version(private_ts->client,(unsigned char *)cur_ver);
	private_ts->pdata->int_enable(private_ts, 1);
	if (ret < 0){
		dbg_err("i2c error, ret = 0x%08x", ret);
		goto error;;
	}
	get_fw_version(pres, res_ver); 	
	for(update_index = HIDEEP_CHECK_VER_FROM_VR; update_index < 4; update_index++){
        dbg_log("%d [0=bl; 1=code; 2=cus; 3=vr] cur=0x%x, res=0x%x", update_index,cur_ver[update_index], res_ver[update_index]);
	  	if(cur_ver[update_index]<res_ver[update_index]){	      		
	      		break;
	  	}
	}
	if(update_index >= 4){
  		dbg_err("no need updated.");
	}
	*index = update_index;
	return ret;
error:
	return ret;

}

void hideep_update_fw_thread(struct work_struct *work)
{
	int ret;
#ifdef HIDEEP_CHECK_INT_AFTER_REBOOT
	int i;
#endif
	int update_index;
    unsigned char *fw_buf;
    unsigned int fw_length;
#ifdef HIDEEP_AUTO_UPDATE_FROM_BIN
	const struct firmware *fw_entry;
#endif

    private_ts->wait_int=1;
#if 1
    #ifdef HIDEEP_AUTO_UPDATE_FROM_BIN
	    #ifdef	HIDEEP_TP_VENDOR
    dbg_fun();
    if(private_ts->pdata->tp_vendor()){
    	dbg_fun();
		ret = request_firmware(&fw_entry, HIDEEP_FW, &private_ts->client->dev);
	}
	else{
    	dbg_fun();
		ret = request_firmware(&fw_entry, HIDEEP_MT_FW, &private_ts->client->dev);
	}
    	#else
    dbg_fun();
	ret = request_firmware(&fw_entry, HIDEEP_FW, &private_ts->client->dev);
		#endif
	if(ret != 0){
	    dbg_err("request_firmware : fail(%d)\n", ret);
	    goto exit;
	}
    fw_buf = (unsigned char *)fw_entry->data;
    fw_length = (unsigned int)fw_entry->size;
    #else
    	#ifdef	HIDEEP_TP_VENDOR
    if(private_ts->pdata->tp_vendor()){
    	dbg_fun();
    	fw_buf = (unsigned char *)HiDeep_FW;
    	fw_length = (unsigned int)HiDeep_FW_nLength;
    }
    else{
    	dbg_fun();
    	fw_buf = (unsigned char *)HiDeep_MT_FW;
    	fw_length = (unsigned int)HiDeep_MT_FW_nLength;
    }
    	#else
    fw_buf = (unsigned char *)HiDeep_FW;
    fw_length = (unsigned int)HiDeep_FW_nLength;
    	#endif
    #endif
    if(fw_length<0x300){
        dbg_err("fw is not correct len = %d !", fw_length);
        goto exit;
    }
	if(private_ts->wait_int==0){
		dbg_err("CANNOT get the interrupt signal");
	    ret = hideep_fuse_ucode(private_ts->client,fw_buf,fw_length,0);
		//ret = hideep_update_function(fw_buf,fw_length, 0);
	}
	else{
		ret = hideep_check_which_part_be_updated(fw_buf, &update_index);
		if(ret<0)
			goto exit;
		if(update_index>=4)
			goto exit;
	    ret = hideep_fuse_ucode(private_ts->client,fw_buf,fw_length,0);
	  	//ret = hideep_update_function(fw_buf,fw_length, update_index);
	}
    #ifdef HIDEEP_AUTO_UPDATE_FROM_BIN
	release_firmware(fw_entry);
    #endif
    private_ts->updating = 0;
    return;
exit:
#endif
    private_ts->updating = 0;
	return;
}


int hideep_load_dwz(struct hideep_data *ts, u16* buf)
{
    s32 ret = 0;
    int retry = 4;
    //------------------------------------------
    // enter pgm
    while(retry--)
    {
        dbg_fun("enter_pgm : %d", retry);
        ret = hideep_enter_pgm(ts->client);
        if(ret >= 0)
            break;
        //do not reset after entering pgm mode.
        //ist_sw_reset(client, 10);
    }
    if(retry <= 0)
    {
        dbg_fun("dwz enter_pgm : failed");
        return -1;
    }

    mdelay(50);

    ret = hideep_pgm_r_mem(ts->client, DWZ_ADDR, &packet_r, sizeof(DWZ_INFO_T));
    if(ret < 0){
        dbg_err("i2c failed");
        goto i2c_err;
	}
    memcpy((u8*)&ts->dwz_info, packet_r.payload, sizeof(DWZ_INFO_T));
    hideep_sw_reset(ts->client, 10);
    //---------------------------------
    printk("firmware boot version   : %04x\n", ts->dwz_info.ver_b);
    printk("firmware core version   : %04x\n", ts->dwz_info.ver_c);
    printk("firmware custom version : %04x\n", ts->dwz_info.ver_d);
    printk("firmware vr version     : %04x\n", ts->dwz_info.ver_v);
    *buf++=ts->dwz_info.ver_b;
    *buf++=ts->dwz_info.ver_c;
    *buf++=ts->dwz_info.ver_d;
    *buf++=ts->dwz_info.ver_v;
    mdelay(50);

    return 0;
i2c_err:
    return -1;
}

