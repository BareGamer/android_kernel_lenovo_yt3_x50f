

#ifndef __HIDEEP__
#define __HIDEEP__

#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>

#include <linux/module.h>

#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h> // slot
#include <linux/input/hideep_blueberry_ts.h> // slot
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/gfp.h>
//#include "tpd.h"
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
//#include <plat/gpio-cfg.h>
#include <mach/gpio.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <uapi/linux/i2c.h>

#include "hideep_fw.h"

#define DD_VERSION_MAJOR  0
#define DD_VERSION_MIDDLE 1
#define DD_VERSION_MINOR  1

#define TOUCH_COUNT_ADDR            0x0240
#define TOUCH_READ_START_ADDR       0x0242 //Fixed value
#define TOUCH_FLAG_MASK             0x10
#define TOUCH_DEEP_SLEEP_MASK       0x10
#define MAX_TRANSACTION_LENGTH      16
#define MAX_I2C_DMA_LENGTH          252
#define I2C_DMA_BUF_SIZE						(4*1024)
#define HIDEEP_RX_COUNT 58
#define HIDEEP_TX_COUNT 35


#define VR_ADDR_IMAGE               0x1000
#define VR_ADDR_BASE                0x0000
#define DWZ_ADDR                    0x02c0
#define __HIDEEP_MAJOR_VERSION__ 1
#define __HIDEEP_MINOR_VERSION__ 0
#define __HIDEEP_UPDATE_BOOTLOADER__ 1
#define __HIDEEP_VERIFY_METHOD__ 1
#define __HIDEEP_FLASH_VERIFY_TOGETHER__  0

#define TOUCH_READ_SW_VER_ADDR     0x0132 //Model Dependent

#define TOUCH_READ_BL_VER_ADDR     0X8014
#define TOUCH_READ_CODE_VER_ADDR     0X8016
#define TOUCH_READ_CUST_VER_ADDR     0X8018
#define TOUCH_READ_VR_VER_ADDR     0X801A
#define TOUCH_READ_FACTORY_ID_ADDR	0X8022


#define OPM_RAW_FLAG (0x80)
#define OPM_TOUCH_A (0x00)
#define OPM_MOD (0x80)
#define OPM_MOD_CAP (0x81)
#define OPM_MOD_PRS (0x82)
#define OPM_FRAME_PRS (0x84)
#define OPM_DEMOD (0x85)
#define OPM_DEMOD_CAP (0x86)
#define OPM_DEMOD_PRS (0x87)
#define OPM_FRAME (0x88)
#define OPM_FRAME_CAP (0x89)
#define OPM_DIFF (0x8A)
#define OPM_DIFF_CAP (0x8B)
#define OPM_DIFF_PRS (0x8C)
#define OPM_BASELINE_CAP (0x8D)
#define OPM_AULU_DIFF (0x8E)
#define OPM_AULU_DATA (0x8F)
#define OPM_STATUS (0x90)
#define OPM_LASTDIFF (0x91)
#define OPM_PARM0 (0x92)
#define OPM_PARM1 (0x93)
#define OPM_PARM2 (0x94)
#define OPM_PARM3 (0x95)

extern struct hideep_data *private_ts;
extern int g_loglevel;

#ifdef HIDEEP_IF_DEVICE
int hideep_iface_init    (struct hideep_data *ts);
void hideep_iface_uninit (struct hideep_data *ts);
s32 hideep_get_image(struct hideep_data *ts);
#endif
int hideep_load_dwz(struct hideep_data *ts, u16* buf);
int hideep_sysfs_init   (struct hideep_data *ts);
int hideep_sysfs_exit   (struct hideep_data *ts);
void hideep_reset(void);
#ifdef CONFIG_HIDEEP_UPDATE_FW_I2C
int hideep_fuse_ucode(struct i2c_client *client, const u8 *code, size_t len, int offset);
#endif
void procfs_create(void);
int hideep_get_rawdata(struct rawdata_info *info);
void hideep_update_fw_thread(struct work_struct *work);
int check_version(struct i2c_client *client, u8 *val);
int hideep_update(struct device *dev, const char *buf);
int hideep_i2c_read(struct i2c_client *client, uint16_t addr, uint16_t len, uint8_t *buf);
int hideep_i2c_write(struct i2c_client *client, uint16_t addr, uint16_t len, uint8_t *buf);
#ifdef HIDEEP_SELF_TEST
void hideep_test_mode(struct hideep_data *ts);
#endif
#ifdef	HIDEEP_TP_VENDOR
int hideep_tp_vendor(void);
#endif

#endif //__HIDEEP__
