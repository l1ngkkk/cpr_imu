/**
  ******************************************************************************
  * @file    imu_process.h
  * @brief   LSM6DSV 鍏酱 IMU 椹卞姩澶存枃浠讹紙I2C 鎺ュ彛 / STM32 HAL锛?  * @note    鍣ㄤ欢锛歋T LSM6DSV锛? 杞村姞閫熷害 + 3 杞撮檧铻猴紝甯?SFLP/FSM锛?  *          鎬荤嚎锛欼2C1锛孭B6 = SCL锛孭B7 = SDA锛?00 kHz
  *
  * ---------------------------------------------------------------------------
  *  涓€銆佺‖浠惰繛鎺ワ紙Mode 1锛孖2C 浠庢満锛?  * ---------------------------------------------------------------------------
  *      LSM6DSV 寮曡剼            STM32F103
  *      ------------------------------------------------
  *      VDD   (8)   -------->   3.3 V   锛?00 nF + 1 uF 鍘昏€︼級
  *      VDDIO (5)   -------->   3.3 V   锛?00 nF 鍘昏€︼級
  *      GND   (6,7) -------->   GND
  *      SCL   (13)  -------->   PB6     锛?.7k 涓婃媺鍒?3.3 V锛?  *      SDA   (14)  -------->   PB7     锛?.7k 涓婃媺鍒?3.3 V锛?  *      CS    (12)  -------->   3.3 V   锛圛2C 妯″紡蹇呴』涓洪珮锛侊級
  *      SDO/SA0 (1) -------->   3.3 V   锛堟湰鏉垮湴鍧€ 0x6B锛涙帴 GND 鍒欐槸 0x6A锛?  *      SDx   (2)   -------->   GND     锛圡ode 1 涓嶈兘鎮┖锛?  *      SCx   (3)   -------->   GND
  *      INT1  (4)   ------->   鍙€夛紝鎺?MCU 澶栭儴涓柇
  *      OCS_Aux(10)銆丼DO_Aux(11) -> 鎮┖
  *
  * ---------------------------------------------------------------------------
  *  浜屻€侀噺绋嬩笌鐏垫晱搴︼紙鎹㈢畻鐢紝鏉ヨ嚜鏁版嵁鎵嬪唽 Table 3锛?  * ---------------------------------------------------------------------------
  *      鍔犻€熷害锛毬?g  -> 0.061 mg/LSB    闄€铻猴細卤125  dps -> 4.375 mdps/LSB
  *              卤4g  -> 0.122 mg/LSB           卤250  dps -> 8.75  mdps/LSB
  *              卤8g  -> 0.244 mg/LSB           卤500  dps -> 17.5  mdps/LSB
  *              卤16g -> 0.488 mg/LSB           卤1000 dps -> 35    mdps/LSB
  *                                             卤2000 dps -> 70    mdps/LSB
  *                                             卤4000 dps -> 140   mdps/LSB
  *
  * ---------------------------------------------------------------------------
  *  涓夈€佸揩閫熶娇鐢紙璇﹁ imu_process.c 椤堕儴璇存槑锛?  * ---------------------------------------------------------------------------
  *      IMU_Config_t cfg;
  *      IMU_Data_t   data;
  *
  *      IMU_DefaultConfig(&cfg);        // 鍏堝彇榛樿閰嶇疆
  *      cfg.acc_odr = IMU_ODR_120;      // 鍐嶆寜闇€淇敼
  *      cfg.gyr_odr = IMU_ODR_120;
  *      cfg.acc_fs  = IMU_ACC_FS_4G;
  *      cfg.gyr_fs  = IMU_GYR_FS_500;
  *
  *      if (IMU_Init(&cfg) != IMU_OK) { Error_Handler(); }
  *      IMU_CalibrateGyro(200);         // 淇濇寔闈欐锛屾牎鍑嗛檧铻洪浂鍋?  *
  *      while (1) {
  *          if (IMU_Read(&data) == IMU_OK) {
  *              // data.acc_mg.x / data.gyr_dps.x / data.temp_c
  *          }
  *          HAL_Delay(10);
  *      }
  ******************************************************************************
  */

#ifndef __IMU_PROCESS_H
#define __IMU_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*==============================================================================
 *                              涓€銆佺‖浠剁浉鍏冲畯
 *============================================================================*/

/* I2C 鍙ユ焺锛氬鏋滀互鍚庢崲鐢?I2C2锛屽彧鏀硅繖涓€澶勫嵆鍙?*/
#define IMU_I2C_HANDLE          hi2c1

/* 7 浣嶄粠鏈哄湴鍧€宸︾Щ 1 浣嶏紝HAL 闇€瑕?8 浣嶆牸寮忋€? * SDO/SA0 鎺?GND   -> 7 浣嶅湴鍧€ 0x6A
 * SDO/SA0 鎺?VDDIO -> 7 浣嶅湴鍧€ 0x6B   <-- 鏈澘 SA0 鎺?VCC锛屾墍浠ョ敤 0x6B
 * 鎵嬪唽绗?20 椤电粰鐨?0xD4/0xD5 鏄妸璇?鍐欎綅涔熺畻杩涘幓鐨?8 浣嶅啓娉曪紝HAL 閲屼笉瑕佺洿鎺ョ敤銆?*/
#define IMU_I2C_ADDR            (0x6B << 1)

/* I2C 瓒呮椂锛坢s锛変笌澶辫触閲嶈瘯娆℃暟 */
#define IMU_I2C_TIMEOUT         100
#define IMU_I2C_RETRY           3

/* WHO_AM_I 鍥哄畾鍊硷紝鐢ㄦ潵纭閫氫俊姝ｅ父銆佸瀷鍙锋纭?*/
#define IMU_WHO_AM_I_VALUE      0x70

/*==============================================================================
 *                           浜屻€佸瘎瀛樺櫒鍦板潃锛堟暟鎹墜鍐岀 49 椤碉級
 *============================================================================*/

#define IMU_REG_FUNC_CFG_ACCESS 0x01    /* 宓屽叆寮忓姛鑳?浼犳劅鍣?Hub 椤靛垏鎹?*/
#define IMU_REG_PIN_CTRL        0x02    /* 寮曡剼涓婃媺鎺у埗 */
#define IMU_REG_IF_CFG          0x03    /* 鎺ュ彛閰嶇疆锛堜腑鏂瀬鎬с€?/4 绾跨瓑锛?*/
#define IMU_REG_FIFO_CTRL1      0x07    /* FIFO 姘翠綅 */
#define IMU_REG_FIFO_CTRL2      0x08    /* FIFO 鍘嬬缉绛?*/
#define IMU_REG_FIFO_CTRL3      0x09    /* FIFO 鎵瑰鐞嗛€熺巼 BDR */
#define IMU_REG_FIFO_CTRL4      0x0A    /* FIFO 妯″紡閫夋嫨 */
#define IMU_REG_INT1_CTRL       0x0D    /* INT1 寮曡剼涓柇浣胯兘 */
#define IMU_REG_INT2_CTRL       0x0E    /* INT2 寮曡剼涓柇浣胯兘 */
#define IMU_REG_WHO_AM_I        0x0F    /* 鍣ㄤ欢 ID */
#define IMU_REG_CTRL1           0x10    /* 鍔犻€熷害锛氬姛鑰楁ā寮?+ ODR */
#define IMU_REG_CTRL2           0x11    /* 闄€铻猴細鍔熻€楁ā寮?+ ODR */
#define IMU_REG_CTRL3           0x12    /* BDU / IF_INC / SW_RESET */
#define IMU_REG_CTRL4           0x13    /* 鏁版嵁灏辩华銆両NT2 璺敱绛?*/
#define IMU_REG_CTRL5           0x14    /* I3C 鐩稿叧 */
#define IMU_REG_CTRL6           0x15    /* 闄€铻猴細LPF1 甯﹀ + 閲忕▼ FS_G */
#define IMU_REG_CTRL7           0x16    /* 淇濈暀/鍐呴儴 */
#define IMU_REG_CTRL8           0x17    /* 鍔犻€熷害锛歀PF2 甯﹀ + 鍙岄€氶亾 + 閲忕▼ FS_XL */
#define IMU_REG_CTRL9           0x18    /* 鍔犻€熷害楂橀€?鏂滅巼婊ゆ尝 */
#define IMU_REG_CTRL10          0x19    /* 鑷 */
#define IMU_REG_STATUS          0x1E    /* 鏁版嵁灏辩华鐘舵€?*/
#define IMU_REG_OUT_TEMP_L      0x20    /* 娓╁害浣庡瓧鑺?*/
#define IMU_REG_OUT_TEMP_H      0x21    /* 娓╁害楂樺瓧鑺?*/
#define IMU_REG_OUTX_L_G        0x22    /* 闄€铻?X 浣庡瓧鑺傦紙0x22~0x27 涓?GX/GY/GZ锛?*/
#define IMU_REG_OUTX_L_A        0x28    /* 鍔犻€熷害 X 浣庡瓧鑺傦紙0x28~0x2D 涓?AX/AY/AZ锛?*/
#define IMU_REG_X_OFS_USR       0x73    /* 鐢ㄦ埛闆跺亸琛ュ伩 X锛?x73~0x75锛?*/
#define IMU_REG_FIFO_DATA_OUT   0x78    /* FIFO 璇诲嚭锛? 瀛楄妭 TAG + 6 瀛楄妭鏁版嵁 */

/*------------------------------------------------------------------------------
 * 甯哥敤瀵勫瓨鍣ㄤ綅瀹氫箟
 *----------------------------------------------------------------------------*/

/* CTRL3 (0x12) */
#define IMU_CTRL3_BOOT          0x80    /* 閲嶆柊鍔犺浇鍑哄巶鏍″噯鍊?*/
#define IMU_CTRL3_BDU           0x40    /* 1 = 楂樹綆瀛楄妭璇婚綈涔嬪墠涓嶆洿鏂拌緭鍑哄瘎瀛樺櫒 */
#define IMU_CTRL3_IF_INC        0x04    /* 1 = 澶氬瓧鑺傝闂椂鍦板潃鑷姩閫掑 */
#define IMU_CTRL3_SW_RESET      0x01    /* 1 = 杞浣嶏紙鑷姩娓呴浂锛?*/

/* CTRL4 (0x13) */
#define IMU_CTRL4_INT2_ON_INT1  0x10    /* 鎶?INT2 鐨勪俊鍙蜂篃閫佸埌 INT1 */
#define IMU_CTRL4_DRDY_MASK     0x08    /* 婊ゆ尝鍣ㄧǔ瀹氬墠灞忚斀 DRDY */
#define IMU_CTRL4_DRDY_PULSED   0x02    /* 1 = DRDY 杈撳嚭 65us 鑴夊啿 */

/* STATUS (0x1E) */
#define IMU_STATUS_XLDA         0x01    /* 鍔犻€熷害鏂版暟鎹氨缁?*/
#define IMU_STATUS_GDA          0x02    /* 闄€铻烘柊鏁版嵁灏辩华 */
#define IMU_STATUS_TDA          0x04    /* 娓╁害鏂版暟鎹氨缁?*/
#define IMU_STATUS_GDA_EIS      0x10    /* EIS 闄€铻烘柊鏁版嵁 */
#define IMU_STATUS_OIS_DRDY     0x20    /* OIS 鏁版嵁灏辩华 */

/* INT1_CTRL (0x0D) */
#define IMU_INT1_DRDY_XL        0x01
#define IMU_INT1_DRDY_G         0x02
#define IMU_INT1_FIFO_TH        0x08
#define IMU_INT1_FIFO_OVR       0x10
#define IMU_INT1_FIFO_FULL      0x20
#define IMU_INT1_CNT_BDR        0x40

/*==============================================================================
 *                              涓夈€佽繑鍥炵姸鎬? *============================================================================*/

typedef enum
{
    IMU_OK           =  0,   /* 鎴愬姛 */
    IMU_ERR_I2C      = -1,   /* I2C 璇诲啓澶辫触锛堟棤搴旂瓟銆佹€荤嚎蹇欍€佽秴鏃讹級 */
    IMU_ERR_ID       = -2,   /* WHO_AM_I 涓嶅尮閰嶏紙鎺ョ嚎/鍦板潃/CS 鏈夐棶棰橈級 */
    IMU_ERR_PARAM    = -3,   /* 鍏ュ弬闈炴硶 */
    IMU_ERR_NOT_INIT = -4,   /* 杩樻病璋冪敤 IMU_Init */
    IMU_ERR_TIMEOUT  = -5    /* 绛夊緟鏁版嵁灏辩华瓒呮椂 */
} IMU_Status_t;

/*==============================================================================
 *                        鍥涖€侀厤缃敤鐨勬灇涓撅紙瀵瑰簲瀵勫瓨鍣ㄧ紪鐮侊級
 *============================================================================*/

/* 杈撳嚭鏁版嵁鐜?ODR銆? * 娉ㄦ剰锛氬姞閫熷害缂栫爜 1 = 1.875 Hz锛屼絾鍙湪浣庡姛鑰楁ā寮忎笅鏈夋晥锛? *       闄€铻烘病鏈?1.875 Hz 杩欎竴妗ｏ紝鍐?1 鏄繚鐣欏€笺€?*/
typedef enum
{
    IMU_ODR_OFF    = 0,     /* 鎺夌數 */
    IMU_ODR_1H875  = 1,     /* 1.875 Hz锛堜粎鍔犻€熷害 LPM锛?*/
    IMU_ODR_7H5    = 2,     /* 7.5 Hz */
    IMU_ODR_15     = 3,     /* 15 Hz */
    IMU_ODR_30     = 4,     /* 30 Hz */
    IMU_ODR_60     = 5,     /* 60 Hz */
    IMU_ODR_120    = 6,     /* 120 Hz */
    IMU_ODR_240    = 7,     /* 240 Hz */
    IMU_ODR_480    = 8,     /* 480 Hz */
    IMU_ODR_960    = 9,     /* 960 Hz */
    IMU_ODR_1920   = 10,    /* 1.92 kHz */
    IMU_ODR_3840   = 11,    /* 3.84 kHz */
    IMU_ODR_7680   = 12     /* 7.68 kHz */
} IMU_Odr_t;

/* 鍔犻€熷害閲忕▼锛堝啓杩?CTRL8 鐨?FS_XL[1:0]锛?*/
typedef enum
{
    IMU_ACC_FS_2G  = 0,     /* 卤2 g  */
    IMU_ACC_FS_4G  = 1,     /* 卤4 g  */
    IMU_ACC_FS_8G  = 2,     /* 卤8 g  */
    IMU_ACC_FS_16G = 3      /* 卤16 g */
} IMU_AccFs_t;

/* 闄€铻洪噺绋嬶紙鍐欒繘 CTRL6 鐨?FS_G[3:0]锛屾敞鎰?卤4000 dps 鐨勭紪鐮佹槸 1100 涓嶆槸 0101锛?*/
typedef enum
{
    IMU_GYR_FS_125  = 0,    /* 卤125  dps */
    IMU_GYR_FS_250  = 1,    /* 卤250  dps */
    IMU_GYR_FS_500  = 2,    /* 卤500  dps */
    IMU_GYR_FS_1000 = 3,    /* 卤1000 dps */
    IMU_GYR_FS_2000 = 4,    /* 卤2000 dps */
    IMU_GYR_FS_4000 = 5     /* 卤4000 dps锛堥渶鍏抽棴 OIS 閾撅級 */
} IMU_GyrFs_t;

/* 鍔犻€熷害鍔熻€楁ā寮忥紙鍐欒繘 CTRL1 鐨?OP_MODE_XL[2:0]锛?*/
typedef enum
{
    IMU_ACC_MODE_HP     = 0,    /* 楂樻€ц兘锛屽叏 ODR 鍙敤 */
    IMU_ACC_MODE_HAODR  = 1,    /* 楂樼簿搴?ODR */
    IMU_ACC_MODE_LPM1   = 4,    /* 浣庡姛鑰?1锛? 娆″钩鍧囷級 */
    IMU_ACC_MODE_LPM2   = 5,    /* 浣庡姛鑰?2锛? 娆″钩鍧囷級 */
    IMU_ACC_MODE_LPM3   = 6,    /* 浣庡姛鑰?3锛? 娆″钩鍧囷級 */
    IMU_ACC_MODE_NORMAL = 7     /* 鏅€氭ā寮忥紙7.5 Hz~1.92 kHz锛?*/
} IMU_AccMode_t;

/* 闄€铻哄姛鑰楁ā寮忥紙鍐欒繘 CTRL2 鐨?OP_MODE_G[2:0]锛?*/
typedef enum
{
    IMU_GYR_MODE_HP    = 0,     /* 楂樻€ц兘 */
    IMU_GYR_MODE_HAODR = 1,     /* 楂樼簿搴?ODR */
    IMU_GYR_MODE_SLEEP = 4,     /* 鐫＄湢 */
    IMU_GYR_MODE_LP    = 5      /* 浣庡姛鑰楋紙7.5~240 Hz锛?*/
} IMU_GyrMode_t;

/*==============================================================================
 *                              浜斻€佹暟鎹粨鏋? *============================================================================*/

/* 涓夎酱鍘熷鍊硷紙鏈夌鍙?16 浣嶈ˉ鐮侊級 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} IMU_Raw3_t;

/* 涓夎酱鐗╃悊閲?*/
typedef struct
{
    float x;
    float y;
    float z;
} IMU_Vec3f_t;

/* 涓€娆¤鍙栧緱鍒扮殑瀹屾暣鏁版嵁 */
typedef struct
{
    IMU_Raw3_t  acc_raw;    /* 鍔犻€熷害鍘熷 LSB */
    IMU_Raw3_t  gyr_raw;    /* 闄€铻哄師濮?LSB */
    IMU_Vec3f_t acc_mg;     /* 鍔犻€熷害鐗╃悊閲忥紝鍗曚綅 mg锛堝彧鍋氶噺绋嬫崲绠楋紝鏈墸闆跺亸锛?*/
    IMU_Vec3f_t gyr_dps;    /* 闄€铻虹墿鐞嗛噺锛屽崟浣?dps锛堝凡鎵ｆ帀 IMU_CalibrateGyro 寰楀埌鐨勯浂鍋忥級 */
    float       temp_c;     /* 鑺墖娓╁害锛屽崟浣?鈩?*/
} IMU_Data_t;

/* 鍒濆鍖栭厤缃?*/
typedef struct
{
    uint8_t acc_odr;        /* IMU_Odr_t锛屽姞閫熷害 ODR */
    uint8_t gyr_odr;        /* IMU_Odr_t锛岄檧铻?ODR */
    uint8_t acc_fs;         /* IMU_AccFs_t锛屽姞閫熷害閲忕▼ */
    uint8_t gyr_fs;         /* IMU_GyrFs_t锛岄檧铻洪噺绋?*/
    uint8_t acc_mode;       /* IMU_AccMode_t锛屽姞閫熷害鍔熻€楁ā寮?*/
    uint8_t gyr_mode;       /* IMU_GyrMode_t锛岄檧铻哄姛鑰楁ā寮?*/
} IMU_Config_t;

/*==============================================================================
 *                              鍏€佸澶栨帴鍙? *============================================================================*/

/**
 * @brief  鐢ㄩ粯璁ゅ弬鏁板～鍏呴厤缃粨鏋勪綋锛?20 Hz / 卤4 g / 卤500 dps / 楂樻€ц兘锛? * @param  cfg  寰呭～鍏呯殑缁撴瀯浣撴寚閽? * @retval 鏃? */
void IMU_DefaultConfig(IMU_Config_t *cfg);

/**
 * @brief  鍒濆鍖?LSM6DSV
 * @param  cfg  閰嶇疆鍙傛暟锛屼笉鑳戒负 NULL
 * @retval IMU_OK 鎴愬姛锛涘叾浣欒 IMU_Status_t
 * @note   鍐呴儴娴佺▼锛氱瓑涓婄數 -> 璇?WHO_AM_I -> 杞浣?-> 閰?BDU/IF_INC
 *         -> 閰嶉噺绋?ODR -> 绛夐檧铻虹ǔ瀹?30 ms
 */
IMU_Status_t IMU_Init(const IMU_Config_t *cfg);

/**
 * @brief  璇?WHO_AM_I
 * @param  id  杈撳嚭锛屾甯稿簲涓?0x70
 */
IMU_Status_t IMU_WhoAmI(uint8_t *id);

/**
 * @brief  鎺㈡祴鏌愪釜 7 浣嶅湴鍧€涓婃湁娌℃湁鍣ㄤ欢搴旂瓟锛圛2C 鎵弿鐢級
 * @param  addr7  7 浣嶅湴鍧€锛屼緥濡?0x6A銆?x6B
 * @retval IMU_OK 鏈夊簲绛旓紱IMU_ERR_I2C 鏃犲簲绛? * @note   鍙仛鍦板潃鎺㈡祴锛屼笉璇诲啓瀵勫瓨鍣紝鐢ㄦ潵蹇€熷垽鏂€滆姱鐗囧湪涓嶅湪鎬荤嚎涓娾€? */
IMU_Status_t IMU_ProbeAddr(uint8_t addr7);

/**
 * @brief  杞浣嶏紝鎵€鏈夊瘎瀛樺櫒鎭㈠榛樿鍊硷紙涔嬪悗闇€瑕侀噸鏂?IMU_Init锛? */
IMU_Status_t IMU_SoftReset(void);

/**
 * @brief  璁╁姞閫熷害鍜岄檧铻洪兘杩涘叆鎺夌數妯″紡锛堢數娴佺害 2.6 uA锛? */
IMU_Status_t IMU_PowerDown(void);

/**
 * @brief  璇诲彇鍘熷鏁版嵁锛堜竴娆¤繛缁 14 瀛楄妭锛屼繚璇佸悓涓€閲囨牱鏃跺埢锛? * @param  acc       鍔犻€熷害鍘熷鍊硷紝鍙紶 NULL
 * @param  gyr       闄€铻哄師濮嬪€硷紝鍙紶 NULL
 * @param  temp_raw  娓╁害鍘熷鍊硷紝鍙紶 NULL
 */
IMU_Status_t IMU_ReadRaw(IMU_Raw3_t *acc, IMU_Raw3_t *gyr, int16_t *temp_raw);

/**
 * @brief  璇诲彇骞舵崲绠楁垚鐗╃悊閲忥紙mg / dps / 鈩冿級
 * @param  out  杈撳嚭缁撴瀯浣? * @note   闄€铻轰細鑷姩鍑忓幓 IMU_CalibrateGyro() 寰楀埌鐨勯浂鍋? */
IMU_Status_t IMU_Read(IMU_Data_t *out);

/**
 * @brief  杞绛夊緟鍔犻€熷害鍜岄檧铻洪兘浜х敓鏂版暟鎹? * @param  timeout_ms  瓒呮椂鏃堕棿锛坢s锛? */
IMU_Status_t IMU_WaitDataReady(uint32_t timeout_ms);

/**
 * @brief  闄€铻洪浂鍋忔牎鍑嗭細鑺墖闈欐鏃惰皟鐢紝鍙?samples 娆″钩鍧囦綔涓洪浂鍋? * @param  samples  閲囨牱娆℃暟锛? 琛ㄧず浣跨敤榛樿 200 娆★紙绾?1 绉掞級
 * @note   鏍″噯鏈熼棿蹇呴』淇濇寔闈欐锛屽惁鍒欓浂鍋忎細琚繍鍔ㄦ薄鏌? */
IMU_Status_t IMU_CalibrateGyro(uint16_t samples);

/**
 * @brief  鍙栧綋鍓嶉檧铻洪浂鍋? * @param  bx/by/bz  杈撳嚭锛屽崟浣?dps
 */
void IMU_GetGyroBias(float *bx, float *by, float *bz);

/**
 * @brief  鎶婂姞閫熷害/闄€铻虹殑鏁版嵁灏辩华淇″彿璺敱鍒?INT1 寮曡剼
 * @param  en  0 = 鍏抽棴锛岄潪 0 = 鎵撳紑
 */
IMU_Status_t IMU_EnableDrdyInt1(uint8_t en);

/**
 * @brief  璇荤姸鎬佸瘎瀛樺櫒
 * @param  status  杈撳嚭锛屼綅瀹氫箟瑙?IMU_STATUS_xxx
 */
IMU_Status_t IMU_ReadStatus(uint8_t *status);

/**
 * @brief  鍐欎竴涓瘎瀛樺櫒锛堣皟璇曠敤锛? */
IMU_Status_t IMU_WriteReg(uint8_t reg, uint8_t val);

/**
 * @brief  璇讳竴涓瘎瀛樺櫒锛堣皟璇曠敤锛? */
IMU_Status_t IMU_ReadReg(uint8_t reg, uint8_t *val);

/**
 * @brief  褰撳墠閰嶇疆涓嬪姞閫熷害鐨勭伒鏁忓害
 * @retval mg/LSB锛涙湭鍒濆鍖栬繑鍥?0
 */
float IMU_GetAccLsbMg(void);

/**
 * @brief  褰撳墠閰嶇疆涓嬮檧铻虹殑鐏垫晱搴? * @retval dps/LSB锛涙湭鍒濆鍖栬繑鍥?0
 */
float IMU_GetGyrLsbDps(void);

/*==============================================================================
 *                涓冦€丆PR 蹇冭偤澶嶈嫃鎸夊帇鐩戞祴锛堜笂涓嬩綅绉?/ 娣卞害 / 棰戠巼锛? *============================================================================*/

/*
 * 鍘熺悊绠€杩帮紙涓轰粈涔堜笉鍋氫簩娆＄Н鍒嗭級锛? *   1. 鍔犻€熷害璁℃祴鍒扮殑鏄?閲嶅姏 + 杩愬姩鍔犻€熷害"銆傜敤鎱㈤€熶綆閫氫及璁￠噸鍔涙柟鍚戯紝
 *      鎶婁笁杞村姞閫熷害鎶曞奖鍒伴噸鍔涙柟鍚戝苟鍑忓幓 1 g锛屽緱鍒扮珫鐩磋繍鍔ㄥ姞閫熷害銆? *   2. 鍋?0.8~20 Hz 甯﹂€氾紝婊ゆ帀鎵嬬殑鎱㈢Щ銆佸櫔澹板拰娈嬩綑闆跺亸銆? *   3. 鎸夊帇鏄噯鍛ㄦ湡杩愬姩锛宎 = d''锛屾墍浠ヤ竴涓懆鏈熷唴锛? *          d_pp = a_pp / 蠅虏 ,  蠅 = 2蟺/T
 *      鍙娴嬪嚭"鍔犻€熷害宄板嘲鍊?鍜?鍛ㄦ湡"锛屽氨鑳界洿鎺ョ畻浣嶇Щ锛屼笉闇€瑕佺Н鍒嗐€? *      绉垎鍣ㄧ殑婕傜Щ銆佽浆鍔ㄣ€佸啿鍑诲湪杩欓噷鍙奖鍝嶅綋鍓嶅懆鏈燂紝涓嬩竴鍛ㄦ湡鑷姩鎭㈠銆? *   4. 鐢?涓婂崌杩囬浂 + 杩熸粸"鍒掑垎鍛ㄦ湡锛岃繜婊?30 mg 闃叉鍣０鍙嶅瑙﹀彂銆? *
 * 浣跨敤闄愬埗锛? *   - 閲囨牱鐜囧繀椤讳笌 IMU ODR 涓€鑷达紙榛樿 120 Hz锛夛紝骞跺湪 CPR_Init 閲屼紶杩涘幓锛? *   - 璁惧瑕?璐?鍦ㄦ寜鍘嬩綅缃笂锛岄殢鑳稿粨涓€璧蜂笂涓嬭繍鍔紱
 *   - 娣卞害鍜屾寜鍘嬫尝褰㈠舰鐘舵湁鍏筹紝鍑哄巶鍓嶇敤鏍囧噯娣卞害锛? cm 鏍囧昂/澶嶈嫃妯″瀷锛? *     鏍囧畾涓€娆★細瀹為檯娣卞害 / 鏄剧ず娣卞害 -> CPR_SetDepthGain()銆? */

/* CPR 绠楁硶閰嶇疆銆備紶 NULL 缁?CPR_Init 琛ㄧず鍏ㄩ儴鐢ㄩ粯璁ゅ€?*/
typedef struct
{
    float fs;             /* 閲囨牱鐜?Hz锛屽繀椤讳笌 IMU ODR 涓€鑷达紝榛樿 120 */
    float hp_hz;          /* 甯﹂€氶珮閫氭埅姝紙涓ょ骇绾ц仈=4闃讹級锛岄粯璁?1.0 Hz */
    float lp_hz;          /* 甯﹂€氫綆閫氭埅姝紝榛樿 8 Hz */
    float cross_hyst_mg;  /* 杩囬浂杩熸粸锛宮g锛岄粯璁?30 */
    float warmup_s;       /* 婊ゆ尝鍣ㄧǔ瀹氭椂闂达紝榛樿 2 s锛岃繖娈垫椂闂翠笉鍑虹粨鏋?*/
    float min_depth_mm;   /* 鏈夋晥娣卞害涓嬮檺锛岄粯璁?15 mm */
    float max_depth_mm;   /* 鏈夋晥娣卞害涓婇檺锛岄粯璁?90 mm */
    float gravity_tau_s;  /* 閲嶅姏浼拌鏃堕棿甯告暟锛岄粯璁?1.0 s */
} CPR_Config_t;

/* CPR 杈撳嚭缁撴灉 */
typedef struct
{
    float    disp_mm;      /* 漏积分得到的竖直位移 mm（新方案，可画波形） */
    float    vel_mm_s;     /* 漏积分得到的竖直速度 mm/s（过零即位移极值） */
    float    depth_mm;     /* 旧方案：单频正交(dft1)的逐次深度 mm */
    float    depth_avg_mm; /* 旧方案：平滑后的深度 mm */
    float    depth2_mm;    /* 新方案：漏积分+峰谷差，本次按压行程 mm */
    float    depth2_avg_mm;/* 新方案：最近 2 次平均 mm */
    float    rate_cpm;     /* 按压频率 次/分 */
    uint32_t count;        /* 旧方案累计有效按压次数 */
    uint32_t count2;       /* 新方案累计有效按压次数 */
    uint32_t t_ms;         /* 本次按压发生的时刻（HAL_GetTick, ms） */
    uint32_t interval_ms;  /* 距上一次按压的间隔 ms（第一次为 0） */
    uint8_t  valid;        /* 最近一次深度是否在有效范围内 */
    uint8_t  ready;        /* 1 = 已过预热期，输出可信 */
    uint8_t  ev2;          /* 1 = 本次调用刚产生一次新方案结果（EVL 用） */
    /* --- 以下为调试量，正常使用可忽略 --- */
    float    acc_v_mg;     /* 滤后的竖直加速度 mg */
    float    acc_mag_mg;   /* 三轴合加速度 mg（正常约 1000） */
    float    gyr_mag_dps;  /* 三轴合角速度 dps */
    uint8_t  stat;         /* 本帧状态（保留） */
    uint16_t cross_n;      /* 上升过零次数（调试） */
    uint16_t rej_period;   /* 周期超范围丢弃次数（调试） */
    uint16_t rej_depth;    /* 深度超范围丢弃次数（调试） */
    uint16_t rej_gyro;     /* 转动过大丢弃次数（调试） */
} CPR_Result_t;

/**
 * @brief  鍒濆鍖?CPR 绠楁硶
 * @param  cfg  閰嶇疆锛屼紶 NULL 浣跨敤榛樿锛?20 Hz锛? * @note   蹇呴』鍦?IMU_Init 涔嬪悗銆佸紑濮嬭鏁颁箣鍓嶈皟鐢ㄤ竴娆? */
void CPR_Init(const CPR_Config_t *cfg);

/**
 * @brief  娓呯┖绠楁硶鐘舵€侊紙閲嶆柊寮€濮嬭鏁帮級
 */
void CPR_Reset(void);

/**
 * @brief  璁剧疆娣卞害鏍囧畾澧炵泭
 * @param  gain  瀹為檯娣卞害 / 娴嬮噺娣卞害锛岄粯璁?1.0
 */
void CPR_SetDepthGain(float gain);

/**
 * @brief  閫佸叆涓€甯?IMU 鏁版嵁锛屾洿鏂版寜鍘嬬洃娴? * @param  imu  IMU_Read() 寰楀埌鐨勬暣甯ф暟鎹紙鍙敤鍔犻€熷害锛? * @param  dt   璺濅笂涓€甯х殑鏃堕棿锛岀锛堝缓璁敤 HAL_GetTick 瀹炴祴锛? * @param  out  杈撳嚭缁撴灉锛屼笉鑳戒负 NULL
 * @retval 1 = 鏈瀹屾垚涓€娆℃寜鍘嬶紙娣卞害/棰戠巼宸叉洿鏂帮級锛? = 鏅€氭暟鎹抚
 * @note   姣忔潵涓€甯ф柊鏁版嵁璋冪敤涓€娆★紝涓嶈閲嶅鍠傚悓涓€甯? */
float   CPR_GetPeriodAvg(void);

uint8_t CPR_Update(const IMU_Data_t *imu, float dt, CPR_Result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_PROCESS_H */
