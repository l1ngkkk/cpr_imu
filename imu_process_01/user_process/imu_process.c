/**
  ******************************************************************************
  * @file    imu_process.c
  * @brief   LSM6DSV 鍏酱 IMU 椹卞姩瀹炵幇锛圛2C / STM32 HAL锛?  * @note    鏈枃浠跺彧渚濊禆锛?  *            - STM32 HAL 鐨?I2C 闃诲寮忔帴鍙ｏ紙HAL_I2C_Mem_Read / Mem_Write锛?  *            - HAL_Delay / HAL_GetTick
  *          涓嶄緷璧栦换浣曠涓夋柟搴擄紝鍙互鐩存帴鏀捐繘 CubeMX 宸ョ▼銆?  *
  * ---------------------------------------------------------------------------
  *  涓€銆佷笂鐢靛埌鍑烘暟鎹殑瀹屾暣娴佺▼锛堜篃鏄?IMU_Init 鍐呴儴鍋氱殑浜嬶級
  * ---------------------------------------------------------------------------
  *   1. 涓婄數鍚庣瓑 10 ms锛岃鍐呴儴 LDO/鎸崱鍣ㄧǔ瀹氾紱
  *   2. 璇?WHO_AM_I(0x0F)锛屽簲涓?0x70銆備笉瀵瑰氨璇存槑 CS 娌℃帴楂樸€佸湴鍧€閿欎簡鎴栨病涓婃媺锛?  *   3. 鍐?CTRL3(0x12) = 0x01 鍋氳蒋澶嶄綅锛岀瓑 15 ms锛?  *   4. 鍐?CTRL3 = 0x44锛?  *        bit6 BDU    = 1  淇濊瘉 16 浣嶆暟鎹珮/浣庡瓧鑺傛潵鑷悓涓€娆￠噰鏍?  *        bit2 IF_INC = 1  澶氬瓧鑺傝繛缁鏃跺湴鍧€鑷姩鍔?1
  *   5. 鍏堝啓閲忕▼锛欳TRL6(0x15) 鐨?FS_G[3:0]銆丆TRL8(0x17) 鐨?FS_XL[1:0]锛?  *   6. 鍐嶅啓 ODR 鍚姩锛欳TRL1(0x10) 鐨?OP_MODE_XL[6:4]+ODR_XL[3:0]銆?  *      CTRL2(0x11) 鐨?OP_MODE_G[6:4]+ODR_G[3:0]锛?  *   7. 绛?30 ms锛堥檧铻哄缓绔嬫椂闂达級锛屼箣鍚庢暟鎹墠鏈夋晥銆?  *
  *  浜屻€佷负浠€涔堜竴娆¤ 14 涓瓧鑺?  *     0x20~0x2D 渚濇鏄細娓╁害(2) + 闄€铻?X/Y/Z(6) + 鍔犻€熷害 X/Y/Z(6)銆?  *     涓€娆?burst 璇诲畬锛屾棦蹇張鑳戒繚璇侀檧铻哄拰鍔犻€熷害鏄悓涓€鏃跺埢鐨勯噰鏍凤紝
  *     鍋氬Э鎬佽В绠楁椂涓嶄細鍥犱负涓ゆ璇讳箣闂寸殑鏃堕棿宸紩鍏ヨ宸€?  *
  *  涓夈€佹暟鎹崲绠?  *     鍔犻€熷害(mg)  = 鍘熷鍊?脳 鐏垫晱搴?mg/LSB)
  *     闄€铻?dps)   = 鍘熷鍊?脳 鐏垫晱搴?mdps/LSB) / 1000 鈭?闆跺亸(dps)
  *     娓╁害(鈩?     = 25 + 鍘熷鍊?/ 256      锛?5鈩?鏃惰緭鍑?0锛?56 LSB/鈩冿級
  *
  *  鍥涖€佸叧浜庨浂鍋?  *     闄€铻哄嚭鍘傚彧鏍″噯鐏垫晱搴︼紝涓嶆牎鍑嗛浂鍋忋€傞潤姝㈡椂浠嶆湁鍑?dps 鐨勮緭鍑猴紝
  *     绉垎鎴愯搴︿細涓€鐩存紓銆侷MU_CalibrateGyro() 鍦ㄩ潤姝㈡椂鍙?N 娆″钩鍧囷紝
  *     鎶婄粨鏋滃瓨璧锋潵锛屼箣鍚?IMU_Read() 鑷姩鍑忔帀銆?  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "imu_process.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>

/*==============================================================================
 *                            鍐呴儴甯搁噺涓庣姸鎬? *============================================================================*/

/* 鍔犻€熷害鐏垫晱搴﹁〃锛屽崟浣?mg/LSB锛屼笅鏍?= FS 妗ｄ綅锛?:卤2g 1:卤4g 2:卤8g 3:卤16g锛?*/
static const float s_acc_sens_mg[4] =
{
    0.061f,     /* 卤2 g  */
    0.122f,     /* 卤4 g  */
    0.244f,     /* 卤8 g  */
    0.488f      /* 卤16 g */
};

/* 闄€铻虹伒鏁忓害琛紝鍗曚綅 mdps/LSB锛屼笅鏍?= FS 妗ｄ綅锛?:卤125 ... 5:卤4000锛?*/
static const float s_gyr_sens_mdps[6] =
{
    4.375f,     /* 卤125  dps */
    8.75f,      /* 卤250  dps */
    17.5f,      /* 卤500  dps */
    35.0f,      /* 卤1000 dps */
    70.0f,      /* 卤2000 dps */
    140.0f      /* 卤4000 dps */
};

/* 闄€铻洪噺绋嬪啓杩?CTRL6 鐨勫疄闄呯紪鐮併€? * 娉ㄦ剰 卤4000 dps 鐨勭紪鐮佹槸 1100(0x0C)锛屼笉鏄『搴忕殑 0101锛屾墍浠ュ繀椤绘煡琛ㄣ€?*/
static const uint8_t s_gyr_fs_code[6] =
{
    0x00,       /* 卤125  dps */
    0x01,       /* 卤250  dps */
    0x02,       /* 卤500  dps */
    0x03,       /* 卤1000 dps */
    0x04,       /* 卤2000 dps */
    0x0C        /* 卤4000 dps */
};

/* 褰撳墠閰嶇疆锛屾崲绠楁椂瑕佺敤 */
static IMU_Config_t s_cfg;

/* 鍒濆鍖栨爣蹇楋細0 = 杩樻病閰嶇疆锛? = 宸查厤缃彲璇绘暟鎹?*/
static uint8_t s_inited = 0U;

/* 闄€铻洪浂鍋忥紝鍗曚綅 dps锛岀敱 IMU_CalibrateGyro() 璁＄畻 */
static float s_gyro_bias[3] = { 0.0f, 0.0f, 0.0f };

/*==============================================================================
 *                          鍐呴儴鍑芥暟锛欼2C 璇诲啓
 *============================================================================*/

/**
 * @brief  鍐欏崟涓瘎瀛樺櫒
 * @note   澶辫触浼氳嚜鍔ㄩ噸璇?IMU_I2C_RETRY 娆★紝瑕嗙洊鎬荤嚎鍋跺彂 BUSY 鐨勬儏鍐? */
static IMU_Status_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t i;

    for (i = 0U; i < IMU_I2C_RETRY; i++)
    {
        /* HAL_I2C_Mem_Write 鍐呴儴浼氳嚜鍔ㄤ骇鐢?START + 璁惧鍦板潃 + 瀵勫瓨鍣ㄥ湴鍧€ + 鏁版嵁 + STOP */
        if (HAL_I2C_Mem_Write(&IMU_I2C_HANDLE,
                              IMU_I2C_ADDR,
                              (uint16_t)reg,
                              I2C_MEMADD_SIZE_8BIT,
                              &val,
                              1U,
                              IMU_I2C_TIMEOUT) == HAL_OK)
        {
            return IMU_OK;
        }
    }

    return IMU_ERR_I2C;
}

/**
 * @brief  杩炵画璇诲涓瘎瀛樺櫒
 * @param  reg  璧峰瀵勫瓨鍣ㄥ湴鍧€
 * @param  buf  鎺ユ敹缂撳啿鍖? * @param  len  璇诲彇闀垮害
 * @note   渚濊禆 CTRL3.IF_INC = 1锛屽惁鍒欏湴鍧€涓嶄細鑷锛岃鍥炴潵鍏ㄦ槸鍚屼竴涓瓧鑺? */
static IMU_Status_t imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t i;

    for (i = 0U; i < IMU_I2C_RETRY; i++)
    {
        /* HAL_I2C_Mem_Read 浼氫骇鐢燂細START+鍐欏湴鍧€+瀵勫瓨鍣ㄥ湴鍧€+閲嶅START+璇诲湴鍧€+鏁版嵁+NACK+STOP */
        if (HAL_I2C_Mem_Read(&IMU_I2C_HANDLE,
                             IMU_I2C_ADDR,
                             (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf,
                             len,
                             IMU_I2C_TIMEOUT) == HAL_OK)
        {
            return IMU_OK;
        }
    }

    return IMU_ERR_I2C;
}

/* 鎶婅繑鍥炲€兼鏌ュ啓鎴愬畯锛岄伩鍏嶆瘡姝ラ兘鍐?if (... != IMU_OK) return ...; */
#define IMU_CHECK(expr)                                          \
    do {                                                         \
        IMU_Status_t _imu_st = (expr);                           \
        if (_imu_st != IMU_OK) { return _imu_st; }               \
    } while (0)

/*==============================================================================
 *                            瀵瑰鎺ュ彛锛氶厤缃? *============================================================================*/

/**
 * @brief  榛樿閰嶇疆锛?20 Hz銆伮? g銆伮?00 dps銆佸姞閫熷害/闄€铻洪兘鐢ㄩ珮鎬ц兘妯″紡
 * @note   120 Hz 瀵瑰Э鎬佽В绠楄冻澶燂紝卤4 g/卤500 dps 鏃㈣兘瑕嗙洊鎸ユ墜鍔ㄤ綔鍙堜笉浼氬お绮? */
void IMU_DefaultConfig(IMU_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    cfg->acc_odr  = (uint8_t)IMU_ODR_120;
    cfg->gyr_odr  = (uint8_t)IMU_ODR_120;
    cfg->acc_fs   = (uint8_t)IMU_ACC_FS_4G;
    cfg->gyr_fs   = (uint8_t)IMU_GYR_FS_500;
    cfg->acc_mode = (uint8_t)IMU_ACC_MODE_HP;
    cfg->gyr_mode = (uint8_t)IMU_GYR_MODE_HP;
}

/**
 * @brief  鍒濆鍖?LSM6DSV
 */
IMU_Status_t IMU_Init(const IMU_Config_t *cfg)
{
    uint8_t id = 0U;
    uint8_t reg;

    /* ---------------- 1. 鍙傛暟妫€鏌?---------------- */
    if (cfg == 0)
    {
        return IMU_ERR_PARAM;
    }
    if ((cfg->acc_fs > 3U) || (cfg->gyr_fs > 5U))
    {
        return IMU_ERR_PARAM;
    }
    if ((cfg->acc_odr > 12U) || (cfg->gyr_odr > 12U))
    {
        return IMU_ERR_PARAM;
    }
    if (cfg->gyr_odr == 1U)
    {
        /* 0x01 鏄檧铻虹殑淇濈暀缂栫爜锛屾病鏈?1.875 Hz 杩欎竴妗?*/
        return IMU_ERR_PARAM;
    }

    /* ---------------- 2. 绛変笂鐢电ǔ瀹氾紝骞跺浣嶄竴娆?I2C 澶栬 ----------------
     * STM32F1 鐨?HAL I2C 涓婄數鍚庡伓灏斾細甯︾潃 BUSY 鏍囧織杩涘叆绗竴娆′紶杈擄紝
     * 琛ㄧ幇涓?绗竴娆¤鍏ㄥけ璐ワ紝绱ф帴鐫€鍐嶈灏辨甯?銆?     * 杩欓噷鍏?DeInit/Init 涓€娆★紝鎶婂璁剧姸鎬佹満娓呭共鍑€鍐嶉€氫俊銆?*/
    HAL_Delay(20);

    HAL_I2C_DeInit(&IMU_I2C_HANDLE);
    HAL_I2C_Init(&IMU_I2C_HANDLE);

    /* ---------------- 3. 纭鑺墖鍦ㄧ嚎锛堟渶澶氶噸璇?5 娆★級 ---------------- */
    {
        uint8_t  i;
        IMU_Status_t st = IMU_ERR_I2C;

        for (i = 0U; i < 5U; i++)
        {
            st = IMU_WhoAmI(&id);
            if ((st == IMU_OK) && (id == IMU_WHO_AM_I_VALUE))
            {
                break;
            }
            HAL_Delay(10);
        }

        if (st != IMU_OK)
        {
            return st;                  /* I2C 瀹屽叏涓嶉€?*/
        }
    }

    if (id != IMU_WHO_AM_I_VALUE)
    {
        /* 璇诲緱鍒颁絾鍊间笉瀵癸細鍨嬪彿涓嶅鎴栧湴鍧€鍐茬獊
           瀹屽叏璇讳笉鍒帮細CS 娌℃帴楂樸€佹病涓婃媺銆佸湴鍧€閿欍€佽姱鐗囨病渚涚數 */
        return IMU_ERR_ID;
    }

    /* ---------------- 4. 杞浣嶏紝鍥炲埌宸茬煡鐘舵€?---------------- */
    IMU_CHECK(IMU_SoftReset());
    HAL_Delay(15);

    /* ---------------- 5. BDU + IF_INC ----------------
     * BDU=1    : 璇诲畬楂樺瓧鑺備箣鍓嶈緭鍑哄瘎瀛樺櫒涓嶆洿鏂帮紝闃叉璇诲埌涓€鍗婅鏂伴噰鏍疯鐩?     * IF_INC=1 : 澶氬瓧鑺傝繛缁鏃跺湴鍧€鑷锛?x20~0x2D 涓€娆¤瀹岀殑鍓嶆彁锛?*/
    IMU_CHECK(imu_write_reg(IMU_REG_CTRL3, IMU_CTRL3_BDU | IMU_CTRL3_IF_INC));

    /* ---------------- 6. 鍏堥厤閲忕▼锛屽啀閰?ODR ----------------
     * 椤哄簭寰堥噸瑕侊細鍏堝畾閲忕▼鍐嶅惎鍔紝閬垮厤鍒氬惎鍔ㄦ椂鐢ㄩ敊璇殑閲忕▼閲囧埌鑴忔暟鎹?*/

    /* CTRL6: [6:4]=LPF1_G_BW锛堜繚鎸?000 榛樿甯﹀锛夛紝[3:0]=FS_G */
    reg = (uint8_t)(s_gyr_fs_code[cfg->gyr_fs] & 0x0FU);
    IMU_CHECK(imu_write_reg(IMU_REG_CTRL6, reg));

    /* CTRL8: [5:3]=HP_LPF2_XL_BW锛堜繚鎸?000 榛樿锛夛紝[2]=XL_DualC_EN锛? 鍏抽棴鍙岄€氶亾锛夛紝
     *        [1:0]=FS_XL */
    reg = (uint8_t)(cfg->acc_fs & 0x03U);
    IMU_CHECK(imu_write_reg(IMU_REG_CTRL8, reg));

    /* CTRL1: [6:4]=OP_MODE_XL锛孾3:0]=ODR_XL銆俠it7 蹇呴』涓?0 */
    reg = (uint8_t)(((cfg->acc_mode & 0x07U) << 4) | (cfg->acc_odr & 0x0FU));
    IMU_CHECK(imu_write_reg(IMU_REG_CTRL1, reg));

    /* CTRL2: [6:4]=OP_MODE_G锛孾3:0]=ODR_G銆俠it7 蹇呴』涓?0 */
    reg = (uint8_t)(((cfg->gyr_mode & 0x07U) << 4) | (cfg->gyr_odr & 0x0FU));
    IMU_CHECK(imu_write_reg(IMU_REG_CTRL2, reg));

    /* ---------------- 7. 淇濆瓨閰嶇疆锛屾竻鎺夋棫闆跺亸 ---------------- */
    s_cfg = *cfg;
    s_gyro_bias[0] = 0.0f;
    s_gyro_bias[1] = 0.0f;
    s_gyro_bias[2] = 0.0f;
    s_inited = 1U;

    /* ---------------- 8. 绛夐檧铻哄缓绔?----------------
     * 鎵嬪唽锛氶檧铻?turn-on time 鍏稿瀷 30 ms锛岃繖娈垫椂闂寸殑杈撳嚭涓嶅彲淇?*/
    HAL_Delay(30);

    return IMU_OK;
}

/**
 * @brief  璇?WHO_AM_I
 */
IMU_Status_t IMU_WhoAmI(uint8_t *id)
{
    if (id == 0)
    {
        return IMU_ERR_PARAM;
    }

    return imu_read_regs(IMU_REG_WHO_AM_I, id, 1U);
}

/**
 * @brief  鎺㈡祴鏌愪釜 7 浣嶅湴鍧€涓婃湁娌℃湁鍣ㄤ欢搴旂瓟
 * @note   HAL_I2C_IsDeviceReady 鍙彂涓€涓湴鍧€瀛楄妭鐪嬫湁娌℃湁 ACK锛? *         涓嶈鍐欎换浣曞瘎瀛樺櫒锛岄€傚悎鍋氭€荤嚎鎵弿銆? */
IMU_Status_t IMU_ProbeAddr(uint8_t addr7)
{
    if (HAL_I2C_IsDeviceReady(&IMU_I2C_HANDLE,
                              (uint16_t)((uint16_t)addr7 << 1),
                              2U,
                              20U) == HAL_OK)
    {
        return IMU_OK;
    }

    return IMU_ERR_I2C;
}

/**
 * @brief  杞浣? * @note   SW_RESET 鏄嚜娓呴浂浣嶏紱澶嶄綅鍚庢墍鏈夋帶鍒跺瘎瀛樺櫒鍥為粯璁ゅ€硷紝
 *         鎵€浠ヨ皟鐢ㄤ箣鍚庡繀椤婚噸鏂?IMU_Init()銆侾IN_CTRL / IF_CFG 涓嶅彈褰卞搷銆? */
IMU_Status_t IMU_SoftReset(void)
{
    IMU_Status_t st;

    st = imu_write_reg(IMU_REG_CTRL3, IMU_CTRL3_SW_RESET);
    if (st != IMU_OK)
    {
        return st;
    }

    s_inited = 0U;

    return IMU_OK;
}

/**
 * @brief  鍔犻€熷害 + 闄€铻哄叏閮ㄦ帀鐢? * @note   涔嬪悗瑕侀噸鏂?IMU_Init() 鎵嶈兘鍐嶅彇鏁版嵁
 */
IMU_Status_t IMU_PowerDown(void)
{
    IMU_Status_t st;

    /* CTRL1 / CTRL2 鐨?ODR 瀛楁鍐?0000 灏辨槸 power-down */
    st = imu_write_reg(IMU_REG_CTRL1, 0x00U);
    if (st != IMU_OK)
    {
        return st;
    }

    st = imu_write_reg(IMU_REG_CTRL2, 0x00U);
    if (st != IMU_OK)
    {
        return st;
    }

    s_inited = 0U;

    return IMU_OK;
}

/*==============================================================================
 *                            瀵瑰鎺ュ彛锛氳鏁? *============================================================================*/

/**
 * @brief  璇诲師濮嬫暟鎹? * @note   0x20~0x2D 鍏?14 瀛楄妭锛? *            b[0..1]   娓╁害      L,H
 *            b[2..7]   闄€铻?GX,GY,GZ锛堟瘡涓?L,H锛? *            b[8..13]  鍔犻€熷害 AX,AY,AZ锛堟瘡涓?L,H锛? *         鎵€鏈夐噺閮芥槸灏忕锛堜綆瀛楄妭鍦ㄥ墠锛夈€佹湁绗﹀彿 16 浣嶈ˉ鐮併€? */
IMU_Status_t IMU_ReadRaw(IMU_Raw3_t *acc, IMU_Raw3_t *gyr, int16_t *temp_raw)
{
    uint8_t b[14];
    IMU_Status_t st;

    st = imu_read_regs(IMU_REG_OUT_TEMP_L, b, 14U);
    if (st != IMU_OK)
    {
        return st;
    }

    /* 鍏堟嫾鎴?uint16 鍐嶈浆 int16锛岄伩鍏嶇洿鎺ョЩ浣嶆椂绗﹀彿浣嶅嚭闂 */
    if (temp_raw != 0)
    {
        *temp_raw = (int16_t)(((uint16_t)b[1] << 8) | (uint16_t)b[0]);
    }

    if (gyr != 0)
    {
        gyr->x = (int16_t)(((uint16_t)b[3]  << 8) | (uint16_t)b[2]);
        gyr->y = (int16_t)(((uint16_t)b[5]  << 8) | (uint16_t)b[4]);
        gyr->z = (int16_t)(((uint16_t)b[7]  << 8) | (uint16_t)b[6]);
    }

    if (acc != 0)
    {
        acc->x = (int16_t)(((uint16_t)b[9]  << 8) | (uint16_t)b[8]);
        acc->y = (int16_t)(((uint16_t)b[11] << 8) | (uint16_t)b[10]);
        acc->z = (int16_t)(((uint16_t)b[13] << 8) | (uint16_t)b[12]);
    }

    return IMU_OK;
}

/**
 * @brief  璇讳竴娆″畬鏁存暟鎹苟鎹㈢畻鎴愮墿鐞嗛噺
 */
IMU_Status_t IMU_Read(IMU_Data_t *out)
{
    IMU_Raw3_t acc;
    IMU_Raw3_t gyr;
    int16_t t_raw = 0;
    IMU_Status_t st;

    if (out == 0)
    {
        return IMU_ERR_PARAM;
    }
    if (s_inited == 0U)
    {
        return IMU_ERR_NOT_INIT;
    }

    st = IMU_ReadRaw(&acc, &gyr, &t_raw);
    if (st != IMU_OK)
    {
        return st;
    }

    /* 鍘熷鍊煎師鏍峰甫鍥烇紝鏂逛究鑷繁鍋氭护娉㈡垨鐪嬮ケ鍜屾儏鍐?*/
    out->acc_raw = acc;
    out->gyr_raw = gyr;

    /* 鍔犻€熷害锛歮g = LSB 脳 鐏垫晱搴?*/
    out->acc_mg.x = (float)acc.x * s_acc_sens_mg[s_cfg.acc_fs];
    out->acc_mg.y = (float)acc.y * s_acc_sens_mg[s_cfg.acc_fs];
    out->acc_mg.z = (float)acc.z * s_acc_sens_mg[s_cfg.acc_fs];

    /* 闄€铻猴細dps = LSB 脳 鐏垫晱搴?mdps) / 1000 鈭?闆跺亸 */
    out->gyr_dps.x = (float)gyr.x * s_gyr_sens_mdps[s_cfg.gyr_fs] / 1000.0f - s_gyro_bias[0];
    out->gyr_dps.y = (float)gyr.y * s_gyr_sens_mdps[s_cfg.gyr_fs] / 1000.0f - s_gyro_bias[1];
    out->gyr_dps.z = (float)gyr.z * s_gyr_sens_mdps[s_cfg.gyr_fs] / 1000.0f - s_gyro_bias[2];

    /* 娓╁害锛?5鈩?瀵瑰簲 0锛屽垎杈ㄧ巼 1/256 鈩?*/
    out->temp_c = 25.0f + ((float)t_raw / 256.0f);

    return IMU_OK;
}

/**
 * @brief  绛変竴甯ф柊鏁版嵁
 * @note   杞 STATUS_REG 鐨?XLDA/GDA 浣嶏紱閫傚悎 ODR 鈮?200 Hz 鐨勫満鍚堛€? *         鏇撮珮 ODR 寤鸿鐢?FIFO 鎴?INT1 涓柇锛岄伩鍏嶄富寰幆琚疆璇㈡嫋鎱€? */
IMU_Status_t IMU_WaitDataReady(uint32_t timeout_ms)
{
    uint32_t t0;
    uint8_t  status = 0U;
    IMU_Status_t st;

    t0 = HAL_GetTick();

    for (;;)
    {
        st = imu_read_regs(IMU_REG_STATUS, &status, 1U);
        if (st != IMU_OK)
        {
            return st;
        }

        if ((status & (IMU_STATUS_XLDA | IMU_STATUS_GDA)) ==
            (IMU_STATUS_XLDA | IMU_STATUS_GDA))
        {
            return IMU_OK;
        }

        if ((HAL_GetTick() - t0) > timeout_ms)
        {
            return IMU_ERR_TIMEOUT;
        }

        HAL_Delay(1);
    }
}

/**
 * @brief  闄€铻洪浂鍋忔牎鍑? * @note   鍘熺悊锛氶潤姝㈡椂闄€铻虹悊璁鸿緭鍑轰负 0锛屽疄娴嬬殑骞冲潎鍊煎氨鏄浂鍋忋€? *         閲囨牱闂撮殧 5 ms锛?00 娆＄害 1 绉掋€傛牎鍑嗘湡闂翠笉鑳芥檭鍔ㄦ澘瀛愩€? */
IMU_Status_t IMU_CalibrateGyro(uint16_t samples)
{
    int32_t  sum[3];
    IMU_Raw3_t acc;
    IMU_Raw3_t gyr;
    uint16_t i;
    float    lsb_dps;
    IMU_Status_t st;

    if (s_inited == 0U)
    {
        return IMU_ERR_NOT_INIT;
    }

    if (samples == 0U)
    {
        samples = 200U;
    }
    if (samples > 1000U)
    {
        /* 涓婇檺 1000 娆★紝闃叉 int32 绱姞婧㈠嚭 */
        samples = 1000U;
    }

    sum[0] = 0;
    sum[1] = 0;
    sum[2] = 0;

    for (i = 0U; i < samples; i++)
    {
        st = IMU_ReadRaw(&acc, &gyr, 0);
        if (st != IMU_OK)
        {
            return st;
        }

        sum[0] += (int32_t)gyr.x;
        sum[1] += (int32_t)gyr.y;
        sum[2] += (int32_t)gyr.z;

        HAL_Delay(5);
    }

    /* 鎶婂钩鍧?LSB 鎹㈢畻鎴?dps */
    lsb_dps = s_gyr_sens_mdps[s_cfg.gyr_fs] / 1000.0f;

    s_gyro_bias[0] = ((float)sum[0] / (float)samples) * lsb_dps;
    s_gyro_bias[1] = ((float)sum[1] / (float)samples) * lsb_dps;
    s_gyro_bias[2] = ((float)sum[2] / (float)samples) * lsb_dps;

    return IMU_OK;
}

/**
 * @brief  鍙栧綋鍓嶉浂鍋忥紙dps锛? */
void IMU_GetGyroBias(float *bx, float *by, float *bz)
{
    if (bx != 0)
    {
        *bx = s_gyro_bias[0];
    }
    if (by != 0)
    {
        *by = s_gyro_bias[1];
    }
    if (bz != 0)
    {
        *bz = s_gyro_bias[2];
    }
}

/*==============================================================================
 *                            瀵瑰鎺ュ彛锛氳緟鍔? *============================================================================*/

/**
 * @brief  鎶婃暟鎹氨缁俊鍙锋帴鍒?INT1
 * @note   寮曡剼榛樿鎺ㄦ尳銆侀珮鏈夋晥锛圛F_CFG 鐨?H_LACTIVE=0銆丳P_OD=0锛夈€? *         鎺?MCU 鏃剁敤涓婂崌娌胯Е鍙戝嵆鍙€? */
IMU_Status_t IMU_EnableDrdyInt1(uint8_t en)
{
    uint8_t val;

    val = (en != 0U) ? (uint8_t)(IMU_INT1_DRDY_XL | IMU_INT1_DRDY_G) : 0x00U;

    return imu_write_reg(IMU_REG_INT1_CTRL, val);
}

/**
 * @brief  璇荤姸鎬佸瘎瀛樺櫒
 */
IMU_Status_t IMU_ReadStatus(uint8_t *status)
{
    if (status == 0)
    {
        return IMU_ERR_PARAM;
    }

    return imu_read_regs(IMU_REG_STATUS, status, 1U);
}

/**
 * @brief  鍐欏瘎瀛樺櫒锛堣皟璇?鎵╁睍鐢級
 */
IMU_Status_t IMU_WriteReg(uint8_t reg, uint8_t val)
{
    return imu_write_reg(reg, val);
}

/**
 * @brief  璇诲瘎瀛樺櫒锛堣皟璇?鎵╁睍鐢級
 */
IMU_Status_t IMU_ReadReg(uint8_t reg, uint8_t *val)
{
    if (val == 0)
    {
        return IMU_ERR_PARAM;
    }

    return imu_read_regs(reg, val, 1U);
}

/**
 * @brief  褰撳墠鍔犻€熷害鐏垫晱搴︼紝mg/LSB
 */
float IMU_GetAccLsbMg(void)
{
    if (s_inited == 0U)
    {
        return 0.0f;
    }

    return s_acc_sens_mg[s_cfg.acc_fs];
}

/**
 * @brief  褰撳墠闄€铻虹伒鏁忓害锛宒ps/LSB
 */
float IMU_GetGyrLsbDps(void)
{
    if (s_inited == 0U)
    {
        return 0.0f;
    }

    return s_gyr_sens_mdps[s_cfg.gyr_fs] / 1000.0f;
}

/*==============================================================================
 *                涓冦€丆PR 鎸夊帇鐩戞祴锛堜綅绉绘繁搴?/ 棰戠巼 / 娆℃暟锛? *============================================================================*/

/*
 * 娴嬮噺鍘熺悊锛堜负浠€涔堜笉鍐嶅仛浜屾绉垎锛夛細
 *
 *   鎸夊帇鏄噯鍛ㄦ湡鐨勪笂涓嬭繍鍔紝浣嶇Щ d(t) 涓庡姞閫熷害 a(t) 婊¤冻 a = d''銆? *   瀵逛竴涓懆鏈熶负 T 鐨勮繎浼兼寮﹁繍鍔紝鏈夛細
 *
 *       d_pp = a_pp / 蠅虏 ,   蠅 = 2蟺 / T
 *
 *   鍙渶瑕佹祴鍑轰竴涓懆鏈熷唴绔栫洿鍔犻€熷害鐨?RMS 骞呭€?a_rms 鍜屽懆鏈?T锛? *   灏辫兘鐩存帴绠楀嚭浣嶇Щ宄板嘲鍊硷紝瀹屽叏涓嶉渶瑕佺Н鍒嗭細
 *       d_pp = 2鈭? 路 a_rms / 蠅虏
 *   杩欓噷鐢?RMS 鑰屼笉鏄嘲宄板€硷紝鏄洜涓哄嘲宄板€煎娉㈠舰閲岀殑鎶樿/鍐插嚮鏋佸叾鏁忔劅
 *   锛堜竴涓皷宄板氨鑳借缁撴灉缈诲€嶏級锛孯MS 鍙湅鑳介噺锛屾姉鍐插嚮鑳藉姏寮哄緱澶氥€? *
 *   杩欐牱鍋氱殑濂藉锛? *     - 涓嶄細婕傜Щ锛氱Н鍒嗗櫒鏈€鎬曠殑闆跺亸銆佽浆鍔ㄣ€佸啿鍑伙紝鍦ㄨ繖閲屽彧褰卞搷褰撳墠杩欎竴涓? *       鍛ㄦ湡锛屼笅涓€鍛ㄦ湡鑷姩鎭㈠锛? *     - 瀵逛綆棰戞檭鍔ㄤ笉鏁忔劅锛氬嘲宄板€煎彧鐪嬫尝褰㈣捣浼忥紝鐩存祦鍋忕疆涓嶅弬涓庤绠楋紱
 *     - 涓嶉渶瑕?濮挎€佷繚鎶?澶嶄綅"杩欑被瀹规槗璇Е鍙戠殑閫昏緫銆? *
 *   浠ｄ环锛氭繁搴﹀拰娉㈠舰褰㈢姸鏈夊叧锛堜笅鍘嬪揩銆佸洖寮规參鐨勬尝褰紝宄板嘲鍊煎亸澶э級锛? *   鎵€浠ョ粨鏋滆涔樹竴涓爣瀹氬鐩娿€傞粯璁?1.0锛岀敤涓€娆℃爣鍑嗘繁搴︽寜鍘嬫爣瀹氬嵆鍙€? */

/* 浜岄樁 IIR锛圧BJ cookbook锛屽反鐗规矁鏂?Q=0.7071锛?*/
typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
} CPR_Biquad_t;

static CPR_Config_t s_cpr_cfg;          /* 褰撳墠閰嶇疆 */
static CPR_Biquad_t s_cpr_hp1;          /* 楂橀€氱 1 绾э紙涓ょ骇绾ц仈 = 4 闃讹級 */
static CPR_Biquad_t s_cpr_hp2;          /* 楂橀€氱 2 绾?*/
static CPR_Biquad_t s_cpr_lp;           /* 浣庨€?*/

static float   s_cpr_g[3];              /* 閲嶅姏鏂瑰悜浼拌锛堝崟浣嶅悜閲忥級 */
static uint8_t s_cpr_g_ok;              /* 閲嶅姏鏄惁宸插垵濮嬪寲 */

static float   s_cpr_av_sq;             /* 鏈懆鏈?av虏 绱姞 */
static uint32_t s_cpr_n;                /* 鏈懆鏈熼噰鏍风偣鏁?*/
static float   s_cpr_gmax;              /* 鏈懆鏈熸渶澶у悎瑙掗€熷害锛宒ps */
static float   s_cpr_d_hist[3];         /* 鏈€杩?3 娆℃繁搴︼紝鐢ㄤ簬涓€兼护娉?*/
static uint8_t s_cpr_d_idx;             /* 娣卞害鍘嗗彶鍐欐寚閽?*/
static uint8_t s_cpr_d_cnt;             /* 娣卞害鍘嗗彶鏈夋晥涓暟 */

static float   s_cpr_t;                 /* 绱杩愯鏃堕棿 s */
static float   s_cpr_t_cross;           /* 涓婁竴娆′笂鍗囪繃闆舵椂鍒?*/
static uint8_t s_cpr_have_cross;        /* 鏄惁宸茬粡鏈夊弬鑰冭繃闆剁偣 */
static uint8_t s_cpr_positive;          /* 褰撳墠鏄惁鍦ㄦ鍗婂懆 */

static float   s_cpr_rate;              /* 骞虫粦鍚庣殑鎸夊帇棰戠巼 */
static float   s_cpr_depth;             /* 鏈€杩戜竴娆℃繁搴?*/
static float   s_cpr_depth_avg;         /* 骞虫粦鍚庣殑娣卞害锛堟帹鑽愭樉绀鸿繖涓級 */
static float   s_cpr_t_event;           /* 涓婁竴娆℃湁鏁堟寜鍘嬫椂闂?*/
static uint32_t s_cpr_count;            /* 绱鎸夊帇娆℃暟 */

static float   s_cpr_hyst = 60.0f;      /* 杩囬浂杩熸粸锛堣嚜閫傚簲锛夛紝mg */
static float   s_cpr_period_avg = 0.5f;
static float   s_cpr_t_rise = 0.0f;
static float   s_cpr_t_rise_prev = 0.0f; /* 鏈€杩戠殑骞冲潎鎸夊帇鍛ㄦ湡锛宻 */
static uint8_t s_cpr_idle = 1U;         /* 1 = 澶勪簬绌洪棽锛堟棤鎸夊帇锛夌姸鎬?*/
static float   s_cpr_rms_avg;           /* 鏈€杩戞寜鍘嬬殑鍔犻€熷害 RMS 鍧囧€硷紝mg */
static uint16_t s_cpr_cross_n;          /* 妫€娴嬪埌鐨勪笂鍗囪繃闆舵鏁?*/
static uint16_t s_cpr_rej_period;       /* 鍛ㄦ湡瓒呰寖鍥翠涪寮冩鏁?*/
static uint16_t s_cpr_rej_depth;        /* 娣卞害瓒呰寖鍥翠涪寮冩鏁?*/
static uint16_t s_cpr_rej_gyro;         /* 杞姩杩囧ぇ涓㈠純娆℃暟 */

static float   s_cpr_gain = 1.0f;       /* 娣卞害鏍囧畾澧炵泭 */
static uint8_t s_cpr_inited;            /* 鏄惁宸插垵濮嬪寲 */

/* ---------------- 新方案状态（漏积分） ---------------- */
static float    s_lep_v      = 0.0f;   /* 竖直速度 mm/s */
static float    s_lep_d      = 0.0f;   /* 竖直位移 mm */
static float    s_lep_hi     = 0.0f;   /* 本周期位移最大值 */
static float    s_lep_lo     = 0.0f;   /* 本周期位移最小值 */
static float    s_lep_t_last = -1.0f;  /* 上一次出结果的时刻 s */
static float    s_lep_depth  = 0.0f;   /* 最近一次的行程 mm */
static float    s_lep_prev   = 0.0f;   /* 上一次的行程（做 2 次平均） */
static float    s_lep_avg    = 0.0f;   /* 2 次平均后的行程 */
static uint8_t  s_lep_n      = 0U;     /* 已有几次样本 */
static uint32_t s_lep_cnt    = 0U;     /* 累计有效次数 */
static uint8_t  s_lep_ev     = 0U;     /* 本帧刚产生新结果 */


/**
 * @brief  鍒濆鍖栦竴涓簩闃舵护娉㈠櫒
 * @param  f         婊ゆ尝鍣ㄧ姸鎬? * @param  fs        閲囨牱鐜?Hz
 * @param  fc        鎴棰戠巼 Hz
 * @param  highpass  1=楂橀€氾紝0=浣庨€? */
static void cpr_biquad_init(CPR_Biquad_t *f, float fs, float fc, uint8_t highpass)
{
    float w0;
    float cw;
    float sw;
    float alpha;
    float a0;

    if (fc > (fs * 0.45f))
    {
        fc = fs * 0.45f;
    }
    if (fc < 0.05f)
    {
        fc = 0.05f;
    }

    w0    = 6.2831853f * fc / fs;
    cw    = cosf(w0);
    sw    = sinf(w0);
    alpha = sw / 1.4142136f;            /* 2*Q锛孮 = 0.7071 */
    a0    = 1.0f + alpha;

    if (highpass != 0U)
    {
        f->b0 = (1.0f + cw) * 0.5f / a0;
        f->b1 = -(1.0f + cw) / a0;
        f->b2 = f->b0;
    }
    else
    {
        f->b0 = (1.0f - cw) * 0.5f / a0;
        f->b1 = (1.0f - cw) / a0;
        f->b2 = f->b0;
    }

    f->a1 = -2.0f * cw / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

/**
 * @brief  鎵ц涓€娆′簩闃舵护娉紙杞疆鐩存帴 II 鍨嬶級
 */
static float cpr_biquad_run(CPR_Biquad_t *f, float x)
{
    float y;

    y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;

    return y;
}

/**
 * @brief  榛樿閰嶇疆
 */
static void cpr_default_config(CPR_Config_t *c)
{
    c->fs            = 120.0f;
    c->hp_hz         = 1.0f;
    c->lp_hz         = 8.0f;
    c->cross_hyst_mg = 30.0f;
    c->warmup_s      = 2.0f;
    c->min_depth_mm  = 10.0f;
    c->max_depth_mm  = 150.0f;
    c->gravity_tau_s = 1.0f;
}

/**
 * @brief  鍒濆鍖?CPR 绠楁硶
 */
void CPR_Init(const CPR_Config_t *cfg)
{
    if (cfg != 0)
    {
        s_cpr_cfg = *cfg;
    }
    else
    {
        cpr_default_config(&s_cpr_cfg);
    }

    if (s_cpr_cfg.fs < 20.0f)
    {
        s_cpr_cfg.fs = 120.0f;
    }
    if (s_cpr_cfg.gravity_tau_s < 0.1f)
    {
        s_cpr_cfg.gravity_tau_s = 1.0f;
    }

    cpr_biquad_init(&s_cpr_hp1, s_cpr_cfg.fs, s_cpr_cfg.hp_hz, 1U);
    cpr_biquad_init(&s_cpr_hp2, s_cpr_cfg.fs, s_cpr_cfg.hp_hz, 1U);
    cpr_biquad_init(&s_cpr_lp,  s_cpr_cfg.fs, s_cpr_cfg.lp_hz,  0U);

    CPR_Reset();
    s_cpr_inited = 1U;
}

/**
 * @brief  娓呯┖绠楁硶鐘舵€? */
void CPR_Reset(void)
{
    s_cpr_hp1.z1 = 0.0f;
    s_cpr_hp1.z2 = 0.0f;
    s_cpr_hp2.z1 = 0.0f;
    s_cpr_hp2.z2 = 0.0f;
    s_cpr_lp.z1  = 0.0f;
    s_cpr_lp.z2  = 0.0f;

    s_cpr_g[0] = 0.0f;
    s_cpr_g[1] = 0.0f;
    s_cpr_g[2] = 0.0f;
    s_cpr_g_ok = 0U;

    s_cpr_av_sq = 0.0f;
    s_cpr_n     = 0U;
    s_cpr_gmax  = 0.0f;

    s_cpr_d_hist[0] = 0.0f;
    s_cpr_d_hist[1] = 0.0f;
    s_cpr_d_hist[2] = 0.0f;

    /* 新方案（漏积分）状态清零 */
    s_lep_v      = 0.0f;
    s_lep_d      = 0.0f;
    s_lep_hi     = 0.0f;
    s_lep_lo     = 0.0f;
    s_lep_t_last = -1.0f;
    s_lep_depth  = 0.0f;
    s_lep_prev   = 0.0f;
    s_lep_avg    = 0.0f;
    s_lep_n      = 0U;
    s_lep_cnt    = 0U;
    s_lep_ev     = 0U;
    s_cpr_d_idx     = 0U;
    s_cpr_d_cnt     = 0U;

    s_cpr_t          = 0.0f;
    s_cpr_t_cross    = 0.0f;
    s_cpr_have_cross = 0U;
    s_cpr_positive   = 0U;

    s_cpr_rate      = 0.0f;
    s_cpr_depth     = 0.0f;
    s_cpr_depth_avg = 0.0f;
    s_cpr_t_event   = 0.0f;
    s_cpr_count     = 0U;

    s_cpr_hyst       = 60.0f;
    s_cpr_period_avg = 0.5f;
    s_cpr_idle       = 1U;
    s_cpr_rms_avg    = 0.0f;
    s_cpr_cross_n   = 0U;
    s_cpr_rej_period = 0U;
    s_cpr_rej_depth = 0U;
    s_cpr_rej_gyro  = 0U;
}

/**
 * @brief  璁剧疆娣卞害鏍囧畾澧炵泭
 */
void CPR_SetDepthGain(float gain)
{
    if ((gain > 0.2f) && (gain < 5.0f))
    {
        s_cpr_gain = gain;
    }
}

/**
 * @brief  閫佷竴甯ф暟鎹紝妫€娴嬩竴娆℃寜鍘嬪苟璁＄畻娣卞害
 * @retval 1 = 瀹屾垚涓€娆℃寜鍘? */

/* ================= 鍩烘尝鎷熷悎锛堟柊娣卞害绠楁硶锛?================= */
#define CPR_FIT_BUF   160

/* ============================================================================
 *  新方案：漏积分二次积分（leaky double integration）
 * ----------------------------------------------------------------------------
 *  思路：不再对加速度做单频拟合，而是直接积分两次得到"位移"，
 *        位移的极大值 = 胸廓回到最高点，极小值 = 压到底，
 *        两者之差就是这一次的按压行程。
 *
 *        v += (a - w0*v) * dt      一次积分（泄漏项 = 自动高通，不用再串高通）
 *        d += (v - w0*d) * dt      二次积分
 *
 *        速度 v 正好是位移的导数，所以位移极值 = v 过零点：
 *          v 由正变负 -> 到顶，记 top
 *          v 由负变正 -> 到底，depth = K * (top - d)，立刻出值
 *
 *  优点：不需要周期 T（旧方案 d 正比于 T^2，周期抖 1% 深度就抖 2%）；
 *        不需要标定带通滤波器的通带增益；积分是 1/w^2 低通，天然抗高频噪声。
 *  标定：K 由三次平台实验拟合得到（1.61 mm vs 平台行程，平台本身有 ±2 mm 误差）
 * ==========================================================================*/
#define CPR_LEAK_HZ         0.5f      /* 泄漏频率 Hz，对应时间常数 318 ms */
#define CPR_DEPTH2_K        0.914f    /* 行程标定增益（三次平台实验联合拟合） */
#define CPR_MG_TO_MMS2      9.80665f  /* 1 mg -> mm/s^2 */
#define CPR_MIN_DEPTH_MM    6.0f      /* 小于这个当噪声丢掉 */
#define CPR_MIN_INTERVAL_S  0.25f     /* 两次按压最小间隔，防抖 */

static float    s_cpr_fit_buf[CPR_FIT_BUF];
static uint16_t s_cpr_fit_idx = 0U;
static float    s_cpr_offset_mm = 13.0f;   /* 娈嬪樊甯告暟淇锛屽彲鐢ㄥ钩鍙版爣瀹?*/

/**
 * @brief  鐢ㄤ竴涓懆鏈熺殑 av 鏍锋湰鍋氭渶灏忎簩涔樺熀娉㈡嫙鍚? * @param  period 鍛ㄦ湡 s
 * @retval 娣卞害 mm锛堝け璐ヨ繑鍥?-1锛? */
static float cpr_fit_depth(float period)
{
    int   n;
    int   k;
    int   idx;
    float w;
    float tt;
    float sn;
    float cs;
    float x;
    float ss = 0.0f;
    float cc = 0.0f;
    float sc = 0.0f;
    float as = 0.0f;
    float ac = 0.0f;
    float det;
    float A;
    float B;

    if (period <= 0.0f)
    {
        return -1.0f;
    }

    n = (int)(period * s_cpr_cfg.fs + 0.5f);
    if (n < 8)
    {
        return -1.0f;
    }
    if (n > CPR_FIT_BUF)
    {
        n = CPR_FIT_BUF;
    }

    w = 6.2831853f / period;

    for (k = 0; k < n; k++)
    {
        idx = (int)s_cpr_fit_idx - n + k;
        while (idx < 0)
        {
            idx += CPR_FIT_BUF;
        }
        tt = (float)(k + 1) / s_cpr_cfg.fs;
        sn = sinf(w * tt);
        cs = cosf(w * tt);
        x  = s_cpr_fit_buf[idx];

        as += x * sn;
        ac += x * cs;
        ss += sn * sn;
        cc += cs * cs;
        sc += sn * cs;
    }

    det = ss * cc - sc * sc;
    if (fabsf(det) < 1e-3f)
    {
        return -1.0f;
    }

    A = ( cc * as - sc * ac) / det;
    B = (-sc * as + ss * ac) / det;

    return 2.0f * sqrtf(A * A + B * B) * 9.80665f / (w * w);
}

/**
 * @brief  新方案：每个采样调用一次，漏积分两次 + 速度过零 + 累计极值
 * @param  av_mg  未滤波的竖直加速度（mg），注意不要传带通之后的值
 * @param  dt     采样间隔 s
 * @param  t_now  当前累计时间 s（CPR_Update 里的 s_cpr_t）
 * @note   位移极值用"累计最大/最小"来取，速度过零只负责定时。
 *         早先版本用 ±15 mm/s 带做判据，那条带会把峰谷各截掉一点点，
 *         导致深度系统性偏小（sim01 少 20%）。改成累计极值后不再有这个问题。
 */
static void cpr_leaky_step(float av_mg, float dt, float t_now)
{
    float w0;
    float a;
    float v_prev;
    float dpt;

    if (dt <= 0.0f)
    {
        return;
    }

    w0     = 6.2831853f * CPR_LEAK_HZ;
    a      = av_mg * CPR_MG_TO_MMS2;     /* mg -> mm/s^2 */
    v_prev = s_lep_v;

    /* 两次带泄漏积分：泄漏项等价于高通，所以这里必须用未高通的 av */
    s_lep_v += (a - w0 * s_lep_v) * dt;
    s_lep_d += (s_lep_v - w0 * s_lep_d) * dt;

    s_lep_ev = 0U;

    /* 累计本周期内的位移最大 / 最小 */
    if (s_lep_d > s_lep_hi)
    {
        s_lep_hi = s_lep_d;
    }
    if (s_lep_d < s_lep_lo)
    {
        s_lep_lo = s_lep_d;
    }

    if ((v_prev < 0.0f) && (s_lep_v >= 0.0f))
    {
        /* 速度由负变正 = 刚过位移谷底（压到底）：本次深度立刻可得 */
        dpt = CPR_DEPTH2_K * (s_lep_hi - s_lep_lo);

        if ((dpt >= CPR_MIN_DEPTH_MM) &&
            ((t_now - s_lep_t_last) >= CPR_MIN_INTERVAL_S))
        {
            s_lep_depth = dpt;

            if (s_lep_n < 2U)
            {
                s_lep_avg = dpt;
                s_lep_n++;
            }
            else
            {
                s_lep_avg = 0.5f * (s_lep_prev + dpt);
            }

            s_lep_prev   = dpt;
            s_lep_cnt++;
            s_lep_ev     = 1U;
            s_lep_t_last = t_now;
        }

        /* 下一段重新累计 */
        s_lep_hi = s_lep_d;
        s_lep_lo = s_lep_d;
    }
}

uint8_t CPR_Update(const IMU_Data_t *imu, float dt, CPR_Result_t *out)
{
    float ax;
    float ay;
    float az;
    float amag;
    float gmag;
    float gn;
    float ux;
    float uy;
    float uz;
    float av;
    float alpha;
    float a_rms;
    float period;
    float half;
    float w;
    float depth;
    float r;
    uint8_t event = 0U;

    if (out == 0)
    {
        return 0U;
    }

    out->disp_mm     = 0.0f;
    out->vel_mm_s    = 0.0f;
    out->depth_mm    = 0.0f;
    out->depth_avg_mm = 0.0f;
    out->disp_mm     = s_lep_d;
    out->vel_mm_s    = s_lep_v;
    out->depth2_mm   = s_lep_depth;
    out->depth2_avg_mm = s_lep_avg;
    out->count2      = s_lep_cnt;
    out->ev2         = 0U;
    out->rate_cpm    = 0.0f;
    out->count       = 0U;
    out->t_ms        = 0U;
    out->interval_ms = 0U;
    out->valid       = 0U;
    out->ready       = 0U;
    out->acc_v_mg    = 0.0f;
    out->acc_mag_mg  = 0.0f;
    out->gyr_mag_dps = 0.0f;
    out->stat        = 0U;
    out->cross_n     = s_cpr_cross_n;
    out->rej_period  = s_cpr_rej_period;
    out->rej_depth   = s_cpr_rej_depth;
    out->rej_gyro    = s_cpr_rej_gyro;

    if (imu == 0)
    {
        return 0U;
    }
    if (dt <= 0.0f)
    {
        return 0U;
    }
    if (dt > 0.2f)
    {
        dt = 0.2f;
    }
    if (s_cpr_inited == 0U)
    {
        CPR_Init(0);
    }

    s_cpr_t += dt;

    ax = imu->acc_mg.x;
    ay = imu->acc_mg.y;
    az = imu->acc_mg.z;

    amag = sqrtf(ax * ax + ay * ay + az * az);
    gmag = sqrtf(imu->gyr_dps.x * imu->gyr_dps.x +
                imu->gyr_dps.y * imu->gyr_dps.y +
                imu->gyr_dps.z * imu->gyr_dps.z);

    out->acc_mag_mg  = amag;
    out->gyr_mag_dps = gmag;
    out->count       = s_cpr_count;
    out->depth_mm    = s_cpr_depth;
    out->depth_avg_mm = s_cpr_depth_avg;
    out->rate_cpm    = s_cpr_rate;
    out->ready       = (s_cpr_t >= s_cpr_cfg.warmup_s) ? 1U : 0U;

    if (amag < 100.0f)
    {
        return 0U;                      /* 瀹屽叏澶遍噸锛屾暟鎹笉鍙俊 */
    }

    /*------------------------------------------------------------------
     * 1. 閲嶅姏鏂瑰悜锛堟參閫熶綆閫氾級
     *----------------------------------------------------------------*/
    if (s_cpr_g_ok == 0U)
    {
        s_cpr_g[0] = ax / amag;
        s_cpr_g[1] = ay / amag;
        s_cpr_g[2] = az / amag;
        s_cpr_g_ok = 1U;
    }
    else
    {
        alpha = dt / (s_cpr_cfg.gravity_tau_s + dt);
        s_cpr_g[0] += (ax / amag - s_cpr_g[0]) * alpha;
        s_cpr_g[1] += (ay / amag - s_cpr_g[1]) * alpha;
        s_cpr_g[2] += (az / amag - s_cpr_g[2]) * alpha;

        gn = sqrtf(s_cpr_g[0] * s_cpr_g[0] +
                   s_cpr_g[1] * s_cpr_g[1] +
                   s_cpr_g[2] * s_cpr_g[2]);
        if (gn < 0.1f)
        {
            gn = 1.0f;
        }
        s_cpr_g[0] /= gn;
        s_cpr_g[1] /= gn;
        s_cpr_g[2] /= gn;
    }

    ux = s_cpr_g[0];
    uy = s_cpr_g[1];
    uz = s_cpr_g[2];

    /*------------------------------------------------------------------
     * 2. 绔栫洿鍔犻€熷害锛屽幓鎺?1 g
     *----------------------------------------------------------------*/
    av = ax * ux + ay * uy + az * uz - 1000.0f;

    /*------------------------------------------------------------------
     * 2b. 新方案：漏积分两次（必须用未带通的 av，泄漏项自己就是高通）
     *     放在预热判断之前，让它一开始就收敛
     *----------------------------------------------------------------*/
    cpr_leaky_step(av, dt, s_cpr_t);

    /*------------------------------------------------------------------
     * 3. 甯﹂€氾細0.8 Hz 鍥涢樁楂橀€?+ 20 Hz 浣庨€?     *    楂橀€氬帇鎺夋墜鐨勬參绉诲拰閲嶅姏鎶曞奖璇樊锛屼綆閫氬帇鎺変紶鎰熷櫒鍣０
     *----------------------------------------------------------------*/
    av = cpr_biquad_run(&s_cpr_hp1, av);
    av = cpr_biquad_run(&s_cpr_hp2, av);
    av = cpr_biquad_run(&s_cpr_lp, av);

    s_cpr_fit_buf[s_cpr_fit_idx] = av;
    s_cpr_fit_idx = (uint16_t)((s_cpr_fit_idx + 1U) % CPR_FIT_BUF);

    out->acc_v_mg = av;

    if (s_cpr_t < s_cpr_cfg.warmup_s)
    {
        return 0U;                      /* 棰勭儹鏈燂紝婊ゆ尝鍣ㄨ繕娌＄ǔ瀹?*/
    }

    /* 瓒呰繃 3 s 娌℃湁鏂版寜鍘嬶細棰戠巼娓呴浂锛屾繁搴﹀钩婊戜篃閲嶆柊寮€濮嬨€?     * 鍚﹀垯鍋滄墜涓€娈垫椂闂村悗锛宎vg 杩樺仠鐣欏湪涓婁竴娆＄殑鏃у€硷紝鐪嬭捣鏉ュ涓嶄笂 */
    if ((s_cpr_t - s_cpr_t_event) > 3.0f)
    {
        /* 鍙湪鈥滃垰杩涘叆绌洪棽鈥濇椂澶嶄綅涓€娆°€傚鏋滄瘡甯ч兘娓咃紝杩囬浂姘歌繙鏀掍笉鎴?         * 涓€涓畬鏁村懆鏈燂紝灏变細鍑虹幇鈥滄寜浜嗗畬鍏ㄦ病鍙嶅簲鈥濓紙涔嬪墠韪╄繃杩欎釜鍧戯級 */
        if (s_cpr_idle == 0U)
        {
            s_cpr_idle      = 1U;
            s_cpr_rate      = 0.0f;
            s_cpr_depth     = 0.0f;
            s_cpr_depth_avg = 0.0f;
            s_cpr_d_cnt     = 0U;
            s_cpr_d_idx     = 0U;
            s_cpr_d_hist[0] = 0.0f;
            s_cpr_d_hist[1] = 0.0f;
            s_cpr_d_hist[2] = 0.0f;

            s_cpr_have_cross = 0U;
            s_cpr_av_sq      = 0.0f;
            s_cpr_n          = 0U;
            s_cpr_gmax       = 0.0f;
            s_cpr_positive   = 0U;
            s_cpr_period_avg = 0.5f;      /* 闃叉闀块棿闅旀妸鑺傚闂ㄩ檺鍗℃ */
            s_cpr_rms_avg    = 0.0f;      /* 鍚岀悊锛岄槻姝㈠箙搴﹂棬闄愰攣姝?*/
        }
    }

    /*------------------------------------------------------------------
     * 4. 涓€涓懆鏈熷唴鐨勮兘閲忓拰鏈€澶ц閫熷害
     *    鐢?av虏 绱姞绠?RMS锛岃€屼笉鏄鏈€澶ф渶灏忓€硷細
     *    宄板嘲鍊煎娉㈠舰閲岀殑"鎶樿/鍐插嚮"鏋佸叾鏁忔劅锛堜竴涓皷宄板氨鑳界炕鍊嶏級锛?     *    RMS 鍙湅鏁翠綋鑳介噺锛屾姉鍐插嚮鑳藉姏寮哄緱澶氥€?     *----------------------------------------------------------------*/
    s_cpr_av_sq += av * av;
    s_cpr_n++;

    if (gmag > s_cpr_gmax)
    {
        s_cpr_gmax = gmag;
    }

    /*------------------------------------------------------------------
     * 5. 鍗婂懆鏈熸娴嬶細涓€涓?涓嬪帇 + 鍥炲脊"璁颁竴娆℃寜鍘?     *
     *    av 鐢辨鍙樿礋 = 寮€濮嬩笅鍘嬶紱鐢辫礋鍙樻 = 涓嬪帇缁撴潫銆?     *    涓€娆℃寜鍘嬪氨鏄竴涓礋鍗婂懆锛屾墍浠ョ敤"璐熷崐鍛ㄦ椂闀?脳 2"浼拌鍛ㄦ湡銆?     *    杩欐牱鍋跺彂鐨勫崟娆℃寜鍘嬩篃鑳界珛鍒荤畻鍑烘繁搴﹀拰棰戠巼锛?     *    涓嶉渶瑕佺瓑涓嬩竴娆℃寜鍘嬫潵娴嬪懆鏈燂紙鏃у仛娉曠殑鏈€澶ф瘺鐥咃級銆?     *----------------------------------------------------------------*/
    if (s_cpr_positive == 0U)
    {
        if (av > s_cpr_hyst)
        {
            s_cpr_t_rise_prev = s_cpr_t_rise;
            s_cpr_t_rise = s_cpr_t;
            half = s_cpr_t - s_cpr_t_cross;
            printf("D,%lu,%d,%d,%.0f\r\n", (unsigned long)HAL_GetTick(), (int)av, (int)s_cpr_hyst, (double)half);

            /* 鍛ㄦ湡蹇呴』钀藉湪褰撳墠鑺傚闄勮繎锛氭瘮骞冲潎鍛ㄦ湡蹇?25% 浠ヤ笂鐨勶紝
             * 鍒や负鍥炲脊灏忚捣浼忥紙瀹冪殑鍛ㄦ湡鐭€佹繁搴﹀繀鐒跺皬锛?*/
            /* 周期必须落在当前节奏附近：比平均周期短 25% 以上的，
             * 判为回弹小起伏（它的周期短、深度必然小）。
             *
             * 注意：0.75 这个系数假设"负半周 ≈ 半个周期"，但实测带通后的
             * 竖直加速度并不对称 —— 负半周只有 0.179~0.188 s（= 0.36~0.38 个周期），
             * 而 0.75*period_avg 正好 0.375，于是大批正常按压被误判丢弃
             * （sim01/sim03 上 63 次只报了 19/12 次）。
             * 改成 0.55 后，只有 2*half < 0.275 s（即周期 < 0.28 s）才丢弃，
             * 既能滤掉回弹小起伏，又不会误杀正常按压。 */
            if ((s_cpr_have_cross != 0U) &&
                (half >= 0.12f) && (half <= 0.50f) &&
                ((2.0f * half) >= (0.55f * s_cpr_period_avg)))
            {
                /* 鍛ㄦ湡浼樺厛鐢ㄧ浉閭讳袱娆℃寜鍘嬬殑鏃堕棿宸紝姣?2脳鍗婂懆鏇村噯锛?                 * 绗竴娆℃病鏈夊弬鑰冩椂閫€鍥炲崐鍛ㄤ及璁?*/
                if (s_cpr_count > 0U)
                {
                    period = s_cpr_t - s_cpr_t_rise_prev;
                    if ((period < 0.30f) || (period > 1.00f))
                    {
                        period = 2.0f * half;
                    }
                }
                else
                {
                    /* 首拍没有"上一次间隔"可比。原来退化成 2*half，但带通后的
                     * 竖直加速度并不对称（实测 half≈0.18 s -> 2*half=0.36 s ->
                     * 167 cpm，比真实值高 40%），而且要靠一阶平滑 15~20 拍才收敛。
                     * 直接用当前节奏均值（初值 0.5 s ≈ 120 cpm），第一拍就准。 */
                    period = s_cpr_period_avg;
                }

                if (s_cpr_n > 0U)
                {
                    a_rms = sqrtf(s_cpr_av_sq / (float)s_cpr_n);
                }
                else
                {
                    a_rms = 0.0f;
                }

                /* 鑷€傚簲杩熸粸锛氭湰鍗婂懆 RMS 鐨?30%锛岄檺鍒?40~100 mg */
                s_cpr_hyst = 0.30f * a_rms;
                if (s_cpr_hyst < 40.0f)
                {
                    s_cpr_hyst = 40.0f;
                }
                if (s_cpr_hyst > 100.0f)
                {
                    s_cpr_hyst = 100.0f;
                }

                /* 鏄庢樉灏忎簬杩戞湡骞冲潎鐨勶紝鏄洖寮瑰皬璧蜂紡锛屼笉鍗曠嫭璁℃暟 */
                if ((a_rms > 10.0f) &&
                    ((s_cpr_rms_avg <= 20.0f) ||
                     (a_rms >= 0.5f * s_cpr_rms_avg)))
                {
                    w = 6.2831853f / period;
                    {
                        float d_fit = cpr_fit_depth(period);

                        if (d_fit > 1.0f)
                        {
                            depth = s_cpr_gain * d_fit;
                        }
                        else
                        {
                            depth = s_cpr_gain *
                                    (2.8284271f * a_rms * 9.80665f) / (w * w);
                        }
                    }

                    /* 娈嬪樊甯告暟淇锛氬墠 2 娆℃寜鍘嬩笉璁″叆锛岄伩鍏嶈捣姝ユ鍋忓ぇ */
                    if (s_cpr_count >= 2U)
                    {
                        depth += s_cpr_offset_mm;
                    }

                    if ((depth >= s_cpr_cfg.min_depth_mm) &&
                        (depth <= s_cpr_cfg.max_depth_mm))
                    {
                        float med;

                        if ((a_rms > 20.0f) && (a_rms < 800.0f))
                        {
                            s_cpr_rms_avg = (s_cpr_rms_avg <= 0.0f)
                                            ? a_rms
                                            : (0.8f * s_cpr_rms_avg + 0.2f * a_rms);
                        }

                        s_cpr_depth   = depth;
                        s_cpr_count++;
                        s_cpr_t_event = s_cpr_t;

                        /* 涓€兼护娉細杩炵画 3 娆″彇涓棿鍊硷紝鍓旈櫎鍗曟寮傚父 */
                        s_cpr_d_hist[s_cpr_d_idx] = depth;
                        s_cpr_d_idx = (uint8_t)((s_cpr_d_idx + 1U) % 3U);
                        if (s_cpr_d_cnt < 3U)
                        {
                            s_cpr_d_cnt++;
                        }

                        med = depth;
                        if (s_cpr_d_cnt >= 3U)
                        {
                            float h0 = s_cpr_d_hist[0];
                            float h1 = s_cpr_d_hist[1];
                            float h2 = s_cpr_d_hist[2];

                            if (((h0 >= h1) && (h0 <= h2)) ||
                                ((h0 >= h2) && (h0 <= h1)))
                            {
                                med = h0;
                            }
                            else if (((h1 >= h0) && (h1 <= h2)) ||
                                     ((h1 >= h2) && (h1 <= h0)))
                            {
                                med = h1;
                            }
                            else
                            {
                                med = h2;
                            }
                        }

                        if (s_cpr_depth_avg <= 0.0f)
                        {
                            s_cpr_depth_avg = med;
                        }
                        else
                        {
                            s_cpr_depth_avg = 0.6f * s_cpr_depth_avg + 0.4f * med;
                        }

                        r = 60.0f / period;
                        if (s_cpr_rate <= 0.0f)
                        {
                            s_cpr_rate = r;
                        }
                        else
                        {
                            s_cpr_rate = 0.8f * s_cpr_rate + 0.2f * r;
                        }

                        s_cpr_idle       = 0U;
                        s_cpr_period_avg = 0.8f * s_cpr_period_avg + 0.2f * period;

                        out->t_ms        = HAL_GetTick();
                        out->interval_ms = (uint32_t)(period * 1000.0f + 0.5f);

                        event = 1U;
                    }
                    else
                    {
                        s_cpr_rej_depth++;
                    }
                }
                else
                {
                    s_cpr_cross_n++;
                }
            }
            else if (s_cpr_have_cross != 0U)
            {
                s_cpr_rej_period++;         /* 鍗婂懆杩囩煭/杩囬暱锛屼涪寮?*/
            }

            s_cpr_positive   = 1U;
            s_cpr_have_cross = 0U;          /* 绛変笅涓€娆′笅鍘?*/
        }
    }
    else
    {
        if (av < -s_cpr_hyst)
        {
            /* 寮€濮嬩竴娆℃柊鐨勪笅鍘嬶細浠庤繖閲屽紑濮嬬疮璁¤兘閲?*/
            s_cpr_positive   = 0U;
            s_cpr_have_cross = 1U;
            s_cpr_t_cross    = s_cpr_t;
            s_cpr_av_sq      = av * av;
            s_cpr_n          = 1U;
            s_cpr_gmax       = gmag;
        }
    }

    /*------------------------------------------------------------------
     * 6. 杈撳嚭
     *----------------------------------------------------------------*/
    out->depth_mm = s_cpr_depth;
    out->depth_avg_mm = s_cpr_depth_avg;
    out->disp_mm  = s_lep_d;
    out->vel_mm_s = s_lep_v;
    out->depth2_mm = s_lep_depth;
    out->depth2_avg_mm = s_lep_avg;
    out->count2   = s_lep_cnt;
    out->ev2      = s_lep_ev;      /* 新方案在位移谷底就给值，见 main.c 的 EVL 行 */
    out->rate_cpm = s_cpr_rate;
    out->count    = s_cpr_count;
    out->valid    = ((s_cpr_depth >= s_cpr_cfg.min_depth_mm) &&
                     (s_cpr_depth <= s_cpr_cfg.max_depth_mm)) ? 1U : 0U;

    return event;
}

float CPR_GetPeriodAvg(void)
{
    return s_cpr_period_avg;
}

