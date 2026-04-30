/*
 * icm20948.c
 *
 * Original: mokhwasomssi
 * Modified: I2C version for Adafruit ICM20948 breakout
 *
 * Tous les appels SPI ont été remplacés par des appels HAL I2C.
 * Le magnéto AK09916 est accédé via le I2C master interne de l'ICM20948
 * (identique à la version SPI — seule la couche transport change).
 */

#include "icm20948.h"

static float gyro_scale_factor;
static float accel_scale_factor;

/* =========================================================
 * Fonctions bas niveau I2C — remplacent SPI + cs_high/low
 * ========================================================= */

static void select_user_bank(userbank ub)
{
    uint8_t val = (uint8_t)ub;
    HAL_I2C_Mem_Write(ICM20948_I2C, ICM20948_I2C_ADDR, REG_BANK_SEL, 1, &val, 1, 10);
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
    uint8_t val = 0;
    select_user_bank(ub);
    HAL_I2C_Mem_Read(ICM20948_I2C, ICM20948_I2C_ADDR, reg, 1, &val, 1, 100);
    return val;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
    select_user_bank(ub);
    HAL_I2C_Mem_Write(ICM20948_I2C, ICM20948_I2C_ADDR, reg, 1, &val, 1, 100);
}

static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
    static uint8_t reg_val[6];
    select_user_bank(ub);
    HAL_I2C_Mem_Read(ICM20948_I2C, ICM20948_I2C_ADDR, reg, 1, reg_val, len, 100);
    return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
    select_user_bank(ub);
    HAL_I2C_Mem_Write(ICM20948_I2C, ICM20948_I2C_ADDR, reg, 1, val, len, 100);
}

/* =========================================================
 * AK09916 — accès via I2C master interne de l'ICM
 * (identique à la version SPI)
 * ========================================================= */

static uint8_t read_single_ak09916_reg(uint8_t reg)
{
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG,  reg);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(2);
    return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG,  reg);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO,   val);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
{
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG,  reg);
    write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);
    HAL_Delay(1);
    return read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}

/* =========================================================
 * Fonctions principales — identiques à la version SPI
 * ========================================================= */

void icm20948_init()
{
    while (!icm20948_who_am_i());

    icm20948_device_reset();
    icm20948_wakeup();

    icm20948_clock_source(1);
    icm20948_odr_align_enable();

    // NOTE : pas de icm20948_spi_slave_enable() en mode I2C

    icm20948_gyro_low_pass_filter(0);
    icm20948_accel_low_pass_filter(0);

    icm20948_gyro_sample_rate_divider(0);
    icm20948_accel_sample_rate_divider(0);

    icm20948_gyro_calibration();
    icm20948_accel_calibration();

    icm20948_gyro_full_scale_select(_2000dps);
    icm20948_accel_full_scale_select(_16g);
}

void ak09916_init()
{
    icm20948_i2c_master_reset();
    icm20948_i2c_master_enable();
    icm20948_i2c_master_clk_frq(7);

    while (!ak09916_who_am_i());

    ak09916_soft_reset();
    ak09916_operation_mode_setting(continuous_measurement_100hz);
}

void icm20948_gyro_read(axises* data)
{
    uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);
    data->x = (int16_t)(temp[0] << 8 | temp[1]);
    data->y = (int16_t)(temp[2] << 8 | temp[3]);
    data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises* data)
{
    uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);
    data->x = (int16_t)(temp[0] << 8 | temp[1]);
    data->y = (int16_t)(temp[2] << 8 | temp[3]);
    data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor;
}

bool ak09916_mag_read(axises* data)
{
    uint8_t* temp;
    uint8_t drdy, hofl;

    drdy = read_single_ak09916_reg(MAG_ST1) & 0x01;
    if (!drdy) return false;

    temp = read_multiple_ak09916_reg(MAG_HXL, 6);

    hofl = read_single_ak09916_reg(MAG_ST2) & 0x08;
    if (hofl) return false;

    data->x = (int16_t)(temp[1] << 8 | temp[0]);
    data->y = (int16_t)(temp[3] << 8 | temp[2]);
    data->z = (int16_t)(temp[5] << 8 | temp[4]);
    return true;
}

void icm20948_gyro_read_dps(axises* data)
{
    icm20948_gyro_read(data);
    data->x /= gyro_scale_factor;
    data->y /= gyro_scale_factor;
    data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{
    icm20948_accel_read(data);
    data->x /= accel_scale_factor;
    data->y /= accel_scale_factor;
    data->z /= accel_scale_factor;
}

bool ak09916_mag_read_uT(axises* data)
{
    axises temp;
    bool new_data = ak09916_mag_read(&temp);
    if (!new_data) return false;
    data->x = (float)(temp.x * 0.15f);
    data->y = (float)(temp.y * 0.15f);
    data->z = (float)(temp.z * 0.15f);
    return true;
}

/* =========================================================
 * Sub Functions
 * ========================================================= */

bool icm20948_who_am_i()
{
    uint8_t id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);
    return (id == ICM20948_ID);
}

bool ak09916_who_am_i()
{
    uint8_t id = read_single_ak09916_reg(MAG_WIA2);
    return (id == AK09916_ID);
}

void icm20948_device_reset()
{
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
    HAL_Delay(100);
}

void ak09916_soft_reset()
{
    write_single_ak09916_reg(MAG_CNTL3, 0x01);
    HAL_Delay(100);
}

void icm20948_wakeup()
{
    uint8_t val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
    val &= 0xBF;
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, val);
    HAL_Delay(100);
}

void icm20948_sleep()
{
    uint8_t val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
    val |= 0x40;
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, val);
    HAL_Delay(100);
}

void icm20948_i2c_master_reset()
{
    uint8_t val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
    val |= 0x02;
    write_single_icm20948_reg(ub_0, B0_USER_CTRL, val);
}

void icm20948_i2c_master_enable()
{
    uint8_t val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
    val |= 0x20;
    write_single_icm20948_reg(ub_0, B0_USER_CTRL, val);
    HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
    uint8_t val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
    val |= config;
    write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, val);
}

void icm20948_clock_source(uint8_t source)
{
    uint8_t val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
    val |= source;
    write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, val);
}

void icm20948_odr_align_enable()
{
    write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
    uint8_t val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
    val |= config << 3;
    write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
    uint8_t val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
    val |= config << 3;
    write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, val);
    // NOTE : bug corrigé ici — l'original écrivait dans B2_GYRO_CONFIG_1 par erreur
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
    write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
    uint8_t div1 = (uint8_t)(divider >> 8);
    uint8_t div2 = (uint8_t)(0x0F & divider);
    write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, div1);
    write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, div2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
    write_single_ak09916_reg(MAG_CNTL2, mode);
    HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
    axises temp;
    int32_t gyro_bias[3] = {0};
    uint8_t gyro_offset[6] = {0};

    for (int i = 0; i < 100; i++) {
        icm20948_gyro_read(&temp);
        gyro_bias[0] += temp.x;
        gyro_bias[1] += temp.y;
        gyro_bias[2] += temp.z;
    }
    gyro_bias[0] /= 100;
    gyro_bias[1] /= 100;
    gyro_bias[2] /= 100;

    gyro_offset[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF;
    gyro_offset[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
    gyro_offset[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;

    write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
    axises temp;
    uint8_t* temp2;
    uint8_t* temp3;
    uint8_t* temp4;

    int32_t accel_bias[3]     = {0};
    int32_t accel_bias_reg[3] = {0};
    uint8_t accel_offset[6]   = {0};
    uint8_t mask_bit[3]       = {0, 0, 0};

    for (int i = 0; i < 100; i++) {
        icm20948_accel_read(&temp);
        accel_bias[0] += temp.x;
        accel_bias[1] += temp.y;
        accel_bias[2] += temp.z;
    }
    accel_bias[0] /= 100;
    accel_bias[1] /= 100;
    accel_bias[2] /= 100;

    temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
    accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
    mask_bit[0] = temp2[1] & 0x01;

    temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
    accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
    mask_bit[1] = temp3[1] & 0x01;

    temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
    accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
    mask_bit[2] = temp4[1] & 0x01;

    accel_bias_reg[0] -= (accel_bias[0] / 8);
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    accel_offset[1] = (accel_bias_reg[0])       & 0xFE;
    accel_offset[1] = accel_offset[1] | mask_bit[0];

    accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    accel_offset[3] = (accel_bias_reg[1])       & 0xFE;
    accel_offset[3] = accel_offset[3] | mask_bit[1];

    accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    accel_offset[5] = (accel_bias_reg[2])       & 0xFE;
    accel_offset[5] = accel_offset[5] | mask_bit[2];

    write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
    write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
    write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
    uint8_t val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
    switch (full_scale) {
        case _250dps:  val |= 0x00; gyro_scale_factor = 131.0f; break;
        case _500dps:  val |= 0x02; gyro_scale_factor = 65.5f;  break;
        case _1000dps: val |= 0x04; gyro_scale_factor = 32.8f;  break;
        case _2000dps: val |= 0x06; gyro_scale_factor = 16.4f;  break;
    }
    write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
    uint8_t val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
    switch (full_scale) {
        case _2g:  val |= 0x00; accel_scale_factor = 16384.0f; break;
        case _4g:  val |= 0x02; accel_scale_factor = 8192.0f;  break;
        case _8g:  val |= 0x04; accel_scale_factor = 4096.0f;  break;
        case _16g: val |= 0x06; accel_scale_factor = 2048.0f;  break;
    }
    write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, val);
}
