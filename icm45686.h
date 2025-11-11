#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "icm45686_imu/icm45686/imu/inv_imu_defs.h"
#include "icm45686_imu/icm45686/imu/inv_imu_driver_advanced.h"
#include "icm45686_imu/icm45686/imu/inv_imu_driver_aux1.h"
#include "icm45686_imu/icm45686/imu/inv_imu_edmp_defs.h"
#include "icm45686_imu/icm45686/imu/inv_imu_driver.h"
#include "icm45686_imu/icm45686/imu/inv_imu_edmp_memmap.h"
#include "icm45686_imu/icm45686/imu/inv_imu_edmp.h"
#include "icm45686_imu/icm45686/imu/inv_imu_regmap_be.h"
#include "icm45686_imu/icm45686/imu/inv_imu_regmap_le.h"
#include "icm45686_imu/icm45686/imu/inv_imu_selftest.h"
#include "icm45686_imu/icm45686/imu/inv_imu_transport.h"
#include "icm45686_imu/icm45686/imu/inv_imu_version.h"
#include "icm45686_imu/icm45686/imu/inv_imu.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

    enum icm45686_transport_type
    {
        ICM45686_TRANSPORT_I2C = 0, // I2C
        ICM45686_TRANSPORT_SPI3,    // 3-wire SPI
        ICM45686_TRANSPORT_SPI4     // 4-wire SPI
    };

    enum inv_msg_level
    {
        INV_MSG_LEVEL_OFF = 0,
        INV_MSG_LEVEL_ERROR,
        INV_MSG_LEVEL_WARNING,
        INV_MSG_LEVEL_INFO,
        INV_MSG_LEVEL_VERBOSE,
        INV_MSG_LEVEL_DEBUG,
        INV_MSG_LEVEL_MAX
    };

#define INV_MSG(level, ...) inv_msg(level, __VA_ARGS__)

#define SI_CHECK_RC(rc)                                                          \
    do                                                                           \
    {                                                                            \
        if (si_print_error_if_any(rc))                                           \
        {                                                                        \
            INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__); \
            vTaskDelay((100));                                                   \
            return rc;                                                           \
        }                                                                        \
    } while (0)

    typedef struct
    {
        inv_imu_transport_t transport;
        bool accel_enable;
        accel_config0_accel_ui_fs_sel_t accel_fsr;
        accel_config0_accel_odr_t accel_odr;
        ipreg_sys2_reg_131_accel_ui_lpfbw_t accel_bw;
        bool gyro_enable;
        gyro_config0_gyro_ui_fs_sel_t gyro_fsr;
        gyro_config0_gyro_odr_t gyro_odr;
        ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t gyro_bw;
        smc_control_0_accel_lp_clk_sel_t accel_lp_clk;
        bool ln_enable;
        inv_imu_int_pin_config_t int_pin_config;
        inv_imu_int_num_t int_num;
        bool enable_clkin;
        gpio_num_t clkin_pin;
        void (*irq_handler)(void *);
        gpio_num_t irq_pin;
    } icm45686_config_t;

    typedef struct
    {
        icm45686_config_t config;
        inv_imu_device_t imu_dev; /* Driver structure */
    } icm45686_handle_t;

    void inv_msg(int level, const char *str, ...);
    int si_print_error_if_any(int rc);

    void icm45686_default_config(icm45686_config_t *config);
    int icm45686_init(const icm45686_config_t *config, icm45686_handle_t *icm45686);
    int icm45686_deinit(icm45686_handle_t *icm45686);
    void icm45686_clk_in_init(gpio_num_t clk_in_pin);

    void icm45686_sleep_us(icm45686_handle_t *icm45686, uint32_t us);

    int icm45686_soft_reset(icm45686_handle_t *icm45686);
    int icm45686_get_who_am_i(icm45686_handle_t *icm45686, uint8_t *who_am_i);

    int icm45686_set_accel_mode(icm45686_handle_t *icm45686, pwr_mgmt0_accel_mode_t mode);
    int icm45686_set_gyro_mode(icm45686_handle_t *icm45686, pwr_mgmt0_gyro_mode_t mode);

    int icm45686_set_accel_frequency(icm45686_handle_t *icm45686, accel_config0_accel_odr_t frequency);
    int icm45686_set_gyro_frequency(icm45686_handle_t *icm45686, gyro_config0_gyro_odr_t frequency);

    int icm45686_set_accel_fsr(icm45686_handle_t *icm45686, accel_config0_accel_ui_fs_sel_t fsr);
    int icm45686_set_gyro_fsr(icm45686_handle_t *icm45686, gyro_config0_gyro_ui_fs_sel_t fsr);

    int icm45686_set_accel_lp_avg(icm45686_handle_t *icm45686, ipreg_sys2_reg_129_accel_lp_avg_sel_t avg);
    int icm45686_set_gyro_lp_avg(icm45686_handle_t *icm45686, ipreg_sys1_reg_170_gyro_lp_avg_sel_t avg);

    int icm45686_set_accel_ln_bw(icm45686_handle_t *icm45686, ipreg_sys2_reg_131_accel_ui_lpfbw_t bw);
    int icm45686_set_gyro_ln_bw(icm45686_handle_t *icm45686, ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t bw);

    int icm45686_get_register_data(icm45686_handle_t *icm45686, inv_imu_sensor_data_t *data);

    int icm45686_set_fifo_config(icm45686_handle_t *icm45686, const inv_imu_fifo_config_t *fifo_config);
    int icm45686_get_fifo_config(icm45686_handle_t *icm45686, inv_imu_fifo_config_t *fifo_config);
    int icm45686_flush_fifo(icm45686_handle_t *icm45686);
    int icm45686_get_frame_count(icm45686_handle_t *icm45686, uint16_t *frame_count);
    int icm45686_get_fifo_frame(icm45686_handle_t *icm45686, inv_imu_fifo_data_t *data);

    int icm45686_set_config_int(icm45686_handle_t *icm45686, inv_imu_int_num_t num, const inv_imu_int_state_t *it);
    int icm45686_get_config_int(icm45686_handle_t *icm45686, inv_imu_int_num_t num, inv_imu_int_state_t *it);
    int icm45686_set_pin_config_int(icm45686_handle_t *icm45686, inv_imu_int_num_t num, const inv_imu_int_pin_config_t *conf);
    int icm45686_get_int_status(icm45686_handle_t *icm45686, inv_imu_int_num_t num, inv_imu_int_state_t *it);

    int icm45686_get_endianness(icm45686_handle_t *icm45686);
    int icm45686_select_accel_lp_clk(icm45686_handle_t *icm45686, smc_control_0_accel_lp_clk_sel_t clk_sel);

    const char *icm45686_get_version(void);

#ifdef __cplusplus
}
#endif