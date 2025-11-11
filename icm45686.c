#include "icm45686.h"

#define TAG "ICM45686"

#define CLK_IN_TIMER LEDC_TIMER_0
#define CLK_IN_MODE LEDC_LOW_SPEED_MODE
#define CLK_IN_CHANNEL LEDC_CHANNEL_0
#define CLK_IN_DUTY_RES LEDC_TIMER_8_BIT
#define CLK_IN_DUTY (128)
#define CLK_IN_FREQUENCY (32000) // 32000Hz

void inv_msg(int level, const char *str, ...)
{
    if (level == INV_MSG_LEVEL_OFF)
    {
        return;
    }

    va_list args;
    va_start(args, str);
    int len = vsnprintf(NULL, 0, str, args) + 1; // Get required length
    va_end(args);
    char buffer[255];
    if (len < 0 || len > sizeof(buffer))
    {
        return; // Message too long or encoding error
    }
    va_start(args, str);
    vsnprintf(buffer, len, str, args);
    va_end(args);
    switch (level)
    {
    case INV_MSG_LEVEL_ERROR:
        ESP_LOGE(TAG, "%s", buffer);
        break;
    case INV_MSG_LEVEL_WARNING:
        ESP_LOGW(TAG, "%s", buffer);
        break;
    case INV_MSG_LEVEL_INFO:
        ESP_LOGI(TAG, "%s", buffer);
        break;
    case INV_MSG_LEVEL_VERBOSE:
    case INV_MSG_LEVEL_DEBUG:
        ESP_LOGD(TAG, "%s", buffer);
        break;
    default:
        break;
    }
}

int si_print_error_if_any(int rc)
{
    if (rc != 0)
    {
        switch (rc)
        {
        case INV_IMU_ERROR:
            ESP_LOGE(TAG, "Unspecified error (%d)", rc);
            break;
        case INV_IMU_ERROR_TRANSPORT:
            ESP_LOGE(TAG, "Error occurred at transport level (%d)", rc);
            break;
        case INV_IMU_ERROR_TIMEOUT:
            ESP_LOGE(TAG, "Action did not complete in the expected time window (%d)", rc);
            break;
        case INV_IMU_ERROR_BAD_ARG:
            ESP_LOGE(TAG, "Invalid argument provided (%d)", rc);
            break;
        case INV_IMU_ERROR_EDMP_BUF_EMPTY:
            ESP_LOGE(TAG, "EDMP buffer is empty (%d)", rc);
            break;
        default:
            ESP_LOGE(TAG, "Unknown error (%d)", rc);
            break;
        }
    }

    return rc;
}

static inline void delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

void icm45686_default_config(icm45686_config_t *config)
{
    if (config == NULL)
        return;

    memset(config, 0, sizeof(icm45686_config_t));

    config->transport.serif_type = UI_SPI4;
    config->transport.sleep_us = delay_us;
    config->transport.read_reg = NULL;
    config->transport.write_reg = NULL;
    config->accel_enable = true;
    config->int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    config->int_pin_config.int_mode = INTX_CONFIG2_INTX_MODE_PULSE;
    config->int_pin_config.int_drive = INTX_CONFIG2_INTX_DRIVE_PP;
    config->accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;
    config->accel_odr = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ;
    config->accel_bw = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4;
    config->gyro_enable = true;
    config->gyro_fsr = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
    config->gyro_odr = GYRO_CONFIG0_GYRO_ODR_6400_HZ;
    config->gyro_bw = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4;
    config->accel_lp_clk = SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC;
    config->ln_enable = true;
    config->int_num = INV_IMU_INT1;
    config->irq_handler = NULL;
    config->enable_clkin = false;
    config->clkin_pin = GPIO_NUM_NC;
    ESP_LOGI(TAG, "Default ICM45686 configuration applied.");
}

int icm45686_init(const icm45686_config_t *config, icm45686_handle_t *icm45686)
{
    int rc;

    if (config == NULL || icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;

    memset(icm45686, 0, sizeof(icm45686_handle_t));
    memcpy(&icm45686->config, config, sizeof(icm45686_config_t));

    icm45686->imu_dev.transport = config->transport;

    uint8_t whoami = 0;
    inv_imu_int_state_t int_config;
    rc = 0;

    if (icm45686->imu_dev.transport.serif_type == UI_SPI3 || icm45686->imu_dev.transport.serif_type == UI_SPI4)
    {
        drive_config0_t drive_config0;
        drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
        rc |= inv_imu_write_reg(&icm45686->imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
        SI_CHECK_RC(rc);
        delay_us(2); /* Takes effect 1.5 us after the register is programmed */
    }

    /* WHOAMI check */
    rc |= inv_imu_get_who_am_i(&icm45686->imu_dev, &whoami);
    SI_CHECK_RC(rc);
    if (whoami != INV_IMU_WHOAMI)
    {
        ESP_LOGE(TAG, "Erroneous WHOAMI value.");
        ESP_LOGE(TAG, "  - Read 0x%02x", whoami);
        ESP_LOGE(TAG, "  - Expected 0x%02x", INV_IMU_WHOAMI);
        return INV_IMU_ERROR;
    }
    ESP_LOGI(TAG, "WHOAMI check passed. Value: 0x%02x", whoami);

    rc |= inv_imu_soft_reset(&icm45686->imu_dev);
    SI_CHECK_RC(rc);

    /* Configure interrupt pin from config */
    rc |= inv_imu_set_pin_config_int(&icm45686->imu_dev, config->int_num, &config->int_pin_config);
    SI_CHECK_RC(rc);

    /* Enable DRDY interrupt */
    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    int_config.INV_UI_DRDY = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int(&icm45686->imu_dev, config->int_num, &int_config);
    SI_CHECK_RC(rc);

#if INV_IMU_CLKIN_SUPPORTED
    if (config->enable_clkin)
    {
        /* CLKIN configuration */
        rc |= inv_imu_adv_set_int2_pin_usage(&icm45686->imu_dev, IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_CLKIN);
        rc |= inv_imu_adv_enable_clkin_rtc(&icm45686->imu_dev);
        SI_CHECK_RC(rc);
        icm45686_clk_in_init(config->clkin_pin);
    }
#endif

    /* Accelerometer configuration */
    if (config->accel_enable)
    {
        rc |= inv_imu_set_accel_fsr(&icm45686->imu_dev, config->accel_fsr);
        rc |= inv_imu_set_accel_frequency(&icm45686->imu_dev, config->accel_odr);
        rc |= inv_imu_set_accel_ln_bw(&icm45686->imu_dev, config->accel_bw);
        SI_CHECK_RC(rc);
    }

    /* Gyroscope configuration */
    if (config->gyro_enable)
    {
        rc |= inv_imu_set_gyro_fsr(&icm45686->imu_dev, config->gyro_fsr);
        rc |= inv_imu_set_gyro_frequency(&icm45686->imu_dev, config->gyro_odr);
        rc |= inv_imu_set_gyro_ln_bw(&icm45686->imu_dev, config->gyro_bw);
        SI_CHECK_RC(rc);
    }

    /* LP clock selection */
    rc |= inv_imu_select_accel_lp_clk(&icm45686->imu_dev, config->accel_lp_clk);
    SI_CHECK_RC(rc);

    /* Set power modes */
    if (config->ln_enable)
    {
        if (config->accel_enable)
            rc |= inv_imu_set_accel_mode(&icm45686->imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
        if (config->gyro_enable)
            rc |= inv_imu_set_gyro_mode(&icm45686->imu_dev, PWR_MGMT0_GYRO_MODE_LN);

        SI_CHECK_RC(rc);
    }
    else
    {
        if (config->accel_enable)
            rc |= inv_imu_set_accel_mode(&icm45686->imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
        if (config->gyro_enable)
            rc |= inv_imu_set_gyro_mode(&icm45686->imu_dev, PWR_MGMT0_GYRO_MODE_LP);

        SI_CHECK_RC(rc);
    }

    if (config->irq_handler != NULL && config->irq_pin != GPIO_NUM_NC)
    {
        gpio_set_direction(config->irq_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->irq_pin, GPIO_PULLUP_ONLY);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(config->irq_pin, config->irq_handler, NULL);
        gpio_set_intr_type(config->irq_pin, GPIO_INTR_POSEDGE);
        gpio_intr_enable(config->irq_pin);
    }

    SI_CHECK_RC(rc);
    return INV_IMU_OK;
}

void icm45686_clk_in_init(gpio_num_t clk_in_pin)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = CLK_IN_MODE,
        .duty_resolution = CLK_IN_DUTY_RES,
        .timer_num = CLK_IN_TIMER,
        .freq_hz = CLK_IN_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = CLK_IN_MODE,
        .channel = CLK_IN_CHANNEL,
        .timer_sel = CLK_IN_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = clk_in_pin,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_set_duty(CLK_IN_MODE, CLK_IN_CHANNEL, CLK_IN_DUTY);
    ledc_update_duty(CLK_IN_MODE, CLK_IN_CHANNEL);
}

int icm45686_deinit(icm45686_handle_t *icm45686)
{
    int rc;

    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;

    const icm45686_config_t *config = &icm45686->config;
    rc = 0;

    /* Disable interrupts if any */
    inv_imu_int_state_t int_config;
    memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
    rc |= inv_imu_set_config_int(&icm45686->imu_dev, config->int_num, &int_config);
    SI_CHECK_RC(rc);

    /* Remove GPIO ISR if configured */
    if (config->irq_handler != NULL && config->irq_pin != GPIO_NUM_NC)
    {
        gpio_isr_handler_remove(config->irq_pin);
        gpio_intr_disable(config->irq_pin);
    }

    /* Put sensors into OFF mode */
    if (config->accel_enable)
        rc |= inv_imu_set_accel_mode(&icm45686->imu_dev, PWR_MGMT0_ACCEL_MODE_OFF);
    if (config->gyro_enable)
        rc |= inv_imu_set_gyro_mode(&icm45686->imu_dev, PWR_MGMT0_GYRO_MODE_OFF);
    SI_CHECK_RC(rc);

#if INV_IMU_CLKIN_SUPPORTED
    if (config->enable_clkin)
    {
        rc |= inv_imu_adv_disable_clkin_rtc(&icm45686->imu_dev);
        // rc |= inv_imu_adv_set_int2_pin_usage(&icm45686->imu_dev, IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_CLKIN);
        // icm45686_clk_in_deinit(config->clkin_pin);
    }
#endif

    /* Optionally perform soft reset to return IMU to known state */
    rc |= inv_imu_soft_reset(&icm45686->imu_dev);
    SI_CHECK_RC(rc);

    /* Clear handle memory for safety */
    memset(icm45686, 0, sizeof(icm45686_handle_t));

    return INV_IMU_OK;
}

int icm45686_soft_reset(icm45686_handle_t *icm45686)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_soft_reset(&icm45686->imu_dev);
}

int icm45686_get_who_am_i(icm45686_handle_t *icm45686, uint8_t *who_am_i)
{
    if (icm45686 == NULL || who_am_i == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_who_am_i(&icm45686->imu_dev, who_am_i);
}

int icm45686_set_accel_mode(icm45686_handle_t *icm45686, pwr_mgmt0_accel_mode_t mode)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_accel_mode(&icm45686->imu_dev, mode);
}

int icm45686_set_gyro_mode(icm45686_handle_t *icm45686, pwr_mgmt0_gyro_mode_t mode)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_gyro_mode(&icm45686->imu_dev, mode);
}

int icm45686_set_accel_frequency(icm45686_handle_t *icm45686, accel_config0_accel_odr_t freq)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_accel_frequency(&icm45686->imu_dev, freq);
}

int icm45686_set_gyro_frequency(icm45686_handle_t *icm45686, gyro_config0_gyro_odr_t freq)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_gyro_frequency(&icm45686->imu_dev, freq);
}

int icm45686_set_accel_fsr(icm45686_handle_t *icm45686, accel_config0_accel_ui_fs_sel_t fsr)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_accel_fsr(&icm45686->imu_dev, fsr);
}

int icm45686_set_gyro_fsr(icm45686_handle_t *icm45686, gyro_config0_gyro_ui_fs_sel_t fsr)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_gyro_fsr(&icm45686->imu_dev, fsr);
}

int icm45686_set_accel_lp_avg(icm45686_handle_t *icm45686, ipreg_sys2_reg_129_accel_lp_avg_sel_t avg)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_accel_lp_avg(&icm45686->imu_dev, avg);
}

int icm45686_set_gyro_lp_avg(icm45686_handle_t *icm45686, ipreg_sys1_reg_170_gyro_lp_avg_sel_t avg)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_gyro_lp_avg(&icm45686->imu_dev, avg);
}

int icm45686_set_accel_ln_bw(icm45686_handle_t *icm45686, ipreg_sys2_reg_131_accel_ui_lpfbw_t bw)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_accel_ln_bw(&icm45686->imu_dev, bw);
}

int icm45686_set_gyro_ln_bw(icm45686_handle_t *icm45686, ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t bw)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_gyro_ln_bw(&icm45686->imu_dev, bw);
}

int icm45686_get_register_data(icm45686_handle_t *icm45686, icm45686_data_t *data)
{
    if (icm45686 == NULL || data == NULL)
        return INV_IMU_ERROR_BAD_ARG;

    inv_imu_sensor_data_t d;
    int rc = inv_imu_get_register_data(&icm45686->imu_dev, &d);
    memcpy(data->accel_data, d.accel_data, sizeof(d.accel_data));
    memcpy(data->gyro_data, d.gyro_data, sizeof(d.gyro_data));
    data->temp_data = d.temp_data;
    return rc;
}

int icm45686_set_fifo_config(icm45686_handle_t *icm45686, const inv_imu_fifo_config_t *cfg)
{
    if (icm45686 == NULL || cfg == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_fifo_config(&icm45686->imu_dev, cfg);
}

int icm45686_get_fifo_config(icm45686_handle_t *icm45686, inv_imu_fifo_config_t *cfg)
{
    if (icm45686 == NULL || cfg == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_fifo_config(&icm45686->imu_dev, cfg);
}

int icm45686_flush_fifo(icm45686_handle_t *icm45686)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_flush_fifo(&icm45686->imu_dev);
}

int icm45686_get_frame_count(icm45686_handle_t *icm45686, uint16_t *count)
{
    if (icm45686 == NULL || count == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_frame_count(&icm45686->imu_dev, count);
}

int icm45686_get_fifo_frame(icm45686_handle_t *icm45686, inv_imu_fifo_data_t *data)
{
    if (icm45686 == NULL || data == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_fifo_frame(&icm45686->imu_dev, data);
}

int icm45686_set_config_int(icm45686_handle_t *icm45686, inv_imu_int_num_t num, const inv_imu_int_state_t *state)
{
    if (icm45686 == NULL || state == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_config_int(&icm45686->imu_dev, num, state);
}

int icm45686_get_config_int(icm45686_handle_t *icm45686, inv_imu_int_num_t num, inv_imu_int_state_t *state)
{
    if (icm45686 == NULL || state == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_config_int(&icm45686->imu_dev, num, state);
}

int icm45686_set_pin_config_int(icm45686_handle_t *icm45686, inv_imu_int_num_t num, const inv_imu_int_pin_config_t *conf)
{
    if (icm45686 == NULL || conf == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_set_pin_config_int(&icm45686->imu_dev, num, conf);
}

int icm45686_get_int_status(icm45686_handle_t *icm45686, inv_imu_int_num_t num, inv_imu_int_state_t *state)
{
    if (icm45686 == NULL || state == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_int_status(&icm45686->imu_dev, num, state);
}

int icm45686_get_endianness(icm45686_handle_t *icm45686)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_get_endianness(&icm45686->imu_dev);
}

int icm45686_select_accel_lp_clk(icm45686_handle_t *icm45686, smc_control_0_accel_lp_clk_sel_t clk_sel)
{
    if (icm45686 == NULL)
        return INV_IMU_ERROR_BAD_ARG;
    return inv_imu_select_accel_lp_clk(&icm45686->imu_dev, clk_sel);
}

const char *icm45686_get_version(void)
{
    return inv_imu_get_version();
}

void icm45686_sleep_us(icm45686_handle_t *icm45686, uint32_t us)
{
    if (icm45686 == NULL)
        return;
    inv_imu_sleep_us(&icm45686->imu_dev, us);
}
