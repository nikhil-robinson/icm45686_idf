#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include <inttypes.h>
#include <math.h>
#include "icm45686.h"
#include "spi_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "example";

#define IMU_INT_PINNUM (gpio_num_t)(7)
#define SPI_MISO_IO (gpio_num_t)(43)
#define SPI_MOSI_IO (gpio_num_t)(12)
#define SPI_SCLK_IO (gpio_num_t)(44)
#define SPI_CS_IO (gpio_num_t)(47)
#define IMU_CLK_IN_PIN (gpio_num_t)(8)
#define SPI_FREQ_HZ (24 * 1000 * 1000)

spi_bus_device_handle_t spi_device_handle = NULL;
static QueueHandle_t imu_queue = NULL;

#define SPI_BUFFER_SIZE 30

static void init_spi(void)
{
    spi_config_t bus_conf = {
        .miso_io_num = SPI_MISO_IO,
        .mosi_io_num = SPI_MOSI_IO,
        .sclk_io_num = SPI_SCLK_IO,
    };
    spi_bus_handle_t spi_bus_handle = spi_bus_create(SPI2_HOST, &bus_conf);

    spi_device_config_t device_conf = {
        .cs_io_num = SPI_CS_IO,
        .mode = 0,
        .clock_speed_hz = SPI_FREQ_HZ,
    };
    spi_device_handle = spi_bus_device_create(spi_bus_handle, &device_conf);
}

static int IRAM_ATTR icm45686_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    static uint8_t tx_buffer[SPI_BUFFER_SIZE];
    static uint8_t rx_buffer[SPI_BUFFER_SIZE];
    if (!buf || !len || (len + 1) > SPI_BUFFER_SIZE)
    {
        return -1;
    }

    tx_buffer[0] = reg | 0x80;
    memset(&tx_buffer[1], 0xFF, len);

    spi_bus_transfer_bytes(spi_device_handle, tx_buffer, rx_buffer, len + 1);
    memcpy(buf, &rx_buffer[1], len);

    return 0;
}

static int IRAM_ATTR icm45686_write_regs(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    static uint8_t tx_buffer[SPI_BUFFER_SIZE];
    if (!buf || !len || (len + 1) > SPI_BUFFER_SIZE)
    {
        return -1;
    }

    tx_buffer[0] = reg & 0x7F;
    memcpy(&tx_buffer[1], buf, len);

    spi_bus_transfer_bytes(spi_device_handle, tx_buffer, NULL, len + 1);

    return 0;
}

static void icm45686_irq_handle(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
    bool ready = true;
    xResult = xQueueSendFromISR(imu_queue, &ready, &xHigherPriorityTaskWoken);
    if (xResult == pdPASS)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
}

static void imu_init(void *pvParameters)
{
    imu_queue = xQueueCreate(32, sizeof(bool));


    icm45686_config_t icm45686_config;
    icm45686_default_config(&icm45686_config);
    icm45686_config.transport.read_reg = icm45686_read_regs;
    icm45686_config.transport.write_reg = icm45686_write_regs;
    icm45686_config.transport.serif_type = ICM45686_TRANSPORT_SPI4; // SPI
    icm45686_config.enable_clkin = true;
    icm45686_config.clkin_pin = IMU_CLK_IN_PIN;
    icm45686_config.irq_pin = IMU_INT_PINNUM;
    icm45686_config.irq_handler = icm45686_irq_handle;

    icm45686_handle_t icm45686_handle;
    icm45686_init(&icm45686_config, &icm45686_handle);

    ESP_LOGI(TAG, "ICM45686 initialized successfully");

    bool res;

    while (true)
    {
        xQueueReceive(imu_queue, &res, portMAX_DELAY);
        if (res)
        {
            icm45686_data_t icm_data;
            icm45686_get_register_data(&icm45686_handle, &icm_data);

            ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d | Temp: %d",
                     icm_data.accel_data[0], icm_data.accel_data[1], icm_data.accel_data[2],
                     icm_data.gyro_data[0], icm_data.gyro_data[1], icm_data.gyro_data[2],
                     icm_data.temp_data);
        }
    }
}

void app_main(void)
{
    init_spi();
    vTaskDelay(pdMS_TO_TICKS(3)); // Let IMU power stabilize
    // imu_init();
    xTaskCreatePinnedToCore(imu_init, "imu", 4096, NULL, 15, NULL, 1);
}
