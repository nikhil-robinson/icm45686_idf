#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "wifi_manager.h"
#include "mavlink.h"

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";

void handle_heartbeat(const mavlink_message_t *message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("Got heartbeat from ");
    switch (heartbeat.autopilot)
    {
    case MAV_AUTOPILOT_GENERIC:
        printf("generic");
        break;
    case MAV_AUTOPILOT_ARDUPILOTMEGA:
        printf("ArduPilot");
        break;
    case MAV_AUTOPILOT_PX4:
        printf("PX4");
        break;
    default:
        printf("other");
        break;
    }
    printf(" autopilot\n");
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1)
    {

        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        static volatile bool src_addr_set = false;

        while (1)
        {
            ESP_LOGI(TAG, "Waiting for data");

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len > 0)
            {
                // Get the sender's ip address as string
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

                // rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);

                mavlink_message_t message;
                mavlink_status_t status;

                src_addr_set = true;

                for (int i = 0; i < len; ++i)
                {
                    if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i], &message, &status) == 1)
                    {
                        ESP_LOGI(TAG, "Received message %d from %d/%d", message.msgid, message.sysid, message.compid);
                        switch (message.msgid)
                        {
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            handle_heartbeat(&message);
                            break;
                        }
                    }
                }
            }

            if (src_addr_set)
            {
                static uint64_t last_send_time = 0;
                uint64_t current_time = esp_timer_get_time();
                // 1 second in microseconds
                if ((current_time - last_send_time) >= 1000000)
                {
                    mavlink_message_t message;

                    const uint8_t system_id = 42;
                    const uint8_t base_mode = 0;
                    const uint8_t custom_mode = 0;

                    mavlink_msg_heartbeat_pack_chan(system_id, MAV_COMP_ID_PERIPHERAL, MAVLINK_COMM_0, &message, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, base_mode, custom_mode, MAV_STATE_STANDBY);
                    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                    const int send_len = mavlink_msg_to_send_buffer(buffer, &message);
                    int err = sendto(sock, buffer, send_len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                    if (err < 0)
                    {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    }
                    last_send_time = current_time;
                }
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(wifi_manager_init());
    xTaskCreate(udp_server_task, "udp_server", 4096, (void *)AF_INET, 5, NULL);
}
