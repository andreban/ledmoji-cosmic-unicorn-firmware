#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"

#include "lwip/dns.h"

#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"

#include "lwip/apps/mqtt_priv.h"

#include "tusb.h"

#include "cosmic_unicorn.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"

#define DEBUG_printf printf

#define IMAGE_WIDTH 32
#define IMAGE_HEIGHT 32
#define BUFFER_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT * 3)
#define MQTT_SERVER_PORT 1883
#define MQTT_TOPIC "ledmoji/32x32"

typedef struct MQTT_CLIENT_T_ {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

err_t mqtt_test_connect(MQTT_CLIENT_T *state);

using namespace pimoroni;
PicoGraphics_PenRGB888 graphics(IMAGE_WIDTH, IMAGE_HEIGHT, nullptr);
CosmicUnicorn cosmic_unicorn;

// Perform initialisation
static MQTT_CLIENT_T* mqtt_client_init(void) {
    auto *state = new MQTT_CLIENT_T;//calloc(1, sizeof(MQTT_CLIENT_T));
    state->received = 0;
    return state;
}

void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T*)callback_arg;
    DEBUG_printf("DNS query finished with resolved addr of %s.\n", ip4addr_ntoa(ipaddr));
    state->remote_addr = *ipaddr;
}

void run_dns_lookup(MQTT_CLIENT_T *state) {
    DEBUG_printf("Running DNS query for %s.\n", MQTT_SERVER_HOST);

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(MQTT_SERVER_HOST, &(state->remote_addr), dns_found, state);
    cyw43_arch_lwip_end();

    if (err == ERR_ARG) {
        DEBUG_printf("failed to start DNS query\n");
        return;
    }

    if (err == ERR_OK) {
        DEBUG_printf("no lookup needed");
        return;
    }

    while (state->remote_addr.addr == 0) {
        cyw43_arch_poll();
        sleep_ms(1);
    }
}

u32_t data_in = 0;
u32_t data_len = 0;
u8_t buffer[BUFFER_SIZE];

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len) {
    DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);

    if (tot_len > BUFFER_SIZE) {
        DEBUG_printf("Message length exceeds buffer size, discarding\n");
    } else {
        DEBUG_printf("Receiving messaged with len %d\n", tot_len);
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    if (data_in > 0) {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0) {
            DEBUG_printf("Message received! %d\n", data_len);
            for (int32_t y = 0; y < IMAGE_HEIGHT; y++) {
                for (int32_t x = 0; x < IMAGE_WIDTH; x++) {
                    int32_t pos = (y * IMAGE_WIDTH + x) * 3;
                    uint8_t red = buffer[pos];
                    uint8_t green = buffer[pos + 1];
                    uint8_t blue = buffer[pos + 2];
                    graphics.set_pen(red, green, blue);
                    graphics.pixel(Point(x, y));
                }
            }
            cosmic_unicorn.update(&graphics);
            DEBUG_printf("Screen updated!\n");
        }
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status != 0) {
        DEBUG_printf("Error during connection: err %d.\n", status);
    } else {
        DEBUG_printf("MQTT connected.\n");
    }
}

void mqtt_sub_request_cb(void *arg, err_t err) {
    DEBUG_printf("mqtt_sub_request_cb: err %d\n", err);
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state) {
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "CosmicUnicorn";
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 0;
    ci.will_topic = NULL;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(
            state->mqtt_client,
            &(state->remote_addr),
            MQTT_SERVER_PORT,
            mqtt_connection_cb,
            state,
            client_info
    );

    if (err != ERR_OK) {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }

    return err;
}

void mqtt_run_test(MQTT_CLIENT_T *state) {
    state->mqtt_client = mqtt_client_new();

    state->counter = 0;

    if (state->mqtt_client == NULL) {
        DEBUG_printf("Failed to create new mqtt client\n");
        return;
    }

    if (mqtt_test_connect(state) == ERR_OK) {
        absolute_time_t timeout = nil_time;
        bool subscribed = false;
        mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb, mqtt_pub_data_cb, 0);

        while (true) {
            cyw43_arch_poll();
            absolute_time_t now = get_absolute_time();
            if (is_nil_time(timeout) || absolute_time_diff_us(now, timeout) <= 0) {
                if (mqtt_client_is_connected(state->mqtt_client)) {
                    cyw43_arch_lwip_begin();

                    if (!subscribed) {
                        mqtt_sub_unsub(
                                state->mqtt_client,
                                MQTT_TOPIC,
                                0,
                                mqtt_sub_request_cb,
                                0,
                                1
                        );
                        subscribed = true;
                    }

                    cyw43_arch_lwip_end();
                } else {
                    // DEBUG_printf(".");
                }
            }
        }
    }
}

int main() {
    stdio_init_all();
    printf("stdio initialised...\n");

    cosmic_unicorn.init();

    // Clear screen.
    graphics.set_pen(0, 0, 0);
    graphics.clear();
    graphics.set_pen(0, 0, 255);
    graphics.set_pixel(Point(0, 0));
    cosmic_unicorn.set_brightness(0.5);
    cosmic_unicorn.update(&graphics);

// Enable the loop below to wait for the serial connection.
//    while (!tud_cdc_connected()) {
//        sleep_ms(100);
//    }

    if (cyw43_arch_init()) {
        DEBUG_printf("failed to initialise\n");
        return 1;
    }
    printf("cyw43 initialised...\n");

    cyw43_arch_enable_sta_mode();
    printf("cyw43_arch_enable_sta_mode...\n");

//    cyw43_wifi_link_status()
    DEBUG_printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        DEBUG_printf("failed to  connect.\n");
        graphics.set_pen(255, 0, 0);
        graphics.set_pixel(Point(0, 0));
        cosmic_unicorn.update(&graphics);
        return 1;
    } else {
        DEBUG_printf("Connected.\n");
    }
    graphics.set_pen(255, 255, 0);
    graphics.set_pixel(Point(0, 0));
    cosmic_unicorn.update(&graphics);
    printf("wifi connected...\n");

    MQTT_CLIENT_T *state = mqtt_client_init();
    printf("MQTT Client initialized...\n");

    run_dns_lookup(state);
    printf("Looked up DNS...\n");

    mqtt_run_test(state);
    printf("mqtt_run_test...\n");

    cyw43_arch_deinit();
    printf("cyw43_arch_deinit...\n");
    return 0;
}
