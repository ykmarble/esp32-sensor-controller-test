#include "mgos.h"
#include "mgos_system.h"
#include "common/str_util.h"
#include "mgos_rpc.h"
#include "mgos_adc.h"
#include "mgos_onewire.h"
#include "mgos_pwm.h"
#include "mgos_mqtt.h"
#include <stdint.h>

#define MA_KERNEL_LENGTH 10
#define TOPIC_NAME_MAXLEN 256
#define IR_FREQ 38000

volatile static int brightness = 0;
volatile static float temperature = 0;

volatile static double last_tp_conv_issued = -1;  // < 0 means all requests are recieved

static char sensor_topic[TOPIC_NAME_MAXLEN];

static struct mgos_onewire *tp_onewire_inst;

static void br_sensor_timer_cb(void *arg) {
    static int last_rawv[MA_KERNEL_LENGTH] = {};
    static int nexti = 0;
    int ave_rawv = 0;

    last_rawv[nexti++] = mgos_adc_read(mgos_sys_config_get_myconfig_br_sensor_pin());
    if (nexti == MA_KERNEL_LENGTH) nexti = 0;

    for (int i = 0; i < MA_KERNEL_LENGTH; ++i) {
        ave_rawv += last_rawv[i] / MA_KERNEL_LENGTH;
    }

    brightness = ave_rawv;
    LOG(LL_INFO, ("updated br sensor value"));
}

static void tp_sensor_req_timer_cb(void *arg) {
    if (last_tp_conv_issued >= 0)
        return;   // don't issue new request without recieving previous request

    mgos_onewire_reset(tp_onewire_inst);
    mgos_onewire_skip(tp_onewire_inst);
    mgos_onewire_write(tp_onewire_inst, 0x44);
    last_tp_conv_issued = mgos_uptime();

    LOG(LL_INFO, ("request device to convert temperature."));
}

static void tp_sensor_recv_timer_cb(void *arg) {
    static float tp_hist[MA_KERNEL_LENGTH] = {};
    static int nexti = 0;

    if (last_tp_conv_issued < 0)
        return;  // all conversion data has been recieved

    if (mgos_uptime() - last_tp_conv_issued < 0.75)
        return;  // wait conversion until next time

    uint8_t recv[9];
    mgos_onewire_reset(tp_onewire_inst);
    mgos_onewire_skip(tp_onewire_inst);
    mgos_onewire_write(tp_onewire_inst, 0xbe);
    for (int i = 0; i < 9; ++i) {
        recv[i] = mgos_onewire_read(tp_onewire_inst);
    }
    if (mgos_onewire_crc8(recv, 9) == 0) {
        tp_hist[nexti++] = (((int)recv[1] << 8) + (int)recv[0]) / 16.;
        if (nexti == MA_KERNEL_LENGTH) nexti = 0;
        LOG(LL_INFO, ("updated temperature."));
    } else {
        LOG(LL_INFO, ("detected error bit."));
    }

    float ave_tp = 0;
    for (int i = 0; i < MA_KERNEL_LENGTH; ++i) {
        ave_tp += tp_hist[i] / MA_KERNEL_LENGTH;
    }

    temperature = ave_tp;

    last_tp_conv_issued = -1;
}


static void mqtt_pub_timer_cb(void *arg) {
    int br = brightness;
    float tp = temperature;
    float uptime = mgos_uptime();

    LOG(LL_INFO,
        ("uptime: %.2f brightness: %d temperature %.2f", uptime, br, tp));
    if (mgos_mqtt_global_connect()) {
        mgos_mqtt_pubf(sensor_topic, 0, false, "%.2f %d %.2f", uptime, br, tp);
    } else {
        LOG(LL_INFO, ("MQTT publishing is unavilable!!"));
    }
}

static void send_ir(struct mg_rpc_request_info *ri, const char *args,
                    const char *src, void *user_data) {
    const int code[] = {1263,380,1252,459,1252,439,1268,431,385,1288,399,1247,1280,436,380,1305,403,1268,436,1264,432,1255,421,34924,1232,445,1264,455,1225,431,1256,424,419,1280,421,1259,1251,437,405,1277,399,1283,404,1268,419,1269,393,34924,1263,380,1252,459,1252,439,1268,431,385,1288,399,1247,1280,436,380,1305,403,1268,436,1264,432,1255,421,34924,1232,445,1264,455,1225,431,1256,424,419,1280,421,1259,1251,437,405,1277,399,1283,404,1268,419,1269,393};
    const int codelen = 95;

    bool fire = true;

    for (int i = 0; i < codelen; ++i) {
        mgos_pwm_set(mgos_sys_config_get_myconfig_irled_pin(), IR_FREQ, fire? 0.5 : 0);
        fire = !fire;
        mgos_usleep(code[i]);
    }
    mgos_pwm_set(mgos_sys_config_get_myconfig_irled_pin(), IR_FREQ, 0);

    mgos_rpc_send_response(ri, "{}");
}

static void toggle_ssr(struct mg_rpc_request_info *ri, const char *args,
                    const char *src, void *user_data) {
    mgos_gpio_toggle(mgos_sys_config_get_myconfig_ssr_pin());
    mgos_rpc_send_response(ri, "{}");
}

enum mgos_app_init_result mgos_app_init(void) {
    // turn on pilot LED
    mgos_gpio_set_mode(mgos_sys_config_get_myconfig_pilot_pin(), MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(mgos_sys_config_get_myconfig_pilot_pin(), true);

    // setup gpio for ssr pin
    mgos_gpio_set_mode(mgos_sys_config_get_myconfig_ssr_pin(), MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(mgos_sys_config_get_myconfig_ssr_pin(), false);

    // setup ADC
    mgos_adc_enable(mgos_sys_config_get_myconfig_br_sensor_pin());

    // setup onewire library
    tp_onewire_inst = mgos_onewire_create(mgos_sys_config_get_myconfig_tp_sensor_pin());

    // setup thermo sensor
    mgos_onewire_reset(tp_onewire_inst);
    mgos_onewire_skip(tp_onewire_inst);
    mgos_onewire_write(tp_onewire_inst, 0x4e);
    mgos_onewire_write(tp_onewire_inst, 0x00);  // Th
    mgos_onewire_write(tp_onewire_inst, 0x00);  // Tl
    mgos_onewire_write(tp_onewire_inst, 0x7f);  // R1 R0
    mgos_onewire_reset(tp_onewire_inst);

    // register RPC
    mgos_rpc_add_handler("MyApp.SendIR", send_ir, NULL);
    mgos_rpc_add_handler("MyApp.ToggleSSR", toggle_ssr, NULL);

    // build mqtt topic name
    int topic_name_len = c_snprintf(sensor_topic, TOPIC_NAME_MAXLEN,
               mgos_sys_config_get_myconfig_sensor_topic(),
               mgos_sys_config_get_device_id());

    if (topic_name_len < 0) {
        LOG(LL_ERROR, ("failed to call c_snprintf"));
        return MGOS_APP_INIT_ERROR;
    }


    // read sensor value peliodically
    mgos_set_timer(200, MGOS_TIMER_REPEAT, br_sensor_timer_cb, NULL);
    mgos_set_timer(200, MGOS_TIMER_REPEAT, tp_sensor_req_timer_cb, NULL);
    mgos_set_timer(200, MGOS_TIMER_REPEAT, tp_sensor_recv_timer_cb, NULL);

    // run mqtt publisher
    mgos_set_timer(1000, MGOS_TIMER_REPEAT, mqtt_pub_timer_cb, NULL);

  return MGOS_APP_INIT_SUCCESS;
}
