/* Play music from Bluetooth device

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_peripherals.h"
#include "periph_touch.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"

#include "i2s_stream.h"
#include "board.h"
#include "bluetooth_service.h"
#include "filter_resample.h"
#include "raw_stream.h"
#ifdef CONFIG_PAIRING_LED_ENABLE
#include "driver/gpio.h"
#include "freertos/timers.h"
#endif
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "a2dp_stream.h"

#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
#include "filter_resample.h"
#endif
#include "bt_keycontrol.h"

#include "audio_idf_version.h"

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
#define HFP_RESAMPLE_RATE 16000
#else
#define HFP_RESAMPLE_RATE 8000
#endif

static const char *TAG = "BLUETOOTH_EXAMPLE";
static const char *BT_HF_TAG = "BT_HF";

static audio_element_handle_t  raw_read, bt_stream_reader, i2s_stream_writer, i2s_stream_reader;
static audio_pipeline_handle_t pipeline_d, pipeline_e;
static bool is_get_hfp = true;
// Maintain a simple global output volume in percent (0-100) applied to codec/PA path
static int s_output_volume = 60;

// Pairing/Key handling state
static bool s_set_pressed = false;
static TickType_t s_set_press_tick = 0;
static bool s_pairing_pending = false;

#ifdef CONFIG_PAIRING_LED_ENABLE
static TimerHandle_t s_pair_led_timer = NULL;
static bool s_pair_led_inited = false;
static void pairing_led_toggle_cb(TimerHandle_t xTimer)
{
    static bool on = false;
    gpio_set_level(CONFIG_PAIRING_LED_GPIO, on ? 1 : 0);
    on = !on;
}

static void pairing_led_start(void)
{
    if (!s_pair_led_inited) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << CONFIG_PAIRING_LED_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = 0,
            .pull_up_en = 0,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        s_pair_led_inited = true;
    }
    if (!s_pair_led_timer) {
        // 1 Hz blink -> 500ms toggle
        s_pair_led_timer = xTimerCreate("pair_led", pdMS_TO_TICKS(500), pdTRUE, NULL, pairing_led_toggle_cb);
    }
    if (s_pair_led_timer) {
        xTimerStart(s_pair_led_timer, 0);
    }
}

static void pairing_led_stop(void)
{
    if (s_pair_led_timer) {
        xTimerStop(s_pair_led_timer, 0);
    }
    if (s_pair_led_inited) {
        gpio_set_level(CONFIG_PAIRING_LED_GPIO, 0);
    }
}
#else
static inline void pairing_led_start(void) {}
static inline void pairing_led_stop(void) {}
#endif

static void enter_pairing_mode(void)
{
    ESP_LOGI(TAG, "[ * ] Enter pairing (discoverable/connectable)");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    pairing_led_start();
}

static void request_pairing(esp_periph_handle_t bt_periph)
{
    // If already connected, disconnect first then enter pairing when DISCONNECTED event arrives
    uint8_t addr[ESP_BD_ADDR_LEN];
    if (periph_bt_get_connected_bd_addr(bt_periph, addr) == ESP_OK) {
        ESP_LOGI(TAG, "[ * ] Pairing requested: disconnect current A2DP peer first");
        s_pairing_pending = true;
        esp_a2d_sink_disconnect(addr);
        return;
    }
    // Not connected, enter pairing immediately
    enter_pairing_mode();
}

const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
};

// esp_hf_client_connection_state_t
const char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "disabled",
    "enabled",
};

// esp_hf_service_availability_status_t
const char *c_service_availability_status_str[] = {
    "unavailable",
    "available",
};

// esp_hf_roaming_status_t
const char *c_roaming_status_str[] = {
    "inactive",
    "active",
};

// esp_hf_client_call_state_t
const char *c_call_str[] = {
    "NO call in progress",
    "call in progress",
};

// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};

// esp_hf_client_callheld_t
const char *c_call_held_str[] = {
    "NONE held",
    "Held and Active",
    "Held",
};

// esp_hf_response_and_hold_status_t
const char *c_resp_and_hold_str[] = {
    "HELD",
    "HELD ACCEPTED",
    "HELD REJECTED",
};

// esp_hf_client_call_direction_t
const char *c_call_dir_str[] = {
    "outgoing",
    "incoming",
};

// esp_hf_client_call_state_t
const char *c_call_state_str[] = {
    "active",
    "held",
    "dialing",
    "alerting",
    "incoming",
    "waiting",
    "held_by_resp_hold",
};

// esp_hf_current_call_mpty_type_t
const char *c_call_mpty_type_str[] = {
    "single",
    "multi",
};

// esp_hf_volume_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE"
};

// esp_hf_at_response_code_t
const char *c_at_response_code_str[] = {
    "OK",
    "ERROR"
    "ERR_NO_CARRIER",
    "ERR_BUSY",
    "ERR_NO_ANSWER",
    "ERR_DELAYED",
    "ERR_BLACKLILSTED",
    "ERR_CME",
};

// esp_hf_subscriber_service_type_t
const char *c_subscriber_service_type_str[] = {
    "unknown",
    "voice",
    "fax",
};

// esp_hf_client_in_band_ring_state_t
const char *c_inband_ring_state_str[] = {
    "NOT provided",
    "Provided",
};

static void bt_app_hf_client_audio_open(void)
{
    ESP_LOGE(BT_HF_TAG, "bt_app_hf_client_audio_open");
    int sample_rate = HFP_RESAMPLE_RATE;
    audio_element_info_t bt_info = {0};
    audio_element_getinfo(bt_stream_reader, &bt_info);
    bt_info.sample_rates = sample_rate;
    bt_info.channels = 1;
    bt_info.bits = 16;
    audio_element_setinfo(bt_stream_reader, &bt_info);
    audio_element_report_info(bt_stream_reader);
}

static void bt_app_hf_client_audio_close(void)
{
    ESP_LOGE(BT_HF_TAG, "bt_app_hf_client_audio_close");
    int sample_rate = periph_bluetooth_get_a2dp_sample_rate();
    audio_element_info_t bt_info = {0};
    audio_element_getinfo(bt_stream_reader, &bt_info);
    bt_info.sample_rates = sample_rate;
    bt_info.channels = 2;
    bt_info.bits = 16;
    audio_element_setinfo(bt_stream_reader, &bt_info);
    audio_element_report_info(bt_stream_reader);
}

static uint32_t bt_app_hf_client_outgoing_cb(uint8_t *p_buf, uint32_t sz)
{
    int out_len_bytes = 0;
    char *enc_buffer = (char *)audio_malloc(sz);
    AUDIO_MEM_CHECK(BT_HF_TAG, enc_buffer, return 0);
    if (is_get_hfp) {
        out_len_bytes = raw_stream_read(raw_read, enc_buffer, sz);
    }

    if (out_len_bytes == sz) {
        is_get_hfp = false;
        memcpy(p_buf, enc_buffer, out_len_bytes);
        free(enc_buffer);
        return sz;
    } else {
        is_get_hfp = true;
        free(enc_buffer);
        return 0;
    }
}

static void bt_app_hf_client_incoming_cb(const uint8_t *buf, uint32_t sz)
{
    if (bt_stream_reader) {
        if (audio_element_get_state(bt_stream_reader) == AEL_STATE_RUNNING) {
            audio_element_output(bt_stream_reader, (char *)buf, sz);
            esp_hf_client_outgoing_data_ready();
        }
    }
}
/* callback for HF_CLIENT */
void bt_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    if (event <= ESP_HF_CLIENT_RING_IND_EVT) {
        ESP_LOGE(BT_HF_TAG, "APP HFP event: %s", c_hf_evt_str[event]);
    } else {
        ESP_LOGE(BT_HF_TAG, "APP HFP invalid event %d", event);
    }

    switch (event) {
    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        ESP_LOGE(BT_HF_TAG, "--connection state %s, peer feats 0x%" PRIx32 ", chld_feats 0x%" PRIx32,
                 c_connection_state_str[param->conn_stat.state],
                 param->conn_stat.peer_feat,
                 param->conn_stat.chld_feat);
        break;
    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        ESP_LOGE(BT_HF_TAG, "--audio state %s",
                 c_audio_state_str[param->audio_stat.state]);
#if CONFIG_HFP_AUDIO_DATA_PATH_HCI
        if ((param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED)
            || (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)) {
            bt_app_hf_client_audio_open();
            esp_hf_client_register_data_callback(bt_app_hf_client_incoming_cb,
                                                 bt_app_hf_client_outgoing_cb);
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
            bt_app_hf_client_audio_close();
        }
#endif /* #if CONFIG_HFP_AUDIO_DATA_PATH_HCI */
        break;
    case ESP_HF_CLIENT_BVRA_EVT:
        ESP_LOGE(BT_HF_TAG, "--VR state %s",
                 c_vr_state_str[param->bvra.value]);
        break;
    case ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT:
        ESP_LOGE(BT_HF_TAG, "--NETWORK STATE %s",
                 c_service_availability_status_str[param->service_availability.status]);
        break;
    case ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT:
        ESP_LOGE(BT_HF_TAG, "--ROAMING: %s",
                 c_roaming_status_str[param->roaming.status]);
        break;
    case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT:
        ESP_LOGE(BT_HF_TAG, "-- signal strength: %d",
                 param->signal_strength.value);
        break;
    case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT:
        ESP_LOGE(BT_HF_TAG, "--battery level %d",
                 param->battery_level.value);
        break;
    case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        ESP_LOGE(BT_HF_TAG, "--operator name: %s",
                 param->cops.name);
        break;
    case ESP_HF_CLIENT_CIND_CALL_EVT:
        ESP_LOGE(BT_HF_TAG, "--Call indicator %s",
                 c_call_str[param->call.status]);
        break;
    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGE(BT_HF_TAG, "--Call setup indicator %s",
                 c_call_setup_str[param->call_setup.status]);
        break;
    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        ESP_LOGE(BT_HF_TAG, "--Call held indicator %s",
                 c_call_held_str[param->call_held.status]);
        break;
    case ESP_HF_CLIENT_BTRH_EVT:
        ESP_LOGE(BT_HF_TAG, "--response and hold %s",
                 c_resp_and_hold_str[param->btrh.status]);
        break;
    case ESP_HF_CLIENT_CLIP_EVT:
        ESP_LOGE(BT_HF_TAG, "--clip number %s",
                 (param->clip.number == NULL) ? "NULL" : (param->clip.number));
        break;
    case ESP_HF_CLIENT_CCWA_EVT:
        ESP_LOGE(BT_HF_TAG, "--call_waiting %s",
                 (param->ccwa.number == NULL) ? "NULL" : (param->ccwa.number));
        break;
    case ESP_HF_CLIENT_CLCC_EVT:
        ESP_LOGE(BT_HF_TAG, "--Current call: idx %d, dir %s, state %s, mpty %s, number %s",
                 param->clcc.idx,
                 c_call_dir_str[param->clcc.dir],
                 c_call_state_str[param->clcc.status],
                 c_call_mpty_type_str[param->clcc.mpty],
                 (param->clcc.number == NULL) ? "NULL" : (param->clcc.number));
        break;
    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGE(BT_HF_TAG, "--volume_target: %s, volume %d",
                 c_volume_control_target_str[param->volume_control.type],
                 param->volume_control.volume);
                 
        // Map HFP volume (0~15 for speaker/mic) to codec volume (0~100)
        if (param->volume_control.type == ESP_HF_VOLUME_CONTROL_TARGET_SPK) {
            int vol = (param->volume_control.volume * 100) / 15;
            if (vol < 0) vol = 0;
            if (vol > 100) vol = 100;
            s_output_volume = vol;
            // Apply to codec; board_handle is not directly visible here, so fetch from board API
            audio_board_handle_t bh = audio_board_init();
            if (bh && bh->audio_hal) {
                audio_hal_set_volume(bh->audio_hal, s_output_volume);
            }
        }
        break;
    case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        ESP_LOGE(BT_HF_TAG, "--AT response event, code %d, cme %d",
                 param->at_response.code, param->at_response.cme);
        break;
    case ESP_HF_CLIENT_CNUM_EVT:
        ESP_LOGE(BT_HF_TAG, "--subscriber type %s, number %s",
                 c_subscriber_service_type_str[param->cnum.type],
                 (param->cnum.number == NULL) ? "NULL" : param->cnum.number);
        break;
    case ESP_HF_CLIENT_BSIR_EVT:
        ESP_LOGE(BT_HF_TAG, "--inband ring state %s",
                 c_inband_ring_state_str[param->bsir.state]);
        break;
    case ESP_HF_CLIENT_BINP_EVT:
        ESP_LOGE(BT_HF_TAG, "--last voice tag number: %s",
                 (param->binp.number == NULL) ? "NULL" : param->binp.number);
        break;
    default:
        ESP_LOGE(BT_HF_TAG, "HF_CLIENT EVT: %d", event);
        break;
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "[ 1 ] Create Bluetooth service");
    bluetooth_service_cfg_t bt_cfg = {
        .device_name = "ESP-ADF-AUDIO",
        .mode = BLUETOOTH_A2DP_SINK,
    };
    bluetooth_service_start(&bt_cfg);
    esp_hf_client_register_callback(bt_hf_client_cb);
    esp_hf_client_init();

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    // Set an initial output volume for PA/speaker
    audio_hal_set_volume(board_handle->audio_hal, s_output_volume);

    ESP_LOGI(TAG, "[ 3 ] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_d = audio_pipeline_init(&pipeline_cfg);
    pipeline_e = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create i2s stream to write data to codec chip and read data from codec chip");
    i2s_stream_cfg_t i2s_cfg1 = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg1.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg1);

    i2s_stream_cfg_t i2s_cfg2 = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg2.type = AUDIO_STREAM_READER;
#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2s_cfg2.chan_cfg.id = I2S_NUM_1;
#else
    i2s_cfg2.i2s_port = I2S_NUM_1;
    i2s_cfg2.i2s_config.use_apll = false;
#endif  /* ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 0, 0) */
#endif  /* CONFIG_ESP_LYRAT_MINI_V1_1_BOARD */
    i2s_stream_reader = i2s_stream_init(&i2s_cfg2);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER;
    raw_read = raw_stream_init(&raw_cfg);

    ESP_LOGI(TAG, "[3.2] Create Bluetooth stream");
    bt_stream_reader = bluetooth_service_create_stream();

#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    rsp_filter_cfg_t rsp_d_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    audio_element_handle_t filter_d = rsp_filter_init(&rsp_d_cfg);
    audio_pipeline_register(pipeline_d, filter_d, "filter_d");

    rsp_filter_cfg_t rsp_e_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_e_cfg.src_rate = 48000;
    rsp_e_cfg.src_ch = 2;
    rsp_e_cfg.dest_rate = HFP_RESAMPLE_RATE;
    rsp_e_cfg.dest_ch = 1;
    audio_element_handle_t filter_e = rsp_filter_init(&rsp_e_cfg);
    audio_pipeline_register(pipeline_e, filter_e, "filter_e");
#endif

    ESP_LOGI(TAG, "[3.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline_d, bt_stream_reader, "bt");
    audio_pipeline_register(pipeline_d, i2s_stream_writer, "i2s_w");

    audio_pipeline_register(pipeline_e, i2s_stream_reader, "i2s_r");
    audio_pipeline_register(pipeline_e, raw_read, "raw");

    ESP_LOGI(TAG, "[3.4] Link it together [Bluetooth]-->bt_stream_reader-->i2s_stream_writer-->[codec_chip]");
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    const char *link_d[3] = {"bt", "filter_d", "i2s_w"};
    audio_pipeline_link(pipeline_d, &link_d[0], 3);

    const char *link_e[3] = {"i2s_r", "filter_e", "raw"};
    audio_pipeline_link(pipeline_e, &link_e[0], 3);
#else
    const char *link_d[2] = {"bt", "i2s_w"};
    audio_pipeline_link(pipeline_d, &link_d[0], 2);

    const char *link_e[2] = {"i2s_r", "raw"};
    audio_pipeline_link(pipeline_e, &link_e[0], 2);
#endif

    ESP_LOGI(TAG, "[ 4 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[4.1] Initialize Touch peripheral");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[4.2] Create Bluetooth peripheral");
    esp_periph_handle_t bt_periph = bluetooth_service_create_periph();

    ESP_LOGI(TAG, "[4.2] Start all peripherals");
    esp_periph_start(set, bt_periph);

    ESP_LOGI(TAG, "[ 5 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[5.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline_d, evt);

    ESP_LOGI(TAG, "[5.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG, "[ 6 ] Start audio_pipeline");
    audio_pipeline_run(pipeline_d);
    audio_pipeline_run(pipeline_e);

    ESP_LOGI(TAG, "[ 7 ] Listen for all pipeline events");
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)bt_stream_reader
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(bt_stream_reader, &music_info);

            ESP_LOGI(TAG, "[ * ] Receive music info from Bluetooth, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
            rsp_filter_set_src_info(filter_d, music_info.sample_rates, music_info.channels);
            i2s_stream_set_clk(i2s_stream_writer, 48000, 16, 2);
#else
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
#endif

#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
            i2s_stream_set_clk(i2s_stream_reader, music_info.sample_rates, music_info.bits, music_info.channels);
#endif

            continue;
        }
        if ((msg.source_type == PERIPH_ID_TOUCH || msg.source_type == PERIPH_ID_BUTTON || msg.source_type == PERIPH_ID_ADC_BTN)) {
            bool is_press_evt = (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED);
            bool is_release_evt = (msg.cmd == PERIPH_TOUCH_RELEASE || msg.cmd == PERIPH_BUTTON_RELEASE || msg.cmd == PERIPH_ADC_BUTTON_RELEASE);

            if ((int)msg.data == get_input_play_id() && is_press_evt) {
                ESP_LOGI(TAG, "[ * ] [Play] touch tap event");
                periph_bluetooth_play(bt_periph);
            } else if ((int)msg.data == get_input_set_id()) {
                // Record press timestamp on press; decide long/short on release
                if (is_press_evt) {
                    s_set_pressed = true;
                    s_set_press_tick = xTaskGetTickCount();
                }
                // Handle release -> check duration
                if (is_release_evt) {
                    if (s_set_pressed) {
                        s_set_pressed = false;
                        TickType_t elapsed = xTaskGetTickCount() - s_set_press_tick;
                        uint32_t ms = elapsed * portTICK_PERIOD_MS;
                        if (ms >= 1000) {
                            // Long press: request pairing safely (disconnect first)
                            request_pairing(bt_periph);
                        } else {
                            // Short press: keep as Pause/Play toggle for convenience
                            ESP_LOGI(TAG, "[ * ] [Set] short press -> Pause/Resume");
                            periph_bluetooth_pause(bt_periph);
                        }
                    }
                }
            } else if ((int)msg.data == get_input_volup_id() && is_press_evt) {
                ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
                // Increase local PA/codec volume
                s_output_volume += 5;
                if (s_output_volume > 100) s_output_volume = 100;
                audio_hal_set_volume(board_handle->audio_hal, s_output_volume);
                ESP_LOGI(TAG, "[ * ] Volume: %d%%", s_output_volume);
            } else if ((int)msg.data == get_input_voldown_id() && is_press_evt) {
                ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
                // Decrease local PA/codec volume
                s_output_volume -= 5;
                if (s_output_volume < 0) s_output_volume = 0;
                audio_hal_set_volume(board_handle->audio_hal, s_output_volume);
                ESP_LOGI(TAG, "[ * ] Volume: %d%%", s_output_volume);
            }
        }

        /* Stop when the Bluetooth is disconnected or suspended */
        if (msg.source_type == PERIPH_ID_BLUETOOTH && msg.source == (void *)bt_periph) {
            if (msg.cmd == PERIPH_BLUETOOTH_DISCONNECTED) {
                ESP_LOGW(TAG, "[ * ] Bluetooth disconnected");
                if (s_pairing_pending) {
                    s_pairing_pending = false;
                    enter_pairing_mode();
                }
                continue;
            } else if (msg.cmd == PERIPH_BLUETOOTH_CONNECTED) {
                pairing_led_stop();
            }
        }
        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (int)msg.data == AEL_STATUS_STATE_STOPPED) {
            ESP_LOGW(TAG, "[ * ] Stop event received");
            break;
        }
    }

    ESP_LOGI(TAG, "[ 8 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline_d);
    audio_pipeline_wait_for_stop(pipeline_d);
    audio_pipeline_terminate(pipeline_d);
    audio_pipeline_stop(pipeline_e);
    audio_pipeline_wait_for_stop(pipeline_e);
    audio_pipeline_terminate(pipeline_e);

    audio_pipeline_unregister(pipeline_d, bt_stream_reader);
    audio_pipeline_unregister(pipeline_d, i2s_stream_writer);

    audio_pipeline_unregister(pipeline_e, i2s_stream_reader);
    audio_pipeline_unregister(pipeline_e, raw_read);

#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    audio_pipeline_unregister(pipeline_d, filter_d);
    audio_pipeline_unregister(pipeline_e, filter_e);
#endif
    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline_d);

    /* Stop all peripherals before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline_d);
    audio_element_deinit(bt_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(raw_read);
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    audio_element_deinit(filter_d);
    audio_element_deinit(filter_e);
#endif
    esp_periph_set_destroy(set);
    bluetooth_service_destroy();
}
