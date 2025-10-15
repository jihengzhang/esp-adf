// 在bt_hf_client_cb函数中添加语音识别相关的日志
void bt_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    // ... 现有代码 ...

    switch (event) {
        // ... 现有case ...
        
        case ESP_HF_CLIENT_CIND_CALL_EVT:
            ESP_LOGE(BT_HF_TAG, "--Call indicator %s", c_call_str[param->call.status]);
            // 添加语音识别相关的日志
            if (param->call.status == ESP_HF_CLIENT_CALL_IN_PROGRESS) {
                ESP_LOGI(TAG, "[VOICE] Call in progress - Voice recognition enabled");
            }
            break;
            
        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
            ESP_LOGE(BT_HF_TAG, "--Call setup indicator %s", c_call_setup_str[param->call_setup.status]);
            // 添加语音识别相关的日志
            if (param->call_setup.status == ESP_HF_CLIENT_CALL_SETUP_INCOMING) {
                ESP_LOGI(TAG, "[VOICE] Incoming call detected - Voice recognition ready");
            }
            break;
            
        default:
            ESP_LOGE(BT_HF_TAG, "HF_CLIENT EVT: %d", event);
            break;
    }
}