

#include "esp_adc/adc_cali.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_private/adc_dma.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/sar_periph_ctrl.h"
#include "hal/adc_hal.h"
#include "hal/adc_hal_common.h"
#include "hal/adc_ll.h"
#include "hal/adc_types.h"
#include "sdkconfig.h"
#if CONFIG_ADC_ENABLE_DEBUG_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_private/gpio.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_cont_bare.h"
#include "adc_cont_bare_internal.h"
#include "adc_dma_internal.h"

static const char *ADC_TAG = "adc_cont_bare";

DMA_ATTR static volatile dma_descriptor_t adc_descriptors[ADC_INTERNAL_BUF_NUM] = {0};

esp_err_t adc_cont_bare_init(uint8_t *dma_buf, adc_cont_bare_ll_t **ret_data)
{
#if CONFIG_ADC_ENABLE_DEBUG_LOG
    esp_log_level_set(ADC_TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE((ADC_CONVERSION_FRAME_SIZE % SOC_ADC_DIGI_DATA_BYTES_PER_CONV == 0), ESP_ERR_INVALID_ARG, ADC_TAG, "conv_frame_size should be in multiples of `SOC_ADC_DIGI_DATA_BYTES_PER_CONV`");

    adc_cont_bare_ll_t *data = heap_caps_calloc(1, sizeof(adc_cont_bare_ll_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (data == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    adc_cont_bare_ctx_t *adc_ctx = heap_caps_calloc(1, sizeof(adc_cont_bare_ctx_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (adc_ctx == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    //assign static buffer used by DMA
    adc_ctx->rx_dma_buf = dma_buf;

    //assign static dma descriptor
    adc_ctx->hal.rx_desc = (dma_descriptor_t *)adc_descriptors;

    if (!adc_ctx->hal.rx_desc) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    //malloc pattern table
    adc_ctx->hal_digi_ctrlr_cfg.adc_pattern = calloc(1, SOC_ADC_PATT_LEN_MAX * sizeof(adc_digi_pattern_config_t));
    if (!adc_ctx->hal_digi_ctrlr_cfg.adc_pattern) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

#if CONFIG_PM_ENABLE
    ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "adc_dma", &adc_ctx->pm_lock);
    if (ret != ESP_OK) {
        goto cleanup;
    }
#endif //CONFIG_PM_ENABLE

    ret = adc_dma_init(&adc_ctx->adc_dma);
    if (ret != ESP_OK) {
        goto cleanup;
    }

    adc_hal_dma_config_t config = {
        .eof_desc_num = ADC_INTERNAL_BUF_NUM,
        .eof_step = 1,
        .eof_num = ADC_CONVERSION_FRAME_SIZE / SOC_ADC_DIGI_DATA_BYTES_PER_CONV
    };
    adc_hal_dma_ctx_config(&adc_ctx->hal, &config);

    data->descriptors = adc_ctx->hal.rx_desc;
    data->handle = adc_ctx;
    *ret_data = data;
    adc_apb_periph_claim();

    return ret;

cleanup:
    adc_cont_bare_deinit(data);
    return ret;

}

esp_err_t adc_cont_bare_deinit(adc_cont_bare_ll_t *data)
{
    if (data == NULL) {
        return ESP_OK;
    }
    adc_cont_bare_handle_t handle = data->handle;
    if (handle->pm_lock) {
        esp_pm_lock_delete(handle->pm_lock);
    }

    free(handle->rx_dma_buf);
    free(handle->hal.rx_desc);
    free(handle->hal_digi_ctrlr_cfg.adc_pattern);
    adc_dma_deinit(handle->adc_dma);
    free(handle);
    handle = NULL;

    adc_apb_periph_free();

    return ESP_OK;
}

static int8_t adc_digi_get_io_num(adc_unit_t adc_unit, uint8_t adc_channel)
{
    if (adc_unit >= 0 && adc_unit < SOC_ADC_PERIPH_NUM) {
        return adc_channel_io_map[adc_unit][adc_channel];
    }
    return -1;
}

static esp_err_t adc_digi_gpio_init(adc_unit_t adc_unit, uint16_t channel_mask)
{
    esp_err_t ret = ESP_OK;
    uint32_t n = 0;
    int8_t io = 0;

    while (channel_mask) {
        if (channel_mask & 0x1) {
            io = adc_digi_get_io_num(adc_unit, n);
            if (io < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            gpio_config_as_analog(io);
        }
        channel_mask = channel_mask >> 1;
        n++;
    }
    return ret;
}

esp_err_t adc_cont_bare_config(adc_cont_bare_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_STATE, ADC_TAG, "The driver isn't initialised");

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    handle->format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
#else
    handle->format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#endif

    uint32_t clk_src_freq_hz = 0;
    esp_clk_tree_src_get_freq_hz(ADC_DIGI_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_src_freq_hz);

    handle->hal_digi_ctrlr_cfg.adc_pattern_len = 1;
    handle->hal_digi_ctrlr_cfg.sample_freq_hz = ADC_SAMPLE_RATE;
    handle->hal_digi_ctrlr_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;

    adc_digi_pattern_config_t adc_pattern;
    adc_pattern.atten = ADC_ATTEN_DB_12;

#if CONFIG_IDF_TARGET_ESP32
    adc_pattern.channel = ADC_CHANNEL_7 & 0x7;
#else
    adc_pattern.channel = ADC_CHANNEL_3;
#endif

    adc_pattern.unit = ADC_UNIT_1;
    adc_pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ESP_LOGI(ADC_TAG, "adc_pattern.atten is :%"PRIx8, adc_pattern.atten);
    ESP_LOGI(ADC_TAG, "adc_pattern.channel is :%"PRIx8, adc_pattern.channel);
    ESP_LOGI(ADC_TAG, "adc_pattern.unit is :%"PRIx8, adc_pattern.unit);

    memcpy(handle->hal_digi_ctrlr_cfg.adc_pattern, &adc_pattern, sizeof(adc_digi_pattern_config_t));
    handle->hal_digi_ctrlr_cfg.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
    handle->hal_digi_ctrlr_cfg.clk_src_freq_hz = clk_src_freq_hz;

    ESP_LOGI(ADC_TAG, "clk_src_freq_hz is: %d", clk_src_freq_hz);

    const int atten_uninitialized = 999;
    handle->adc_atten = atten_uninitialized;
    uint32_t adc_chan_mask = 0;
    adc_chan_mask |= BIT(adc_pattern.channel);

    if (handle->adc_atten == atten_uninitialized) {
        handle->adc_atten = adc_pattern.atten;
    } else if (handle->adc_atten != adc_pattern.atten) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_digi_gpio_init(ADC_UNIT_1, adc_chan_mask);

    return ESP_OK;
}

esp_err_t adc_cont_bare_start(adc_cont_bare_handle_t handle) {
    ANALOG_CLOCK_ENABLE();

    ADC_BUS_CLK_ATOMIC() {
        adc_ll_reset_register();
    }

    if (handle->pm_lock) {
        ESP_RETURN_ON_ERROR(esp_pm_lock_acquire(handle->pm_lock), ADC_TAG, "acquire pm_lock failed");
    }

    sar_periph_ctrl_adc_continuous_power_acquire();
    adc_lock_acquire(ADC_UNIT_1);

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_hal_calibration_init(ADC_UNIT_1);
    adc_set_hw_calibration_code(ADC_UNIT_1, handle->adc_atten);
#endif
    adc_hal_set_controller(ADC_UNIT_1, ADC_HAL_CONTINUOUS_READ_MODE);
#if !CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)(handle->hal_digi_ctrlr_cfg.clk_src), true));
#endif
    adc_hal_digi_init(&handle->hal);
    adc_hal_digi_controller_config(&handle->hal, &handle->hal_digi_ctrlr_cfg);
    adc_hal_digi_enable(false);

    adc_dma_stop(handle->adc_dma);
    adc_hal_digi_connect(false);

    adc_dma_reset(handle->adc_dma);
    adc_hal_digi_reset();
    adc_hal_digi_dma_link(&handle->hal, handle->rx_dma_buf);

    adc_dma_start(handle->adc_dma, handle->hal.rx_desc);
    adc_hal_digi_connect(true);
    adc_hal_digi_enable(true);
    return ESP_OK;
}

esp_err_t adc_cont_bare_stop(adc_cont_bare_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_STATE, ADC_TAG, "The driver isn't initialised");

    adc_dma_stop(handle->adc_dma);
    adc_hal_digi_enable(false);
    adc_hal_digi_connect(false);
#if ADC_LL_WORKAROUND_CLEAR_EOF_COUNTER
    periph_module_reset(PERIPH_SARADC_MODULE);
    adc_hal_digi_clr_eof();
#endif

    adc_hal_digi_deinit();
#if !CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_clk_tree_enable_src((soc_module_clk_t)(handle->hal_digi_ctrlr_cfg.clk_src), false));
#endif
    adc_lock_release(ADC_UNIT_1);
    sar_periph_ctrl_adc_continuous_power_release();

    //release power manager lock
    if (handle->pm_lock) {
        ESP_RETURN_ON_ERROR(esp_pm_lock_release(handle->pm_lock), ADC_TAG, "release pm_lock failed");
    }
    ANALOG_CLOCK_DISABLE();

    return ESP_OK;
}

