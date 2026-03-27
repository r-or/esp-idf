#pragma once
#include <esp_types.h>
#include <stdint.h>
#include "esp_intr_types.h"
#include "sdkconfig.h"
#include "esp_pm.h"
#include "esp_private/gdma.h"
#include "hal/adc_hal.h"
#include "hal/adc_types.h"

#if SOC_GDMA_SUPPORTED
#include "esp_private/gdma.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "hal/spi_types.h"
#include "esp_private/spi_common_internal.h"
#elif CONFIG_IDF_TARGET_ESP32
#include "driver/i2s_types.h"
#endif
#include "adc_dma_internal.h"



#ifdef __cplusplus
extern "C" {
#endif


typedef struct adc_cont_bare_ctx_t adc_cont_bare_ctx_t;


struct adc_cont_bare_ctx_t {
    uint8_t *rx_dma_buf;
    adc_hal_dma_ctx_t hal;
    intptr_t rx_eof_desc_addr;

    adc_atten_t adc_atten;

    adc_hal_digi_ctrlr_cfg_t hal_digi_ctrlr_cfg;
    adc_digi_output_format_t format;
    void *user_data;
    esp_pm_lock_handle_t pm_lock;


    size_t adc_desc_size;
    adc_dma_t                       adc_dma;
    adc_dma_intr_func_t             adc_intr_func;
};


#ifdef __cplusplus
}
#endif
