#pragma once

#include "hal/adc_types.h"
#include "esp_err.h"
#include "hal/dma_types.h"

#ifdef __cplusplus
extern "C" {
#endif


#define ADC_UNIT        ADC_UNIT_1
#define ADC_CONV_MODE   ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH   SOC_ADC_DIGI_MAX_BITWIDTH

#define ADC_CONVERSION_FRAME_SIZE 4092  // max size of a descriptor: 1023 samples
#define INTERNAL_BUF_NUM      5         // amount of descriptors



typedef struct adc_cont_bare_ctx_t *adc_cont_bare_handle_t;
typedef struct {
    dma_descriptor_t *descriptors;
    adc_cont_bare_handle_t handle;
} adc_cont_bare_ll_t;


esp_err_t adc_cont_bare_init(uint8_t *dma_buf, adc_cont_bare_ll_t **ret_data);

esp_err_t adc_cont_bare_deinit(adc_cont_bare_ll_t *data);

esp_err_t adc_cont_bare_config(adc_cont_bare_handle_t handle);

esp_err_t adc_cont_bare_start(adc_cont_bare_handle_t handle);

esp_err_t adc_cont_bare_stop(adc_cont_bare_handle_t handle);

#ifdef __cplusplus
}
#endif
