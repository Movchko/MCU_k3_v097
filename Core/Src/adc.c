#include "main.h"
#include "app.h"

uint16_t MCU_K3_ADC_VAL[MCU_K3_NUM_ADC_CHANNEL];

static uint16_t adc_filter_val[MCU_K3_NUM_ADC_CHANNEL];
static uint16_t adc_sma_buf[MCU_K3_NUM_ADC_CHANNEL][MCU_K3_FILTERSIZE];
static uint32_t adc_sma_sum[MCU_K3_NUM_ADC_CHANNEL];
static uint8_t  adc_sma_fill_index[MCU_K3_NUM_ADC_CHANNEL];
static uint8_t  adc_sma_index[MCU_K3_NUM_ADC_CHANNEL];

static volatile uint16_t lim2_low_adc_filtered = 0;
static volatile uint16_t lim2_high_adc_filtered = 0;
static volatile uint16_t lim1_low_adc_filtered = 0;
static volatile uint16_t lim1_high_adc_filtered = 0;
static volatile uint16_t ign_adc_filtered = 0;
static volatile uint16_t u24_adc_filtered = 0;

static uint16_t SmaProcess(uint8_t num, uint16_t val)
{
    uint16_t old_val = 0;

    if (adc_sma_fill_index[num] == MCU_K3_FILTERSIZE) {
        old_val = adc_sma_buf[num][adc_sma_index[num]];
        adc_sma_sum[num] -= old_val;
    } else {
        adc_sma_fill_index[num]++;
    }

    adc_sma_buf[num][adc_sma_index[num]] = val;
    adc_sma_sum[num] += val;

    adc_sma_index[num]++;
    if (adc_sma_index[num] >= MCU_K3_FILTERSIZE) {
        adc_sma_index[num] = 0;
    }

    return (uint16_t)(adc_sma_sum[num] / adc_sma_fill_index[num]);
}

uint16_t ADC_GetLimit1LowFiltered(void)  { return lim1_low_adc_filtered;  }
uint16_t ADC_GetLimit1HighFiltered(void) { return lim1_high_adc_filtered; }
uint16_t ADC_GetLimit2LowFiltered(void)  { return lim2_low_adc_filtered;  }
uint16_t ADC_GetLimit2HighFiltered(void) { return lim2_high_adc_filtered; }
uint16_t ADC_GetIgniterFiltered(void)    { return ign_adc_filtered;       }
uint16_t ADC_GetU24Filtered(void)        { return u24_adc_filtered;       }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }

    for (uint8_t i = 0; i < MCU_K3_NUM_ADC_CHANNEL; i++) {
        adc_filter_val[i] = SmaProcess(i, MCU_K3_ADC_VAL[i]);
    }

    /* Карта каналов АЦП (MCU_k3_v097, по Rank):
     * [0] CH0  -> контроль спички 1
     * [1] CH1  -> датчик 2 "+" (high)
     * [2] CH3  -> датчик 1 "-" (low)
     * [3] CH11 -> входное 24В
     * [4] CH14 -> датчик 2 "-" (low)
     * [5] CH18 -> датчик 1 "+" (high)
     */
    ign_adc_filtered       = adc_filter_val[0];
    lim2_high_adc_filtered = adc_filter_val[1];
    lim1_low_adc_filtered  = adc_filter_val[2];
    u24_adc_filtered       = adc_filter_val[3];
    lim2_low_adc_filtered  = adc_filter_val[4];
    lim1_high_adc_filtered = adc_filter_val[5];

    App_SetLimit1AdcValues(lim1_low_adc_filtered, lim1_high_adc_filtered, u24_adc_filtered);
    App_SetLimit2AdcValues(lim2_low_adc_filtered, lim2_high_adc_filtered, u24_adc_filtered);
}

