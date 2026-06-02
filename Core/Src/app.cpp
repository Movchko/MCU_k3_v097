#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_igniter.hpp"
#include "device_lswitch.hpp"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#include <string.h>

#include "main.h"
#include "mku_cfg_flash.h"

MKUCfg g_cfg;
MKUCfg g_saved_cfg;

/* 1: limit1, 2: limit2, 3: igniter */
static VDeviceLimitSwitch g_limit1(1);
static VDeviceLimitSwitch g_limit2(2);
static VDeviceIgniter g_igniter(3);

/* Калибровка "сырых" Ом в шкалу порогов device_lib:
 * - открытие должно попадать в 100..159 Ом
 * - норма должна попадать в 820..2000 Ом
 * Точки калибровки с текущего стенда:
 *   raw=0      -> mapped=120  (Открытие)
 *   raw=18437  -> mapped=1000 (Норма)
 */
static uint16_t App_MapLswitchResistanceForLib(uint32_t raw_ohm, uint8_t normal_closed)
{
    const uint32_t RAW_OPEN  = 0u;
    const uint32_t LIB_OPEN  = 120u;
    const uint32_t RAW_NORM  = 18437u;
    const uint32_t LIB_NORM  = 1000u;
    const uint32_t dst_open  = normal_closed ? LIB_NORM : LIB_OPEN;
    const uint32_t dst_norm  = normal_closed ? LIB_OPEN : LIB_NORM;

    if (raw_ohm <= RAW_OPEN) {
        return (uint16_t)dst_open;
    }

    /* Линейная интерполяция между двумя опорными точками.
     * В режиме NC наклон отрицательный, поэтому считаем в signed.
     */
    const int32_t den = (int32_t)(RAW_NORM - RAW_OPEN);
    const int32_t d_dst = (int32_t)dst_norm - (int32_t)dst_open;
    int64_t num = (int64_t)((int32_t)raw_ohm - (int32_t)RAW_OPEN) * (int64_t)d_dst;
    if (num >= 0) {
        num += (int64_t)(den / 2);
    } else {
        num -= (int64_t)(den / 2);
    }
    int64_t mapped_signed = (int64_t)(int32_t)dst_open + (num / (int64_t)den);

    if (mapped_signed < 0) {
        mapped_signed = 0;
    }
    if (mapped_signed > 65535) {
        mapped_signed = 65535;
    }
    return (uint16_t)mapped_signed;
}

static uint32_t g_extinguish_deadline_ms[NUM_DEV_IN_MCU];
static uint8_t  g_extinguish_armed[NUM_DEV_IN_MCU];

void RcvSetSystemTime(uint8_t *data) { (void)data; }
void RcvStatusFire() {}
void RcvReplyStatusFire() {}

static int8_t App_FindIgniterSlotByMsgId(uint32_t MsgID)
{
    can_ext_id_t id;
    id.ID = MsgID & 0x0FFFFFFFu;

    if ((id.field.d_type & 0x7Fu) != DEVICE_IGNITER_TYPE) {
        return -1;
    }
    if ((id.field.h_adr != g_cfg.UId.devId.h_adr) ||
        ((id.field.zone & 0x7Fu) != (g_cfg.UId.devId.zone & 0x7Fu))) {
        return -1;
    }

    if ((id.field.l_adr & 0x3Fu) == 3u && g_cfg.VDtype[2] == DEVICE_IGNITER_TYPE) {
        return 2;
    }
    return -1;
}

extern "C" void RcvStartExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    if (is_mine == 0u) {
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    /* payload backend fire: [0]=cmd, [1]=zone, [2]=zone_delay_s, [3]=module_delay_s */
    uint8_t zd = MsgData[2];
    uint8_t md = MsgData[3];
    uint32_t delay_ms = ((uint32_t)zd + (uint32_t)md) * 1000u;

    g_extinguish_deadline_ms[(uint8_t)ign_slot] = HAL_GetTick() + delay_ms;
    g_extinguish_armed[(uint8_t)ign_slot] = 1u;
    SetReplyStartExtinguishment((uint8_t)(ign_slot + 1)); /* slot2->dev3 */
}

extern "C" void RcvStopExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    (void)MsgData;
    if (is_mine == 0u) {
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    g_extinguish_armed[(uint8_t)ign_slot] = 0u;
    SetReplyStopExtinguishment((uint8_t)(ign_slot + 1)); /* slot2->dev3 */
}

static void App_DPT1_SetResMeasureMode() { HAL_GPIO_WritePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin, GPIO_PIN_SET); }
static void App_DPT1_SetMaxMeasureMode() { HAL_GPIO_WritePin(LINE1_EN_GPIO_Port, LINE1_EN_Pin, GPIO_PIN_SET); }
static void App_DPT2_SetResMeasureMode() { HAL_GPIO_WritePin(LINE2_EN_GPIO_Port, LINE2_EN_Pin, GPIO_PIN_SET); }
static void App_DPT2_SetMaxMeasureMode() { HAL_GPIO_WritePin(LINE2_EN_GPIO_Port, LINE2_EN_Pin, GPIO_PIN_SET); }

/* callback статуса: отправляем его через CAN по протоколу backend */
static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters)
{
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    SendMessage(DNum, Code, data, 0, BUS_CAN12);
}

void SetHAdr(uint8_t h_adr)
{
    g_cfg.UId.devId.h_adr = h_adr;
    extern uint8_t nDevs;
    extern Device BoardDevicesList[];
    for (uint8_t i = 0; i < nDevs; i++) {
        BoardDevicesList[i].h_adr = g_cfg.UId.devId.h_adr;
    }
    SaveConfig();
}

extern "C" {

void DefaultConfig(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    memset(&g_cfg, 0, sizeof(g_cfg));

    g_cfg.UId.UId0 = uid0;
    g_cfg.UId.UId1 = uid1;
    g_cfg.UId.UId2 = uid2;
    g_cfg.UId.UId3 = HAL_GetDEVID();
    g_cfg.UId.UId4 = 1;

    g_cfg.UId.devId.zone  = 0;
    g_cfg.UId.devId.l_adr = 0;

    uint8_t hadr = static_cast<uint8_t>(uid0 & 0xFFu);
    if (hadr == 0u) {
        hadr = static_cast<uint8_t>(uid1 & 0xFFu);
        if (hadr == 0u) {
            hadr = 1u;
        }
    }

    g_cfg.UId.devId.h_adr  = hadr;
    g_cfg.UId.devId.d_type = DEVICE_MCU_K3;

    g_cfg.VDtype[0] = DEVICE_LSWITCH_TYPE;
    g_cfg.VDtype[1] = DEVICE_LSWITCH_TYPE;
    g_cfg.VDtype[2] = DEVICE_IGNITER_TYPE;

    DeviceDPTConfig *l1 = reinterpret_cast<DeviceDPTConfig*>(g_cfg.Devices[0].reserv);
    DeviceDPTConfig *l2 = reinterpret_cast<DeviceDPTConfig*>(g_cfg.Devices[1].reserv);
    memset(l1, 0, sizeof(DeviceDPTConfig));
    memset(l2, 0, sizeof(DeviceDPTConfig));

    l1->mode                  = 1; /* концевик */
    l1->use_max               = 0;
    l1->max_fire_threshold_c  = 60;
    l1->state_change_delay_ms = 100;

    l2->mode                  = 1; /* концевик */
    l2->use_max               = 0;
    l2->max_fire_threshold_c  = 60;
    l2->state_change_delay_ms = 100;

    DeviceIgniterConfig *ign = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[2].reserv);
    memset(ign, 0, sizeof(DeviceIgniterConfig));
    ign->disable_sc_check     = 1u;
    ign->threshold_break_low  = 1000u;
    ign->threshold_break_high = 3000u;
    ign->burn_retry_count     = 0u;
}

void ResetMCU(void) { NVIC_SystemReset(); }

uint32_t GetID(void)
{
    uint32_t id0 = HAL_GetUIDw0();
    uint32_t id1 = HAL_GetUIDw1();
    uint32_t id2 = HAL_GetUIDw2();
    return (id0 ^ id1 ^ id2);
}

void MCU_K3CommandCB(uint8_t Command, uint8_t *Parameters)
{
    if (Command == 20) {
        g_cfg.UId.devId.zone = Parameters[0];
        SaveConfig();
    }
}

void CommandCB(uint8_t Dev, uint8_t Command, uint8_t *Parameters)
{
    switch (Dev) {
    case 0: MCU_K3CommandCB(Command, Parameters); break;
    case 1: g_limit1.CommandCB(Command, Parameters); break;
    case 2: g_limit2.CommandCB(Command, Parameters); break;
    case 3: g_igniter.CommandCB(Command, Parameters); break;
    default: break;
    }
}

void ListenerCommandCB(uint32_t MsgID, uint8_t *MsgData)
{
    (void)MsgID;
    (void)MsgData;
}

void App_Init(void)
{
    extern Device BoardDevicesList[];
    extern uint8_t nDevs;

    if (!FlashReadConfig(&g_cfg)) {
        DefaultConfig();
        SaveConfig();
    }
    g_saved_cfg = g_cfg;
    SetConfigPtr(reinterpret_cast<uint8_t *>(&g_saved_cfg),
                 reinterpret_cast<uint8_t *>(&g_cfg));

    g_limit1.DeviceInit(&g_cfg.Devices[0]);
    g_limit1.VDeviceSetStatus = VDeviceSetStatus;
    g_limit1.VDeviceSaveCfg   = SaveConfig;
    g_limit1.DPT_SetResMeasureMode = App_DPT1_SetResMeasureMode;
    g_limit1.DPT_SetMaxMeasureMode = App_DPT1_SetMaxMeasureMode;
    g_limit1.Init();
    g_cfg.VDtype[0] = g_limit1.GetDT();

    g_limit2.DeviceInit(&g_cfg.Devices[1]);
    g_limit2.VDeviceSetStatus = VDeviceSetStatus;
    g_limit2.VDeviceSaveCfg   = SaveConfig;
    g_limit2.DPT_SetResMeasureMode = App_DPT2_SetResMeasureMode;
    g_limit2.DPT_SetMaxMeasureMode = App_DPT2_SetMaxMeasureMode;
    g_limit2.Init();
    g_cfg.VDtype[1] = g_limit2.GetDT();

    g_igniter.DeviceInit(&g_cfg.Devices[2]);
    g_igniter.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter.VDeviceSaveCfg   = SaveConfig;
    g_igniter.Init();

    nDevs = 1;
    BoardDevicesList[0].zone   = g_cfg.UId.devId.zone;
    BoardDevicesList[0].h_adr  = g_cfg.UId.devId.h_adr;
    BoardDevicesList[0].l_adr  = g_cfg.UId.devId.l_adr;
    BoardDevicesList[0].d_type = DEVICE_MCU_K3;

    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone   = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr  = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr  = 1;
        BoardDevicesList[nDevs].d_type = g_limit1.GetDT();
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone   = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr  = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr  = 2;
        BoardDevicesList[nDevs].d_type = g_limit2.GetDT();
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone   = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr  = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr  = 3;
        BoardDevicesList[nDevs].d_type = DEVICE_IGNITER_TYPE;
        nDevs++;
    }

    App_DPT1_SetResMeasureMode();
    App_DPT2_SetResMeasureMode();

    extern bool isListener;
    isListener = true;
}

void App_SetLimit1AdcValues(uint16_t ch_l, uint16_t ch_h, uint16_t ch_u24)
{
    const uint32_t VREF_MV = 3300u;
    const uint32_t ADC_MAX = 4095u;
    const uint32_t R0_OHM  = 1925u;
    const uint32_t DIV_K   = 2;

    uint32_t v_adc_l_mv = (uint32_t)ch_l * VREF_MV / ADC_MAX;
    uint32_t v_adc_h_mv = (uint32_t)ch_h * VREF_MV / ADC_MAX;
    uint32_t v_adc_u_mv = (uint32_t)ch_u24 * VREF_MV / ADC_MAX;

    uint32_t v_line_l_mv = v_adc_l_mv * DIV_K;
    uint32_t v_line_h_mv = v_adc_h_mv * DIV_K;
    uint32_t u24_mv      = 5000 * DIV_K;//v_adc_u_mv * DIV_K; // мы всегда в этой версии и далее подаём 5В.

    if (v_line_l_mv == 0u) {
        v_line_l_mv = 1u;
    }

    uint32_t r_line_ohm = 0u;
    if (v_line_h_mv > v_line_l_mv) {
        r_line_ohm = R0_OHM * (v_line_h_mv - v_line_l_mv) / v_line_l_mv;
    }

    int32_t k_scaled = -346;
    k_scaled += (int32_t)(673u * (u24_mv / 10u)) / 1000;
    if (k_scaled < 300)  k_scaled = 300;
    if (k_scaled > 1500) k_scaled = 1500;

    uint32_t r_corr = (uint32_t)((r_line_ohm * (uint32_t)k_scaled + 500u) / 1000u);
    DeviceLimitSwitchConfig *cfg = reinterpret_cast<DeviceLimitSwitchConfig*>(g_cfg.Devices[0].reserv);
    uint8_t normal_closed = (cfg != nullptr && cfg->normal_closed) ? 1u : 0u;
    uint16_t r_for_lib = App_MapLswitchResistanceForLib(r_corr, normal_closed);

    g_limit1.SetAdcValues(r_for_lib, (uint16_t)ch_h);
}

void App_SetLimit2AdcValues(uint16_t ch_l, uint16_t ch_h, uint16_t ch_u24)
{
    const uint32_t VREF_MV = 3300u;
    const uint32_t ADC_MAX = 4095u;
    const uint32_t R0_OHM  = 1925u;
    const uint32_t DIV_K   = 2;

    uint32_t v_adc_l_mv = (uint32_t)ch_l * VREF_MV / ADC_MAX;
    uint32_t v_adc_h_mv = (uint32_t)ch_h * VREF_MV / ADC_MAX;
    uint32_t v_adc_u_mv = (uint32_t)ch_u24 * VREF_MV / ADC_MAX;

    uint32_t v_line_l_mv = v_adc_l_mv * DIV_K;
    uint32_t v_line_h_mv = v_adc_h_mv * DIV_K;
    uint32_t u24_mv      = 5000 * DIV_K;//v_adc_u_mv * DIV_K; // мы всегда в этой версии и далее подаём 5В.

    if (v_line_l_mv == 0u) {
        v_line_l_mv = 1u;
    }

    uint32_t r_line_ohm = 0u;
    if (v_line_h_mv > v_line_l_mv) {
        r_line_ohm = R0_OHM * (v_line_h_mv - v_line_l_mv) / v_line_l_mv;
    }

    int32_t k_scaled = -346;
    k_scaled += (int32_t)(673u * (u24_mv / 10u)) / 1000;
    if (k_scaled < 300)  k_scaled = 300;
    if (k_scaled > 1500) k_scaled = 1500;

    uint32_t r_corr = (uint32_t)((r_line_ohm * (uint32_t)k_scaled + 500u) / 1000u);
    DeviceLimitSwitchConfig *cfg = reinterpret_cast<DeviceLimitSwitchConfig*>(g_cfg.Devices[1].reserv);
    uint8_t normal_closed = (cfg != nullptr && cfg->normal_closed) ? 1u : 0u;
    uint16_t r_for_lib = App_MapLswitchResistanceForLib(r_corr, normal_closed);
    g_limit2.SetAdcValues(r_for_lib, (uint16_t)ch_h);
}

void App_Timer1ms(void)
{
    static uint16_t led_cnt = 0u;
    static uint16_t status_cnt = 0u;
    uint32_t now = HAL_GetTick();

    if (status_cnt < 1000u) {
        status_cnt++;
    } else {
        status_cnt = 0u;
        uint8_t status_data[7] = {0};
        status_data[0] = (uint8_t)(now / 1000u);
        status_data[1] = 0u;
        status_data[2] = 0u;
        status_data[3] = 0u;
        status_data[4] = (uint8_t)(CAN1_Active | (CAN2_Active << 1));

        const uint32_t VREF_MV = 3300u;
        const uint32_t ADC_MAX = 4095u;
        const uint32_t DIV_K   = 10u;
        uint32_t raw_u24 = ADC_GetU24Filtered();
        uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
        uint32_t u24_mv   = v_adc_mv * DIV_K;
        uint32_t code_1v = u24_mv / 1000u;
        if (code_1v > 255u) {
            code_1v = 255u;
        }
        status_data[5] = (uint8_t)code_1v;
        status_data[6] = App_GetCanStateMask();
        SendMessage(0, 0, status_data, SEND_NOW, BUS_CAN12);

        uint8_t pos_data[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};
        SendMessage(0, ServiceCmd_PositionDevice, pos_data, SEND_NOW, BUS_CAN12);
    }

    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    for (uint8_t i = 0; i < NUM_DEV_IN_MCU; i++) {
        if (!g_extinguish_armed[i]) {
            continue;
        }
        if ((int32_t)(now - g_extinguish_deadline_ms[i]) >= 0) {
            g_extinguish_armed[i] = 0u;
            uint8_t params[7] = {0};
            if (i == 2u) {
                g_igniter.CommandCB(10, params);
            }
        }
    }

    App_UpdateCanActivity();

    if (!g_igniter.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniterFiltered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter.UpdateLineFromAdcMv(mv);
    }

    g_limit1.Timer1ms();
    g_limit2.Timer1ms();
    g_igniter.Timer1ms();

    BackendProcess();

    uint16_t pwm = g_igniter.GetPwm();
    extern TIM_HandleTypeDef htim4;
    if (pwm > 0u) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
    } else {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    }
}

} /* extern "C" */

