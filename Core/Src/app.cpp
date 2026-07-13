#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_button.hpp"
#include "device_igniter.hpp"
#include "app_igniter_launch.hpp"
#include "device_lswitch.hpp"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#include <string.h>

#include "main.h"
#include "mku_cfg_flash.h"

extern "C" {
extern TIM_HandleTypeDef htim4;
}

extern DTS_HandleTypeDef hdts;

MKUCfg g_cfg;
MKUCfg g_saved_cfg;

/* 1: line1, 2: line2, 3: igniter */
static VDeviceButton g_button1(1);
static VDeviceLimitSwitch g_limit1(1);
static VDeviceButton g_button2(2);
static VDeviceLimitSwitch g_limit2(2);
static VDeviceIgniter g_igniter(3);

/* Пересчёт физических Ом K3 в шкалу порогов device_lib.
 * Опорные точки имитатора (NO):
 *   248 Ом  — норма       -> ~1000 Ом (Normal)
 *   1800 Ом — срабатывание -> ~120 Ом (Press)
 * NC: целевые значения на концах меняются местами.
 */
static uint16_t App_MapLswitchResistanceForLib(uint32_t raw_ohm, uint8_t normal_closed)
{
    const uint32_t RAW_NORMAL  = 248u;
    const uint32_t RAW_TRIGGER = 1800u;
    const uint32_t LIB_NORMAL  = 1000u;
    const uint32_t LIB_TRIGGER = 120u;

    const uint32_t dst_normal  = normal_closed ? LIB_TRIGGER : LIB_NORMAL;
    const uint32_t dst_trigger = normal_closed ? LIB_NORMAL : LIB_TRIGGER;

    if (raw_ohm <= RAW_NORMAL) {
        return (uint16_t)dst_normal;
    }
    if (raw_ohm >= RAW_TRIGGER) {
        return (uint16_t)dst_trigger;
    }

    const int32_t den = (int32_t)(RAW_TRIGGER - RAW_NORMAL);
    const int32_t d_dst = (int32_t)dst_trigger - (int32_t)dst_normal;
    int64_t num = (int64_t)((int32_t)raw_ohm - (int32_t)RAW_NORMAL) * (int64_t)d_dst;
    if (num >= 0) {
        num += (int64_t)(den / 2);
    } else {
        num -= (int64_t)(den / 2);
    }
    int64_t mapped_signed = (int64_t)(int32_t)dst_normal + (num / (int64_t)den);

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

static uint32_t ResetDelayms = 3000;
static uint8_t isReset = 0;

/* Слот 0/1: кнопка или концевик (VDtype 15/16), слот 2: спичка. VDtype==0 — отключён. */
static uint8_t App_IsSlotEnabled(uint8_t slot)
{
	if (slot >= NUM_DEV_IN_MCU) {
		return 0u;
	}
	return (g_cfg.VDtype[slot] != 0u) ? 1u : 0u;
}

static uint8_t App_IsButtonSlot(uint8_t slot)
{
	if (slot >= NUM_DEV_IN_MCU) {
		return 0u;
	}
	return (g_cfg.VDtype[slot] == DEVICE_BUTTON_TYPE) ? 1u : 0u;
}

static uint8_t App_IsLswitchSlot(uint8_t slot)
{
	if (slot >= NUM_DEV_IN_MCU) {
		return 0u;
	}
	return (g_cfg.VDtype[slot] == DEVICE_LSWITCH_TYPE) ? 1u : 0u;
}

static uint8_t App_IsLineSlotEnabled(uint8_t slot)
{
	return (App_IsButtonSlot(slot) || App_IsLswitchSlot(slot)) ? 1u : 0u;
}

static uint8_t App_IsIgniterSlotEnabled(uint8_t slot)
{
	if (slot >= NUM_DEV_IN_MCU) {
		return 0u;
	}
	return (g_cfg.VDtype[slot] == DEVICE_IGNITER_TYPE) ? 1u : 0u;
}

static uint8_t App_SlotBoardDType(uint8_t slot)
{
	if (!App_IsSlotEnabled(slot)) {
		return 0u;
	}
	return (uint8_t)(g_cfg.VDtype[slot] & 0xFFu);
}

static void App_RebuildBoardDevicesList(void)
{
	extern Device BoardDevicesList[];
	extern uint8_t nDevs;

	BoardDevicesList[0].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[0].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[0].l_adr  = g_cfg.UId.devId.l_adr;
	BoardDevicesList[0].d_type = DEVICE_MCU_K3;

	BoardDevicesList[1].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[1].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[1].l_adr  = 1;
	BoardDevicesList[1].d_type = App_SlotBoardDType(0);

	BoardDevicesList[2].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[2].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[2].l_adr  = 2;
	BoardDevicesList[2].d_type = App_SlotBoardDType(1);

	BoardDevicesList[3].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[3].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[3].l_adr  = 3;
	BoardDevicesList[3].d_type = App_IsIgniterSlotEnabled(2) ? DEVICE_IGNITER_TYPE : 0u;

	nDevs = 4;
}

static uint8_t App_IsBoardDevActive(uint8_t dnum)
{
	if (dnum == 1u) {
		return App_IsLineSlotEnabled(0);
	}
	if (dnum == 2u) {
		return App_IsLineSlotEnabled(1);
	}
	if (dnum == 3u) {
		return App_IsIgniterSlotEnabled(2);
	}
	return 1u;
}

static void App_StopDisabledChannels(void)
{
	if (!App_IsIgniterSlotEnabled(2)) {
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
		g_extinguish_armed[2] = 0u;
	}
}

/* callback статуса: отправляем его через CAN по протоколу backend */
static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters)
{
    if (!App_IsBoardDevActive(DNum)) {
        return;
    }
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    SendMessage(DNum, Code, data, 0, BUS_CAN12);
}

static void App_InitLineDevice(VDeviceButton &button, VDeviceLimitSwitch &lswitch,
                               void (*set_res)(void), void (*set_max)(void))
{
	button.VDeviceSetStatus = VDeviceSetStatus;
	button.VDeviceSaveCfg   = SaveConfig;
	button.DPT_SetResMeasureMode = set_res;
	button.DPT_SetMaxMeasureMode = set_max;

	lswitch.VDeviceSetStatus = VDeviceSetStatus;
	lswitch.VDeviceSaveCfg   = SaveConfig;
	lswitch.DPT_SetResMeasureMode = set_res;
	lswitch.DPT_SetMaxMeasureMode = set_max;
}

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

static void App_ArmIgniterSlot(uint8_t ign_slot, uint8_t zd, uint8_t md)
{
    uint32_t delay_ms = ((uint32_t)zd + (uint32_t)md) * 1000u;

    g_extinguish_deadline_ms[ign_slot] = HAL_GetTick() + delay_ms;
    g_extinguish_armed[ign_slot] = 1u;
    SetReplyStartExtinguishment((uint8_t)(ign_slot + 1u));
}

static void App_ArmIgniterSlotWithLaunch(uint8_t ign_slot, uint8_t launch_type, uint8_t cmd_zd, uint8_t cmd_md)
{
    uint8_t zd = 0u;
    uint8_t md = 0u;

    Backend_ResolveIgniterStartDelays(launch_type, cmd_zd, cmd_md, ign_slot,
                                      g_cfg.zone_delay, g_cfg.module_delay, NUM_DEV_IN_MCU,
                                      &zd, &md);
    App_ArmIgniterSlot(ign_slot, zd, md);
}

static void App_ArmAllIgnitersWithLaunch(uint8_t launch_type, uint8_t cmd_zd, uint8_t cmd_md)
{
    for (uint8_t i = 0u; i < NUM_DEV_IN_MCU; i++) {
        if (g_cfg.VDtype[i] == DEVICE_IGNITER_TYPE) {
            App_ArmIgniterSlotWithLaunch(i, launch_type, cmd_zd, cmd_md);
        }
    }
}

extern "C" void VDeviceButton_OnStartExtinguishment(uint8_t zone,
                                                  uint8_t zone_delay_s,
                                                  uint8_t module_delay_s,
                                                  uint8_t launch_type)
{
    if (!Backend_StartExtinguishZoneParamMatches(zone, g_cfg.UId.devId.zone)) {
        return;
    }
    App_ArmAllIgnitersWithLaunch(launch_type, zone_delay_s, module_delay_s);
}

extern "C" void RcvStartExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    if (is_mine == 0u) {
        return;
    }

    uint8_t zd = MsgData[2];
    uint8_t md = MsgData[3];
    uint8_t launch_type = MsgData[4];

    if (Backend_IsIgniterBroadcastId(MsgID)) {
        if (!Backend_StartExtinguishZoneMatches(MsgID, MsgData[1], g_cfg.UId.devId.zone)) {
            return;
        }
        App_ArmAllIgnitersWithLaunch(launch_type, zd, md);
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    App_ArmIgniterSlotWithLaunch((uint8_t)ign_slot, launch_type, zd, md);
}

static uint8_t App_IsIgniterSlot(uint8_t slot, void *ctx)
{
    (void)ctx;
    if (slot >= NUM_DEV_IN_MCU) {
        return 0u;
    }
    return (g_cfg.VDtype[slot] == DEVICE_IGNITER_TYPE) ? 1u : 0u;
}

static uint8_t App_IsIgniterBurnRunning(uint8_t slot, void *ctx)
{
    (void)ctx;
    if (slot == 2u) {
        return g_igniter.IsBurnRunning() ? 1u : 0u;
    }
    return 0u;
}

static void App_FireIgniterSlot(uint8_t slot, void *ctx)
{
    (void)ctx;
    g_extinguish_armed[slot] = 0u;
    uint8_t params[7] = {0};
    if (slot == 2u) {
        g_igniter.CommandCB(10, params);
    }
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



void SetHAdr(uint8_t h_adr)
{
    g_cfg.UId.devId.h_adr = h_adr;
    App_RebuildBoardDevicesList();
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

    g_cfg.VDtype[0] = DEVICE_BUTTON_TYPE;
    g_cfg.VDtype[1] = DEVICE_LSWITCH_TYPE;
    g_cfg.VDtype[2] = DEVICE_IGNITER_TYPE;

    g_cfg.zone_delay = 5u;
    g_cfg.module_delay[0] = 0u;
    g_cfg.module_delay[1] = 0u;
    g_cfg.module_delay[2] = 5u;

    DeviceButtonConfig *btn1 = reinterpret_cast<DeviceButtonConfig*>(g_cfg.Devices[0].reserv);
    DeviceDPTConfig *l2 = reinterpret_cast<DeviceDPTConfig*>(g_cfg.Devices[1].reserv);
    memset(btn1, 0, sizeof(DeviceButtonConfig));
    memset(l2, 0, sizeof(DeviceDPTConfig));

    btn1->mode                  = 2; /* кнопка */
    btn1->use_max               = 0;
    btn1->max_fire_threshold_c  = 60;
    btn1->state_change_delay_ms = 100;
    btn1->button_kind           = DeviceButtonKind_StartSP;

    l2->mode                  = 1; /* концевик */
    l2->use_max               = 0;
    l2->max_fire_threshold_c  = 60;
    l2->state_change_delay_ms = 100;

    DeviceIgniterConfig *ign = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[2].reserv);
    memset(ign, 0, sizeof(DeviceIgniterConfig));
    ign->disable_sc_check     = 0u;
    ign->threshold_break_low  = 100;
    ign->threshold_break_high = 1000;
    ign->burn_retry_count     = 0u;
}

void ResetMCU(void)
{
    isReset = 1;
}

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
    case 1:
        if (App_IsButtonSlot(0)) {
            g_button1.CommandCB(Command, Parameters);
        } else if (App_IsLswitchSlot(0)) {
            g_limit1.CommandCB(Command, Parameters);
        }
        break;
    case 2:
        if (App_IsButtonSlot(1)) {
            g_button2.CommandCB(Command, Parameters);
        } else if (App_IsLswitchSlot(1)) {
            g_limit2.CommandCB(Command, Parameters);
        }
        break;
    case 3:
        if (App_IsIgniterSlotEnabled(2)) {
            g_igniter.CommandCB(Command, Parameters);
        }
        break;
    default: break;
    }
}

void AplyConfig(void)
{
    App_RebuildBoardDevicesList();
    App_StopDisabledChannels();
}

void ListenerCommandCB(uint32_t MsgID, uint8_t *MsgData)
{
    (void)MsgID;
    (void)MsgData;
}

void App_Init(void)
{
    if (!FlashReadConfig(&g_cfg)) {
        DefaultConfig();
        SaveConfig();
    }
    g_saved_cfg = g_cfg;
    SetConfigPtr(reinterpret_cast<uint8_t *>(&g_saved_cfg),
                 reinterpret_cast<uint8_t *>(&g_cfg));

    g_button1.DeviceInit(&g_cfg.Devices[0]);
    App_InitLineDevice(g_button1, g_limit1, App_DPT1_SetResMeasureMode, App_DPT1_SetMaxMeasureMode);
    g_button1.Init();
    g_limit1.DeviceInit(&g_cfg.Devices[0]);
    g_limit1.Init();

    g_button2.DeviceInit(&g_cfg.Devices[1]);
    App_InitLineDevice(g_button2, g_limit2, App_DPT2_SetResMeasureMode, App_DPT2_SetMaxMeasureMode);
    g_button2.Init();
    g_limit2.DeviceInit(&g_cfg.Devices[1]);
    g_limit2.Init();

    g_igniter.DeviceInit(&g_cfg.Devices[2]);
    g_igniter.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter.VDeviceSaveCfg   = SaveConfig;
    g_igniter.Init();

    App_RebuildBoardDevicesList();

    if (App_IsLineSlotEnabled(0)) {
        App_DPT1_SetResMeasureMode();
    }
    if (App_IsLineSlotEnabled(1)) {
        App_DPT2_SetResMeasureMode();
    }

    extern bool isListener;
    isListener = true;
}

void App_SetLimit1AdcValues(uint16_t ch_l, uint16_t ch_h, uint16_t ch_u24)
{
    if (!App_IsLineSlotEnabled(0)) {
        return;
    }

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
    uint8_t normal_closed = 0u;
    if (App_IsButtonSlot(0)) {
        DeviceButtonConfig *cfg = reinterpret_cast<DeviceButtonConfig*>(g_cfg.Devices[0].reserv);
        normal_closed = (cfg != nullptr && cfg->normal_closed) ? 1u : 0u;
    } else if (App_IsLswitchSlot(0)) {
        DeviceLimitSwitchConfig *cfg = reinterpret_cast<DeviceLimitSwitchConfig*>(g_cfg.Devices[0].reserv);
        normal_closed = (cfg != nullptr && cfg->normal_closed) ? 1u : 0u;
    }
    uint16_t r_for_lib = App_MapLswitchResistanceForLib(r_corr, normal_closed);
    if (App_IsButtonSlot(0)) {
        g_button1.SetAdcValues(r_for_lib, (uint16_t)ch_h);
    } else if (App_IsLswitchSlot(0)) {
        g_limit1.SetAdcValues(r_for_lib, (uint16_t)ch_h);
    }
}

void App_SetLimit2AdcValues(uint16_t ch_l, uint16_t ch_h, uint16_t ch_u24)
{
    if (!App_IsLineSlotEnabled(1)) {
        return;
    }

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
    uint8_t normal_closed = 0u;
    if (App_IsButtonSlot(1)) {
        DeviceButtonConfig *cfg = reinterpret_cast<DeviceButtonConfig*>(g_cfg.Devices[1].reserv);
        normal_closed = (cfg != nullptr && cfg->normal_closed) ? 1u : 0u;
    } else if (App_IsLswitchSlot(1)) {
        DeviceLimitSwitchConfig *cfg = reinterpret_cast<DeviceLimitSwitchConfig*>(g_cfg.Devices[1].reserv);
        normal_closed = (cfg != nullptr && cfg->normal_closed) ? 1u : 0u;
    }
    uint16_t r_for_lib = App_MapLswitchResistanceForLib(r_corr, normal_closed);
    if (App_IsButtonSlot(1)) {
        g_button2.SetAdcValues(r_for_lib, (uint16_t)ch_h);
    } else if (App_IsLswitchSlot(1)) {
        g_limit2.SetAdcValues(r_for_lib, (uint16_t)ch_h);
    }
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

        int32_t temperature;
        if(HAL_DTS_GetTemperature(&hdts, &temperature)!= HAL_OK)
        {
            /* DTS GetTemperature Error */
        }

        if(temperature > 128) temperature = 128;
        if(temperature < -128) temperature = -128;

        uint8_t temp = (uint8_t)temperature;

        uint8_t status_data[7] = {0};
        status_data[0] = (uint8_t)(now / 1000u);
        status_data[1] = temp;
        status_data[2] = 0u;
        status_data[3] = 0u;
        status_data[4] = (uint8_t)(CAN1_Active | (CAN2_Active << 1));

        const uint32_t VREF_MV = 3300u;
        const uint32_t ADC_MAX = 4095u;
        uint32_t raw_u24 = ADC_GetU24Filtered();
        uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
        uint32_t u24_mv = v_adc_mv;
        uint32_t code_1v = (u24_mv + 500) / 1000u;
        if (code_1v > 255u) code_1v = 255u;
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

    App_UpdateCanActivity();

    // задержка софт-рестарта. нужно чтобы усройство успело широковещательную переслать команду дальше
    if (isReset) {
        ResetDelayms--;
        if (ResetDelayms == 0u) {
            NVIC_SystemReset();
        }
    }

    if (App_IsIgniterSlotEnabled(2) && !g_igniter.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniterFiltered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter.UpdateLineFromAdcMv(mv);
    }

    if (App_IsButtonSlot(0)) {
        g_button1.Timer1ms();
    } else if (App_IsLswitchSlot(0)) {
        g_limit1.Timer1ms();
    }
    if (App_IsButtonSlot(1)) {
        g_button2.Timer1ms();
    } else if (App_IsLswitchSlot(1)) {
        g_limit2.Timer1ms();
    }

    BackendProcess();

    AppIgniter_RunSequentialScheduler(NUM_DEV_IN_MCU, now, g_extinguish_deadline_ms, g_extinguish_armed,
                                      nullptr, App_IsIgniterSlot, App_IsIgniterBurnRunning,
                                      App_FireIgniterSlot, nullptr);

    if (App_IsIgniterSlotEnabled(2)) {
        g_igniter.Timer1ms();
    }

    if (App_IsIgniterSlotEnabled(2)) {
        uint16_t pwm = g_igniter.GetPwm();
        if (pwm > 0u) {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
        } else {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
        }
    } else {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    }
}

} /* extern "C" */

