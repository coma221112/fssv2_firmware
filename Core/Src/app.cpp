/*
 * main.cpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#include "HardwareConfig.hpp"
#include "usbd_customhid.h"
#include "usb_device.h"
#include "ZeroTracker.hpp"
#include "JoystickProtocol.hpp"
#include "Filters.hpp"
#include "Util.hpp"
#include "Persistence.hpp"

#include "cmath"

extern USBD_HandleTypeDef hUsbDeviceFS;



#define DEBOUNCE_MS 10
#define NUM_BUTTONS 22
typedef struct {
    uint8_t last_raw_state;   // 上一次扫描到的原始电平
    uint8_t stable_state;     // 经过消抖后的稳定状态
    uint32_t last_change_ms;  // 最后一次电平发生变化的时间点
} Button_Typedef;
Button_Typedef buttons[NUM_BUTTONS] = {0};
uint8_t button_stable_state[NUM_BUTTONS] = {0};
uint8_t raw_pins[NUM_BUTTONS];

inline int32_t ApplyDeadzone(int32_t Value, uint16_t Threshold) {
    if (Value > (int32_t)Threshold) return Value - Threshold;
    if (Value < -(int32_t)Threshold) return Value + Threshold;
    return 0;
}

inline float Clamp(float raw_val){
	return fmaxf(-1.0f, fminf(1.0f, raw_val));
}

uint8_t HID_SendReport_Safe(USBD_HandleTypeDef *pdev,
                                        uint8_t *report,
                                        uint16_t len,
                                        uint8_t priority)  // 0=normal, 1=high priority
{
    USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;

    if (priority) {
        // High priority - wait up to 50ms
        uint32_t start = HAL_GetTick();
        while (hhid->state == CUSTOM_HID_BUSY) {
            if ((HAL_GetTick() - start) > 50) {
                return USBD_BUSY;
            }
        }
        return USBD_CUSTOM_HID_SendReport(pdev, report, len);
    } else {
        // Normal priority - drop if busy
        if (hhid->state == CUSTOM_HID_BUSY) {
            return USBD_BUSY;
        }
        return USBD_CUSTOM_HID_SendReport(pdev, report, len);
    }
}

int32_t v0,v1,v2;
int32_t dc0,dc1,dc2;
int32_t dv0,dv1,dv2;
SWV<256> swv0;//,swv1,swv2;
ZeroTracker zt0,zt1,zt2;
EMA swvEma(10240);
float swvRaw;
float swv;
int32_t lastF = 0;
typedef struct{
	int8_t f0;
	int32_t f2;
	int16_t f1;
}__attribute__((packed)) aligntest_t;
aligntest_t at;
volatile uint32_t tear_count = 0;
extern "C" void RealMain(){
	while(!(sg0.configGood&&sg1.configGood&&sg2.configGood));
	zt0.calibrate(sg0.ADCdata);
	zt1.calibrate(sg1.ADCdata);
	zt2.calibrate(sg2.ADCdata);

	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

	Persistence::Read(0, joystickConfig);



    uint32_t lastL = DWT_GetUs();
    int8_t loopCount = 0;
	while(true){
//		if(loopCount%2==0){
//			at.f1=0;
//		}
//		else{
//			at.f1=-1;
//		}
//		if(at.f1==255||at.f1==-256){
//			tear_count++;
//		}
		loopCount++;
		uint32_t loopTime = DWT_GetUs()-lastL;
		lastL = DWT_GetUs();

		v0=sg0.filtered;
		v1=sg1.filtered;
		v2=sg2.filtered;
		dv0=v0-dc0;
		dv1=v1-dc1;
		dv2=v2-dc2;

		if(DWT_GetUs() - lastF > 10000){
			lastF = DWT_GetUs();
			swvRaw = swv0.update(abs(v0-v1) + abs(v1-v2) + abs(v2-v0));
			swv = swvRaw;//swvEma.update(swvRaw);
			if(swv < 400000){
				dc0=zt0.update(v0);
				dc1=zt1.update(v1);
				dc2=zt2.update(v2);
				PC13 = 0;
			}
			else{
				PC13 = 1;
			}
		}

		float maxRange = joystickConfig.maxRange;
		constexpr float axisRange = 32767;
		float scale = axisRange / maxRange;

		float fx,fy,fz;
		fx = Clamp((dv2 - dv1) / maxRange * 0.866f);
		fy = Clamp((0.5f*(dv1 + dv2) - dv0) / maxRange);
		fz = Clamp(((dv0 + dv1 + dv2) / maxRange / 3.f));

		float magnitude = sqrtf(fx * fx + fy * fy);
		if (magnitude < joystickConfig.deadzone) {
		    fx = 0;
		    fy = 0;
		} else {
		    float factor = (magnitude - joystickConfig.deadzone) / (1 - joystickConfig.deadzone);
		    if (factor > 1.0f) factor = 1.0f;
		    fx = (fx / magnitude) * factor;
		    fy = (fy / magnitude) * factor;
		}

		// Set axes
		auto& report = joystickReport;
		report.x = fx * axisRange;
		report.y = fy * axisRange;
		report.z = fz * axisRange;
		report.rx = dv0 * scale;
		report.ry = dv1 * scale;
		report.rz = dv2 * scale;
		report.lt = loopTime;
		report.swv = swv;//+swv1.update(report.y);//+swv2.update(dv2);


		static GPIOPin buttonPins[21] = {
		    PB11,  // 1  <- 20
		    PA15,  // 2  <- 18
		    PB12,  // 3  <- 6
		    PB13,  // 4  <- 14
		    PB15,  // 5  <- 7
		    PB5,   // 6  <- 10
		    PB3,   // 7  <- 4
		    PB6,   // 8  <- 8
		    PB14,  // 9  <- 9
		    PB7,   // 10 <- 19
		    PA6,   // 11 <- 11
		    PA10,  // 12 <- 17
		    PA7,   // 13 <- 13
		    PA9,   // 14 <- 12
		    PB9,   // 15 <- 15
		    PB0,   // 16 <- 16
		    PA8,   // 17 <- 5
		    PB4,   // 18 <- 2
		    PB8,   // 19 <- 3
		    PB10,  // 20 <- 21
		    PB1    // 21 <- 1
		};
//		20 18 6 14 7 10 4 8 9 19 11 17 13 12 15 16 5 2 3 21 1
		for(int i=0;i<21;i++){
			raw_pins[i] = !buttonPins[i];
		}
		uint32_t now = HAL_GetTick(); // 使用毫秒级时间戳

		for (int i = 0; i < NUM_BUTTONS; i++) {
		    uint8_t current_raw = raw_pins[i];

		    // 如果当前读到的引脚状态和上一次不一样
		    if (current_raw != buttons[i].last_raw_state) {
		        buttons[i].last_change_ms = now; // 重置计时器
		    }

		    // 如果当前电平已经稳定维持了超过 DEBOUNCE_MS
		    if ((now - buttons[i].last_change_ms) >= DEBOUNCE_MS) {
		        // 更新稳定状态
		        buttons[i].stable_state = current_raw;
		        button_stable_state[i] = buttons[i].stable_state;
		    }

		    // 更新“上一次”的原始电平记录
		    buttons[i].last_raw_state = current_raw;
		}

		for(int i=0;i<21;i++){
			SetButton(report, button_stable_state[i], i);
		}

		static uint32_t pressStartTime = 0;
		if (GetButton(report,20)) {
		    // 如果是刚按下（计时器还没启动），则记录当前系统时间
		    if (pressStartTime == 0) {
		        pressStartTime = HAL_GetTick();
		    }

		    // 判断按下持续时间是否超过 5000 毫秒
		    if (HAL_GetTick() - pressStartTime > joystickConfig.calibLongPressMs) {
		        zt0.calibrate(sg0.ADCdata);
		        zt1.calibrate(sg1.ADCdata);
		        zt2.calibrate(sg2.ADCdata);
		    }
		} else {
		    // 只要按键松开（电平变回 1），立即清零计时器
		    pressStartTime = 0;
		}



		if (configNeedsSending) {
		    if (HID_SendReport_Safe(&hUsbDeviceFS, (uint8_t*)&joystickConfig, sizeof(joystickConfig), 1) == USBD_OK) {
		    	configNeedsSending = false;
		    }
		}
		else{
			HID_SendReport_Safe(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report),0);
		}
		if(configNeedsSaving){
			Persistence::Save(0, joystickConfig);
			configNeedsSaving = false;
		}
	}
}

