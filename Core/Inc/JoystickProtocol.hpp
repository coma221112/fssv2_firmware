/*
 * JoystickProtocol.hpp
 *
 *  Created on: Jan 25, 2026
 *      Author: Drac
 */

#ifndef INC_JOYSTICKPROTOCOL_HPP_
#define INC_JOYSTICKPROTOCOL_HPP_

//20B
typedef struct {
//	uint8_t pad;
    uint8_t buttons[4];
    int16_t x, y, z, rx, ry, rz, lt, swv;
//    uint8_t  padding[44];
} __attribute__((packed)) USB_HID_JoystickReport_t;

typedef struct{
	uint32_t magic;

	uint32_t maxRange;
    uint32_t calibLongPressMs;
    float deadzone;
    uint8_t  padding[4];
} __attribute__((packed)) JoystickConfig_t;

#ifdef __cplusplus
  inline USB_HID_JoystickReport_t joystickReport;
  inline void SetButton(USB_HID_JoystickReport_t& report, bool set, uint8_t idx){
  	if (idx >= 32) return;
  	if (set) {
  		report.buttons[idx >> 3] |= (1U << (idx & 0x07));
  	} else {
  		report.buttons[idx >> 3] &= ~(1U << (idx & 0x07));
  	}
  }
  inline bool GetButton(USB_HID_JoystickReport_t& report, uint8_t idx){
	if (idx >= 32) return 0;
	return report.buttons[idx >> 3] & (1U << (idx & 0x07));
  }
#else
  extern USB_HID_JoystickReport_t joystickReport;
#endif





#ifdef __cplusplus
volatile inline JoystickConfig_t joystickConfig={999, 800000, 3000, 0};
#else
extern volatile JoystickConfig_t joystickConfig;
#endif

#ifdef __cplusplus
volatile inline uint8_t configNeedsSending = 0;
volatile inline uint8_t configNeedsSaving = 0;
#else
extern volatile uint8_t configNeedsSending;
extern volatile uint8_t configNeedsSaving;
#endif

#endif /* INC_JOYSTICKPROTOCOL_HPP_ */
