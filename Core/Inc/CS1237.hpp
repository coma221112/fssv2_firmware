/*
 * CS1237.hpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_CS1237_HPP_
#define INC_CS1237_HPP_

#include "EXTIManager.hpp"
#include "GPIOHelper.hpp"
#include "Filters.hpp"
#include "Util.hpp"

class CS1237 : public EXTI_Observer {
public:
    GPIOPin DOUT;
    GPIOPin SCK;

    DEMA ema = DEMA(20);
    SMA<3> sma = SMA<3>();
    FastNotch notch50 = FastNotch(1280, 50, 1);
    volatile uint16_t ISRTime = 0;
    volatile int32_t ADCdata = 0;
    volatile int32_t filtered = 0;
    volatile int8_t configState = 0; // 对应原本的 state_g_config
    volatile bool configGood = false;
    uint8_t sampleCounter = 0;
    volatile bool readyRead = false;
    static inline uint8_t targetConfig = 0b01111100;
    static constexpr inline uint8_t sampleInterval = 1;

    CS1237() = delete;

    CS1237(GPIOPin sck, GPIOPin dout) : DOUT(dout), SCK(sck) {
        SCK = 0;
        EXTI_Manager::registerPin(DOUT.pin_, this);
    }

//    inline operator uint32_t() const{
//    	return ADCdata;
//    }
    void onExternalInterrupt() override {
    	if (++sampleCounter % sampleInterval != 0) return;

    	uint16_t start = DWT_GetUs();

		ADCdata = ADCreadImmediately();
		syncConfig();
		filtered = notch50.update(ADCdata);
		filtered = sma.update(filtered);

		uint32_t pinMask = DOUT.pin_;
		__HAL_GPIO_EXTI_CLEAR_IT(pinMask);

		ISRTime = DWT_GetUs() - start;
    }


    inline void syncConfig() {
        const uint8_t mask = 0b11111111;
        switch (configState) {
            case 0: { // 检查配置
                uint8_t current = ReadByte();
                if (((current ^ targetConfig) & mask) == 0) {
                    configState = -1; // 配置一致
                    configGood = true;
                } else {
                    configState = 1;  // 需要写入
                }
                break;
            }
            case 1: { // 写入配置
                WriteByte(targetConfig);
                configState = 0;
                break;
            }
            case -1: // 已就绪，不做动作
            default:
                break;
        }
    }

    inline int32_t ADCreadImmediately() {
//        ADCdata = 0;
//        delay_us(2);
    	int32_t raw = 0;
        // DOUT 切换为输入模式（取决于你的 GPIOPin 类实现，通常 DOUT 此时已是输入）
        for (int i = 0; i < 24; i++) {
            SCL_H(); delay_us();
            raw = (raw << 1) | DOUT;
            SCL_L(); delay_us();

        }
        // 补 3 个脉冲
        for (int i = 0; i < 3; i++) {
            SCL_H(); delay_us();
            SCL_L(); delay_us();
        }
        // 符号扩展：24位转32位有符号
        if (raw & 0x800000) raw |= 0xFF000000;
        ADCdata = raw;
        return raw;
    }

    // --- 寄存器读写逻辑 (ReadByte/WriteByte) ---
    // 注意：ReadByte/WriteByte 内部涉及模式切换，确保 GPIOPin 支持 setMode

    inline uint8_t ReadByte() {
        DOUT.setMode(GPIOMode::OutputPP);
        one_bit(); one_bit(); // 28, 29
        uint8_t cmd = 0x56;   // 读命令
        for (int i = 0; i < 7; i++) {
            (cmd & (0x40 >> i)) ? SDA_H() : SDA_L();
            one_bit();
        }
        one_bit(); // 37 (ACK位等)

        DOUT.setMode(GPIOMode::Input);
        uint8_t res = 0;
        for (int i = 0; i < 8; i++) {
            one_bit();
            res = (res << 1) | DOUT;
        }
        one_bit(); // 46
        return res;
    }

    inline void WriteByte(uint8_t data) {
        DOUT.setMode(GPIOMode::OutputPP);
        one_bit(); one_bit(); // 28, 29
        uint8_t cmd = 0x65;   // 写命令
        for (int i = 0; i < 7; i++) {
            (cmd & (0x40 >> i)) ? SDA_H() : SDA_L();
            one_bit();
        }
        one_bit(); // 37

        for (int i = 0; i < 8; i++) {
            (data & (0x80 >> i)) ? SDA_H() : SDA_L();
            one_bit();
        }
        one_bit(); // 46
        DOUT.setMode(GPIOMode::Input);
    }

    // --- 基础时序 ---
    inline void SCL_H() { SCK = 1; }
    inline void SCL_L() { SCK = 0; }
    inline void SDA_H() { DOUT = 1; }
    inline void SDA_L() { DOUT = 0; }
    inline void one_bit() {
        SCL_H(); delay_us();
        SCL_L(); delay_us();
    }


};



#endif /* INC_CS1237_HPP_ */
