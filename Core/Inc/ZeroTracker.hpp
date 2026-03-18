/*
 * ZeroTracker.hpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_ZEROTRACKER_HPP_
#define INC_ZEROTRACKER_HPP_

#include <stdint.h>

#include <stdint.h>

class ZeroTracker {
public:
    ZeroTracker(int32_t initialZero = 0)
        : zero(initialZero), deadzone(2400000), fastzone(10000), slowzone(2400000)
        , slowRate(1), fastRate(1){}

    void setDeadzone(int32_t threshold) { deadzone = threshold; }
    void setFastzone(int32_t threshold) { fastzone = threshold; }
    void setSlowzone(int32_t threshold) { slowzone = threshold; }
    void setSlowRate(int8_t rate) { slowRate = rate; }
    void setFastRate(int8_t rate) { fastRate = rate; }

    int32_t update(int32_t rawValue) {
		int32_t dev = rawValue - zero;
		int32_t absDev = (dev < 0) ? -dev : dev;
		float step = 0;
		if (absDev < deadzone) {
			if (absDev <= fastzone) {
				step = fastRate;
			} else if (absDev <= slowzone) {
				step = slowRate;
			}

			if (dev > 0) {
				zero += step;
			} else {
				zero -= step;
			}
		}
        return zero;
    }

    int32_t getCorrected(int32_t rawValue) const { return rawValue - zero; }
    int32_t getZero() const { return zero; }
    void calibrate(int32_t rawValue) { zero = rawValue; }

private:
    int32_t zero;
    int32_t deadzone;   // No movement zone
    int32_t fastzone;   // Fast tracking zone
    int32_t slowzone;   // Slow tracking zone
    int32_t slowRate;
    int32_t fastRate;
};



#endif /* INC_ZEROTRACKER_HPP_ */
