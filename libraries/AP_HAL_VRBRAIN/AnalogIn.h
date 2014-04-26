/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_VRBRAIN_ANALOGIN_H__
#define __AP_HAL_VRBRAIN_ANALOGIN_H__

#include <AP_HAL_VRBRAIN.h>
#include <pthread.h>
#include <uORB/uORB.h>

#define VRBRAIN_ANALOG_MAX_CHANNELS 16


#if  defined(CONFIG_ARCH_BOARD_VRBRAIN_V4)
// these are virtual pins that read from the ORB
#define VRBRAIN_ANALOG_ORB_BATTERY_VOLTAGE_PIN     10
#define VRBRAIN_ANALOG_ORB_BATTERY_CURRENT_PIN     11
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V5)
#define VRBRAIN_ANALOG_ORB_BATTERY_VOLTAGE_PIN     10
#define VRBRAIN_ANALOG_ORB_BATTERY_CURRENT_PIN     11
#elif defined(CONFIG_ARCH_BOARD_VRHERO_V1)
#define VRBRAIN_ANALOG_ORB_BATTERY_VOLTAGE_PIN     10
#define VRBRAIN_ANALOG_ORB_BATTERY_CURRENT_PIN     11
#endif

class VRBRAIN::VRBRAINAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class VRBRAIN::VRBRAINAnalogIn;
    VRBRAINAnalogSource(int16_t pin, float initial_value);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();

    // stop pins not implemented on VRBRAIN yet
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    // what pin it is attached to
    int16_t _pin;

    // what value it has
    float _value;
    float _value_ratiometric;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    float _sum_ratiometric;
    void _add_value(float v, float vcc5V);
    float _pin_scaler();
};

class VRBRAIN::VRBRAINAnalogIn : public AP_HAL::AnalogIn {
public:
    VRBRAINAnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t pin);
    void _timer_tick(void);
    float board_voltage(void) { return _board_voltage; }
    float servorail_voltage(void) { return _servorail_voltage; }
    uint16_t power_status_flags(void) { return _power_flags; }

private:
    int _adc_fd;
    int _battery_handle;
    int _servorail_handle;
    int _system_power_handle;
    uint64_t _battery_timestamp;
    uint64_t _servorail_timestamp;
    VRBRAIN::VRBRAINAnalogSource* _channels[VRBRAIN_ANALOG_MAX_CHANNELS];
    uint32_t _last_run;
    float _board_voltage;
    float _servorail_voltage;
    uint16_t _power_flags;
};
#endif // __AP_HAL_VRBRAIN_ANALOGIN_H__
