// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_tracker()
{
    hal.uartA->begin(SERIAL0_BAUD, 128, SERIAL_BUFSIZE);

    // gps port
    hal.uartB->begin(38400, 256, 16);

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    hal.util->available_memory());

    // Check the EEPROM format version before loading any parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // reset the uartA baud rate after parameter load
    hal.uartA->begin(map_baudrate(g.serial0_baud, SERIAL0_BAUD));

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // init the GCS
    gcs[0].init(hal.uartA);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial1_baud, SERIAL1_BAUD),
                     128, SERIAL1_BUFSIZE);
    gcs[1].init(hal.uartC);

    mavlink_system.sysid = g.sysid_this_mav;

    if (g.compass_enabled==true) {
        if (!compass.init() || !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }

    // GPS Initialization
    gps.init(NULL);

    mavlink_system.compid = 4;
    mavlink_system.type = MAV_TYPE_ANTENNA_TRACKER;

    ahrs.init();
    ahrs.set_fly_forward(false);

    ins.init(AP_InertialSensor::WARM_START, ins_sample_rate);
    ahrs.reset();

    init_barometer();

    hal.uartA->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

    // setup antenna control PWM channels
    channel_yaw.set_angle(18000); // Yaw is expected to drive antenna azimuth -180-0-180
    channel_pitch.set_angle(9000); // Pitch is expected to drive elevation -90-0-90

    channel_yaw.output_trim();
    channel_pitch.output_trim();

    channel_yaw.calc_pwm();
    channel_pitch.calc_pwm();

    // use given start positions - useful for indoor testing, and
    // while waiting for GPS lock
    current_loc.lat = g.start_latitude * 1.0e7f;
    current_loc.lng = g.start_longitude * 1.0e7f;

    // see if EEPROM has a default location as well
    if (current_loc.lat == 0 && current_loc.lng == 0) {
        get_home_eeprom(current_loc);
    }

    gcs_send_text_P(SEVERITY_LOW,PSTR("\nReady to track."));
    hal.scheduler->delay(1000); // Why????

    set_mode(AUTO); // tracking

    if (g.startup_delay > 0) {
        // arm servos with trim value to allow them to start up (required
        // for some servos)
        prepare_servos();
    }
}

// Level the tracker by calibrating the INS
// Requires that the tracker be physically 'level' and horizontal
static void calibrate_ins()
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move tracker"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, ins_sample_rate);
    ins.init_accel();
    ahrs.set_trim(Vector3f(0, 0, 0));
    ahrs.reset();
    init_barometer();
}

// updates the status of the notify objects
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

/*
 *  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    cliSerial->println_P(PSTR("Invalid baudrate"));
    return default_baud;
}

/*
  fetch HOME from EEPROM
*/
static bool get_home_eeprom(struct Location &loc)
{
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (g.command_total.get() == 0) {
        return false;
    }

    // read WP position
    mem = WP_START_BYTE;
    loc.options = hal.storage->read_byte(mem);
    mem++;
    
    loc.alt = hal.storage->read_dword(mem);
    mem += 4;
    
    loc.lat = hal.storage->read_dword(mem);
    mem += 4;
    
    loc.lng = hal.storage->read_dword(mem);

    return true;
}

static void set_home_eeprom(struct Location temp)
{
    uint16_t mem = WP_START_BYTE;

    hal.storage->write_byte(mem, temp.options);
    mem++;

    hal.storage->write_dword(mem, temp.alt);
    mem += 4;

    hal.storage->write_dword(mem, temp.lat);
    mem += 4;

    hal.storage->write_dword(mem, temp.lng);

    // Now have a home location in EEPROM
    g.command_total.set_and_save(1); // At most 1 entry for HOME
}

static void set_home(struct Location temp)
{
    set_home_eeprom(temp);
    current_loc = temp;
}

static void arm_servos()
{    
    channel_yaw.enable_out();
    channel_pitch.enable_out();
}

static void disarm_servos()
{
    channel_yaw.disable_out();
    channel_pitch.disable_out();
}

/*
  setup servos to trim value after initialising
 */
static void prepare_servos()
{
    start_time_ms = hal.scheduler->millis();
    channel_yaw.radio_out = channel_yaw.radio_trim;
    channel_pitch.radio_out = channel_pitch.radio_trim;
    channel_yaw.output();
    channel_pitch.output();
}

static void set_mode(enum ControlMode mode)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    control_mode = mode;

	switch (control_mode) {
    case AUTO:
    case MANUAL:
    case SCAN:
        arm_servos();
        break;

    case STOP:
    case INITIALISING:
        disarm_servos();
        break;
    }
}

static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud, SERIAL1_BAUD));
    }
#endif
}
