// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Common GCS MAVLink functions for all vehicle types

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <GCS.h>
#include <AP_AHRS.h>

extern const AP_HAL::HAL& hal;

uint32_t GCS_MAVLINK::last_radio_status_remrssi_ms;

GCS_MAVLINK::GCS_MAVLINK() :
    waypoint_receive_timeout(1000)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void
GCS_MAVLINK::init(AP_HAL::UARTDriver *port)
{
    GCS_Class::init(port);
    if (port == (AP_HAL::BetterStream*)hal.uartA) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
        initialised = true;
    } else if (port == (AP_HAL::BetterStream*)hal.uartC) {
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
        initialised = true;
#if MAVLINK_COMM_NUM_BUFFERS > 2
    } else if (port == (AP_HAL::BetterStream*)hal.uartD) {
        mavlink_comm_2_port = port;
        chan = MAVLINK_COMM_2;
        initialised = true;
#endif
    }
    _queued_parameter = NULL;
    reset_cli_timeout();
}


uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
        do {
            _parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
    }
    return _parameter_count;
}

/**
 * @brief Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_param_send()
{
    if (!initialised || _queued_parameter == NULL) {
        return;
    }

    uint16_t bytes_allowed;
    uint8_t count;
    uint32_t tnow = hal.scheduler->millis();

    // use at most 30% of bandwidth on parameters. The constant 26 is
    // 1/(1000 * 1/8 * 0.001 * 0.3)
    bytes_allowed = 57 * (tnow - _queued_parameter_send_time_ms) * 26;
    if (bytes_allowed > comm_get_txspace(chan)) {
        bytes_allowed = comm_get_txspace(chan);
    }
    count = bytes_allowed / (MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);

    while (_queued_parameter != NULL && count--) {
        AP_Param      *vp;
        float value;

        // copy the current parameter and prepare to move to the next
        vp = _queued_parameter;

        // if the parameter can be cast to float, report it here and break out of the loop
        value = vp->cast_to_float(_queued_parameter_type);

        char param_name[AP_MAX_NAME_SIZE];
        vp->copy_name_token(_queued_parameter_token, param_name, sizeof(param_name), true);

        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(_queued_parameter_type),
            _queued_parameter_count,
            _queued_parameter_index);

        _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index++;
    }
    _queued_parameter_send_time_ms = tnow;
}

/**
 * @brief Send the next pending waypoint, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (initialised &&
        waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

void GCS_MAVLINK::reset_cli_timeout() {
      _cli_timeout = hal.scheduler->millis();
}

void GCS_MAVLINK::send_meminfo(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
    extern unsigned __brkval;
#else
    unsigned __brkval = 0;
#endif
    mavlink_msg_meminfo_send(chan, __brkval, hal.util->available_memory());
}

// report power supply status
void GCS_MAVLINK::send_power_status(void)
{
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
    mavlink_msg_power_status_send(chan,
                                  hal.analogin->board_voltage() * 1000,
                                  hal.analogin->servorail_voltage() * 1000,
                                  hal.analogin->power_status_flags());
#endif
}

// report AHRS2 state
void GCS_MAVLINK::send_ahrs2(AP_AHRS &ahrs)
{
#if AP_AHRS_NAVEKF_AVAILABLE
    Vector3f euler;
    struct Location loc;
    if (ahrs.get_secondary_attitude(euler) && ahrs.get_secondary_position(loc)) {
        mavlink_msg_ahrs2_send(chan,
                               euler.x,
                               euler.y,
                               euler.z,
                               loc.alt*1.0e-2f,
                               loc.lat,
                               loc.lng);
    }
#endif
}

/*
  handle a MISSION_REQUEST_LIST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    // reply with number of commands in the mission.  The GCS will then request each command separately
    mavlink_msg_mission_count_send(chan,msg->sysid, msg->compid, mission.num_commands());

    // set variables to help handle the expected sending of commands to the GCS
    waypoint_receiving = false;             // record that we are sending commands (i.e. not receiving)
    waypoint_dest_sysid = msg->sysid;       // record system id of GCS who has requested the commands
    waypoint_dest_compid = msg->compid;     // record component id of GCS who has requested the commands
}

/*
  handle a MISSION_REQUEST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request(AP_Mission &mission, mavlink_message_t *msg)
{
    AP_Mission::Mission_Command cmd;
    // decode
    mavlink_mission_request_t packet;
    mavlink_msg_mission_request_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
        goto mission_item_send_failed;
    }

    // convert mission command to mavlink mission item packet
    mavlink_mission_item_t ret_packet;
    memset(&ret_packet, 0, sizeof(ret_packet));
    if (!AP_Mission::mission_cmd_to_mavlink(cmd, ret_packet)) {
        goto mission_item_send_failed;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    /*
      avoid the _send() function to save memory on APM2, as it avoids
      the stack usage of the _send() function by using the already
      declared ret_packet above
     */
    ret_packet.target_system = msg->sysid;
    ret_packet.target_component = msg->compid;
    ret_packet.seq = packet.seq;
    ret_packet.command = cmd.id;

    _mav_finalize_message_chan_send(chan, 
                                    MAVLINK_MSG_ID_MISSION_ITEM,
                                    (const char *)&ret_packet,
                                    MAVLINK_MSG_ID_MISSION_ITEM_LEN,
                                    MAVLINK_MSG_ID_MISSION_ITEM_CRC);
    return;

mission_item_send_failed:
    // send failure message
    mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ERROR);
}

/*
  handle a MISSION_SET_CURRENT mavlink packet
 */
void GCS_MAVLINK::handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_set_current_t packet;
    mavlink_msg_mission_set_current_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system,packet.target_component)) {
        return;
    }

    // set current command
    if (mission.set_current_cmd(packet.seq)) {
        mavlink_msg_mission_current_send(chan, mission.get_current_nav_cmd().index);
    }
}

/*
  handle a MISSION_COUNT mavlink packet
 */
void GCS_MAVLINK::handle_mission_count(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system,packet.target_component)) {
        return;
    }

    // start waypoint receiving
    if (packet.count > mission.num_commands_max()) {
        // send NAK
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_NO_SPACE);
        return;
    }

    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);

    // set variables to help handle the expected receiving of commands from the GCS
    waypoint_timelast_receive = hal.scheduler->millis();    // set time we last received commands to now
    waypoint_receiving = true;              // record that we expect to receive commands
    waypoint_request_i = 0;                 // reset the next expected command number to zero
    waypoint_request_last = packet.count;   // record how many commands we expect to receive
    waypoint_timelast_request = 0;          // set time we last requested commands to zero
}

/*
  handle a MISSION_CLEAR_ALL mavlink packet
 */
void GCS_MAVLINK::handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    // clear all waypoints
    if (mission.clear()) {
        // send ack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_RESULT_ACCEPTED);
    }else{
        // send nack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, 1);
    }
}

/*
  handle a MISSION_WRITE_PARTIAL_LIST mavlink packet
 */
void GCS_MAVLINK::handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_write_partial_list_t packet;
    mavlink_msg_mission_write_partial_list_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system,packet.target_component)) {
        return;
    }

    // start waypoint receiving
    if ((unsigned)packet.start_index > mission.num_commands() ||
        (unsigned)packet.end_index > mission.num_commands() ||
        packet.end_index < packet.start_index) {
        send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
        return;
    }

    waypoint_timelast_receive = hal.scheduler->millis();
    waypoint_timelast_request = 0;
    waypoint_receiving   = true;
    waypoint_request_i   = packet.start_index;
    waypoint_request_last= packet.end_index;
}

/*
  return true if a channel has flow control
 */
bool GCS_MAVLINK::have_flow_control(void)
{
    switch (chan) {
    case MAVLINK_COMM_0:
        // assume USB has flow control
        return hal.gpio->usb_connected() || hal.uartA->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;

    case MAVLINK_COMM_1:
        return hal.uartC->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;

    case MAVLINK_COMM_2:
        return hal.uartD != NULL && hal.uartD->get_flow_control() != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;

    default:
        break;
    }
    return false;
}


/*
  handle a request to change stream rate. Note that copter passes in
  save==false, as sending mavlink messages on copter on APM2 costs
  enough that it can cause flight issues, so we don't want the save to
  happen when the user connects the ground station.
 */
void GCS_MAVLINK::handle_request_data_stream(mavlink_message_t *msg, bool save)
{
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(msg, &packet);

    if (mavlink_check_target(packet.target_system, packet.target_component))
        return;

    int16_t freq = 0;     // packet frequency

    if (packet.start_stop == 0)
        freq = 0;                     // stop sending
    else if (packet.start_stop == 1)
        freq = packet.req_message_rate;                     // start sending
    else
        return;

    AP_Int16 *rate = NULL;
    switch (packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL:
        // note that we don't set STREAM_PARAMS - that is internal only
        for (uint8_t i=0; i<STREAM_PARAMS; i++) {
            if (save) {
                streamRates[i].set_and_save_ifchanged(freq);
            } else {
                streamRates[i].set(freq);
            }
        }
        break;
    case MAV_DATA_STREAM_RAW_SENSORS:
        rate = &streamRates[STREAM_RAW_SENSORS];
        break;
    case MAV_DATA_STREAM_EXTENDED_STATUS:
        rate = &streamRates[STREAM_EXTENDED_STATUS];
        break;
    case MAV_DATA_STREAM_RC_CHANNELS:
        rate = &streamRates[STREAM_RC_CHANNELS];
        break;
    case MAV_DATA_STREAM_RAW_CONTROLLER:
        rate = &streamRates[STREAM_RAW_CONTROLLER];
        break;
    case MAV_DATA_STREAM_POSITION:
        rate = &streamRates[STREAM_POSITION];
        break;
    case MAV_DATA_STREAM_EXTRA1:
        rate = &streamRates[STREAM_EXTRA1];
        break;
    case MAV_DATA_STREAM_EXTRA2:
        rate = &streamRates[STREAM_EXTRA2];
        break;
    case MAV_DATA_STREAM_EXTRA3:
        rate = &streamRates[STREAM_EXTRA3];
        break;
    }

    if (rate != NULL) {
        if (save) {
            rate->set_and_save_ifchanged(freq);
        } else {
            rate->set(freq);
        }
    }
}

void GCS_MAVLINK::handle_param_request_list(mavlink_message_t *msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system,packet.target_component)) {
        return;
    }

#if CONFIG_HAL_BOARD != HAL_BOARD_APM1 && CONFIG_HAL_BOARD != HAL_BOARD_APM2
    // send system ID if we can
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        send_text(SEVERITY_LOW, sysid);
    }
#endif

    // Start sending parameters - next call to ::update will kick the first one out
    _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index = 0;
    _queued_parameter_count = _count_parameters();
}

void GCS_MAVLINK::handle_param_request_read(mavlink_message_t *msg)
{
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system,packet.target_component)) {
        return;
    }
    enum ap_var_type p_type;
    AP_Param *vp;
    char param_name[AP_MAX_NAME_SIZE+1];
    if (packet.param_index != -1) {
        AP_Param::ParamToken token;
        vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
        if (vp == NULL) {
            return;
        }
        vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
        param_name[AP_MAX_NAME_SIZE] = 0;
    } else {
        strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
        param_name[AP_MAX_NAME_SIZE] = 0;
        vp = AP_Param::find(param_name, &p_type);
        if (vp == NULL) {
            return;
        }
    }
    
    float value = vp->cast_to_float(p_type);
    mavlink_msg_param_value_send_buf(
        msg,
        chan,
        param_name,
        value,
        mav_var_type(p_type),
        _count_parameters(),
        packet.param_index);
}

void GCS_MAVLINK::handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash)
{
    AP_Param *vp;
    enum ap_var_type var_type;

    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(msg, &packet);

    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    // set parameter
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    // find the requested parameter
    vp = AP_Param::find(key, &var_type);
    if ((NULL != vp) &&                                 // exists
        !isnan(packet.param_value) &&                       // not nan
        !isinf(packet.param_value)) {                       // not inf

        // add a small amount before casting parameter values
        // from float to integer to avoid truncating to the
        // next lower integer value.
        float rounding_addition = 0.01;
        
        // handle variables with standard type IDs
        if (var_type == AP_PARAM_FLOAT) {
            ((AP_Float *)vp)->set_and_save(packet.param_value);
        } else if (var_type == AP_PARAM_INT32) {
            if (packet.param_value < 0) rounding_addition = -rounding_addition;
            float v = packet.param_value+rounding_addition;
            v = constrain_float(v, -2147483648.0, 2147483647.0);
            ((AP_Int32 *)vp)->set_and_save(v);
        } else if (var_type == AP_PARAM_INT16) {
            if (packet.param_value < 0) rounding_addition = -rounding_addition;
            float v = packet.param_value+rounding_addition;
            v = constrain_float(v, -32768, 32767);
            ((AP_Int16 *)vp)->set_and_save(v);
        } else if (var_type == AP_PARAM_INT8) {
            if (packet.param_value < 0) rounding_addition = -rounding_addition;
            float v = packet.param_value+rounding_addition;
            v = constrain_float(v, -128, 127);
            ((AP_Int8 *)vp)->set_and_save(v);
        } else {
            // we don't support mavlink set on this parameter
            return;
        }

        // Report back the new value if we accepted the change
        // we send the value we actually set, which could be
        // different from the value sent, in case someone sent
        // a fractional value to an integer type
        mavlink_msg_param_value_send_buf(
            msg,
            chan,
            key,
            vp->cast_to_float(var_type),
            mav_var_type(var_type),
            _count_parameters(),
            -1);     // XXX we don't actually know what its index is...
        if (DataFlash != NULL) {
            DataFlash->Log_Write_Parameter(key, vp->cast_to_float(var_type));
        }
    }
}


void
GCS_MAVLINK::send_text(gcs_severity severity, const char *str)
{
    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        mavlink_statustext_t *s = &pending_status;
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        send_message(MSG_STATUSTEXT);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

void
GCS_MAVLINK::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
        if (m.text[i] == '\0') {
            break;
        }
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    send_text(severity, (const char *)m.text);
}


void GCS_MAVLINK::handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio)
{
    mavlink_radio_t packet;
    mavlink_msg_radio_decode(msg, &packet);

    // record if the GCS has been receiving radio messages from
    // the aircraft
    if (packet.remrssi != 0) {
        last_radio_status_remrssi_ms = hal.scheduler->millis();
    }

    // use the state of the transmit buffer in the radio to
    // control the stream rate, giving us adaptive software
    // flow control
    if (packet.txbuf < 20 && stream_slowdown < 100) {
        // we are very low on space - slow down a lot
        stream_slowdown += 3;
    } else if (packet.txbuf < 50 && stream_slowdown < 100) {
        // we are a bit low on space, slow down slightly
        stream_slowdown += 1;
    } else if (packet.txbuf > 95 && stream_slowdown > 10) {
        // the buffer has plenty of space, speed up a lot
        stream_slowdown -= 2;
    } else if (packet.txbuf > 90 && stream_slowdown != 0) {
        // the buffer has enough space, speed up a bit
        stream_slowdown--;
    }

    //log rssi, noise, etc if logging Performance monitoring data
    if (log_radio) {
        dataflash.Log_Write_Radio(packet);
    }
}


/*
  handle an incoming mission item
 */
void GCS_MAVLINK::handle_mission_item(mavlink_message_t *msg, AP_Mission &mission)
{
    mavlink_mission_item_t packet;
    uint8_t result = MAV_MISSION_ACCEPTED;
    struct AP_Mission::Mission_Command cmd = {};

    mavlink_msg_mission_item_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system,packet.target_component)) {
        return;
    }

    // convert mavlink packet to mission command
    if (!AP_Mission::mavlink_to_mission_cmd(packet, cmd)) {
        result = MAV_MISSION_INVALID;
        goto mission_ack;
    }

    if (packet.current == 2) {                                               
        // current = 2 is a flag to tell us this is a "guided mode"
        // waypoint and not for the mission
        handle_guided_request(cmd);

        // verify we recevied the command
        result = 0;
        goto mission_ack;
    }

    if (packet.current == 3) {
        //current = 3 is a flag to tell us this is a alt change only
        // add home alt if needed
        handle_change_alt_request(cmd);

        // verify we recevied the command
        result = 0;
        goto mission_ack;
    }

    // Check if receiving waypoints (mission upload expected)
    if (!waypoint_receiving) {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }

    // check if this is the requested waypoint
    if (packet.seq != waypoint_request_i) {
        result = MAV_MISSION_INVALID_SEQUENCE;
        goto mission_ack;
    }
    
    // if command index is within the existing list, replace the command
    if (packet.seq < mission.num_commands()) {
        if (mission.replace_cmd(packet.seq,cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if command is at the end of command list, add the command
    } else if (packet.seq == mission.num_commands()) {
        if (mission.add_cmd(cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if beyond the end of the command list, return an error
    } else {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }
    
    // update waypoint receiving state machine
    waypoint_timelast_receive = hal.scheduler->millis();
    waypoint_request_i++;
    
    if (waypoint_request_i >= waypoint_request_last) {
        mavlink_msg_mission_ack_send_buf(
            msg,
            chan,
            msg->sysid,
            msg->compid,
            MAV_MISSION_ACCEPTED);
        
        send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
        waypoint_receiving = false;
        // XXX ignores waypoint radius for individual waypoints, can
        // only set WP_RADIUS parameter
    } else {
        waypoint_timelast_request = hal.scheduler->millis();
        // if we have enough space, then send the next WP immediately
        if (comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= MAVLINK_MSG_ID_MISSION_ITEM_LEN) {
            queued_waypoint_send();
        } else {
            send_message(MSG_NEXT_WAYPOINT);
        }
    }
    return;

mission_ack:
    // we are rejecting the mission/waypoint
    mavlink_msg_mission_ack_send_buf(
        msg,
        chan,
        msg->sysid,
        msg->compid,
        result);
}

// send a message using mavlink, handling message queueing
void GCS_MAVLINK::send_message(enum ap_message id)
{
    uint8_t i, nextid;

    // see if we can send the deferred messages, if any
    while (num_deferred_messages != 0) {
        if (!try_send_message(deferred_messages[next_deferred_message])) {
            break;
        }
        next_deferred_message++;
        if (next_deferred_message == MSG_RETRY_DEFERRED) {
            next_deferred_message = 0;
        }
        num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = next_deferred_message; i < num_deferred_messages; i++) {
        if (deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MSG_RETRY_DEFERRED) {
            nextid = 0;
        }
    }

    if (num_deferred_messages != 0 ||
        !try_send_message(id)) {
        // can't send it now, so defer it
        if (num_deferred_messages == MSG_RETRY_DEFERRED) {
            // the defer buffer is full, discard
            return;
        }
        nextid = next_deferred_message + num_deferred_messages;
        if (nextid >= MSG_RETRY_DEFERRED) {
            nextid -= MSG_RETRY_DEFERRED;
        }
        deferred_messages[nextid] = id;
        num_deferred_messages++;
    }
}
