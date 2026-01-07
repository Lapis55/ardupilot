-- poscontrol API diagnostics: log velocity/accel targets and offsets

local SCRIPT_NAME = "poscontrol_diag"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 97
local PARAM_TABLE_PREFIX = "PDIAG_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT_NAME, PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5),
       string.format("%s: could not add param table", SCRIPT_NAME))

--[[
    // @Param: PDIAG_ENABLE
    // @DisplayName: Poscontrol diagnostics enable
    // @Description: Enable periodic poscontrol API logging
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local PDIAG_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
    // @Param: PDIAG_LOG_MS
    // @DisplayName: Poscontrol diagnostics log interval
    // @Description: Interval (ms) for status text, 0 disables
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
--]]
local PDIAG_LOG_MS = bind_add_param("LOG_MS", 2, 1000)

--[[
    // @Param: PDIAG_CMD_EN
    // @DisplayName: Velocity command enable
    // @Description: Enable periodic velocity commands to trace a circle in Guided
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local PDIAG_CMD_EN = bind_add_param("CMD_EN", 3, 0)

--[[
    // @Param: PDIAG_SPEED
    // @DisplayName: Velocity command speed
    // @Description: Horizontal speed (m/s) for circular velocity command
    // @Units: m/s
    // @Range: 0.1 5
    // @User: Standard
--]]
local PDIAG_SPEED = bind_add_param("SPEED", 4, 1.0)

--[[
    // @Param: PDIAG_RADIUS
    // @DisplayName: Velocity command radius
    // @Description: Radius (m) for circular velocity command
    // @Units: m
    // @Range: 1 50
    // @User: Standard
--]]
local PDIAG_RADIUS = bind_add_param("RADIUS", 5, 5.0)

local last_log_ms = 0
local last_cmd_ms = 0

local GUIDED_MODE = 4

local function log_status(msg)
    local interval = PDIAG_LOG_MS:get()
    if interval <= 0 then
        return
    end
    local now = millis()
    if now - last_log_ms > interval then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": " .. msg)
        last_log_ms = now
    end
end

local function update()
    if PDIAG_ENABLE:get() <= 0 then
        return update, 1000
    end

    if PDIAG_CMD_EN:get() > 0 and vehicle:get_mode() == GUIDED_MODE then
        local speed = PDIAG_SPEED:get()
        local radius = PDIAG_RADIUS:get()
        if speed and radius and speed > 0 and radius > 0 then
            local omega = speed / radius
            local t = millis():tofloat() * 0.001
            local vel = Vector3f()
            vel:x(speed * math.cos(omega * t))
            vel:y(speed * math.sin(omega * t))
            vel:z(0.0)
            vehicle:set_target_velocity_NED(vel)
            last_cmd_ms = millis()
        end
    end

    local vel = poscontrol:get_vel_target()
    local acc = poscontrol:get_accel_target()
    local pos_off, vel_off, acc_off = poscontrol:get_posvelaccel_offset()

    local vel_str = vel and string.format("%.2f,%.2f,%.2f", vel:x(), vel:y(), vel:z()) or "nil"
    local acc_str = acc and string.format("%.2f,%.2f,%.2f", acc:x(), acc:y(), acc:z()) or "nil"
    local off_str = "nil"
    if pos_off and vel_off and acc_off then
        off_str = string.format("p(%.2f,%.2f,%.2f) v(%.2f,%.2f,%.2f) a(%.2f,%.2f,%.2f)",
                                pos_off:x(), pos_off:y(), pos_off:z(),
                                vel_off:x(), vel_off:y(), vel_off:z(),
                                acc_off:x(), acc_off:y(), acc_off:z())
    end

    log_status(string.format("mode=%d vel=%s acc=%s off=%s", vehicle:get_mode(), vel_str, acc_str, off_str))
    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": loaded")

return update()
