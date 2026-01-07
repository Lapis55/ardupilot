-- CBF step 1: fence distance/dir diagnostics and velocity offset stop

local SCRIPT_NAME = "cbf_step1"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 98
local PARAM_TABLE_PREFIX = "CBF1_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT_NAME, PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5),
       string.format("%s: could not add param table", SCRIPT_NAME))

--[[
    // @Param: CBF1_ENABLE
    // @DisplayName: CBF step1 enable
    // @Description: Enable fence diagnostics and velocity stop offset
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local CBF1_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
    // @Param: CBF1_TYPES
    // @DisplayName: Fence types bitmask
    // @Description: Fence types to use (1:AltMax,2:Circle,4:Polygon,8:AltMin)
    // @Range: 1 15
    // @User: Advanced
--]]
local CBF1_TYPES = bind_add_param("TYPES", 2, 6)

--[[
    // @Param: CBF1_STOP_D
    // @DisplayName: Stop distance
    // @Description: Distance (m) at which velocity offset stops motion
    // @Units: m
    // @Range: 0.5 50
    // @User: Standard
--]]
local CBF1_STOP_D = bind_add_param("STOP_D", 3, 5.0)

--[[
    // @Param: CBF1_LOG_MS
    // @DisplayName: Status log interval
    // @Description: Interval (ms) for status text, 0 disables
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
--]]
local CBF1_LOG_MS = bind_add_param("LOG_MS", 4, 500)

--[[
    // @Param: CBF1_VFB_K
    // @DisplayName: Velocity feedback gain
    // @Description: Gain for actual velocity feedback in stop offset
    // @Range: 0 5
    // @User: Standard
--]]
local CBF1_VFB_K = bind_add_param("VFB_K", 5, 2.0)

local GUIDED_MODE = 4
local AUTO_MODE = 3
local UPDATE_MS = 100

local last_log_ms = 0

local function clamp(v, vmin, vmax)
    if v < vmin then
        return vmin
    end
    if v > vmax then
        return vmax
    end
    return v
end

local function log_status(msg)
    local interval = CBF1_LOG_MS:get()
    if interval <= 0 then
        return
    end
    local now = millis()
    if now - last_log_ms > interval then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": " .. msg)
        last_log_ms = now
    end
end

local function get_dir_ned_meters()
    local types = CBF1_TYPES:get()
    local best_dir = nil
    local best_dist = nil

    if (types & 2) ~= 0 then
        local dir_circle = fence:get_breach_direction_NED(2)
        if dir_circle then
            local dist_circle = dir_circle:length()
            best_dir = dir_circle
            best_dist = dist_circle
        end
    end

    if (types & 4) ~= 0 then
        local dir_poly = fence:get_breach_direction_NED(4)
        if dir_poly then
            local dist_poly = dir_poly:length() * 0.01
            if best_dist == nil or dist_poly < best_dist then
                local dir_poly_m = Vector3f()
                dir_poly_m:x(dir_poly:x() * 0.01)
                dir_poly_m:y(dir_poly:y() * 0.01)
                dir_poly_m:z(0.0)
                best_dir = dir_poly_m
                best_dist = dist_poly
            end
        end
    end

    return best_dir
end

local function update()
    if CBF1_ENABLE:get() <= 0 then
        return update, 1000
    end

    local mode = vehicle:get_mode()
    if mode ~= GUIDED_MODE and mode ~= AUTO_MODE then
        return update, 500
    end

    local dir_ned = get_dir_ned_meters()
    if dir_ned == nil then
        clear_offsets()
        return update, UPDATE_MS
    end

    dir_ned:z(0.0)
    local dist = dir_ned:length()

    local vel = poscontrol:get_vel_target()
    local acc = poscontrol:get_accel_target()
    local vel_act = ahrs:get_velocity_NED()
    local vel_str = vel and string.format("%.2f,%.2f,%.2f", vel:x(), vel:y(), vel:z()) or "nil"
    local acc_str = acc and string.format("%.2f,%.2f,%.2f", acc:x(), acc:y(), acc:z()) or "nil"
    local vel_act_str = vel_act and string.format("%.2f,%.2f,%.2f", vel_act:x(), vel_act:y(), vel_act:z()) or "nil"

    if dist < CBF1_STOP_D:get() then
        local vel_off = Vector3f()
        if vel then
            vel_off:x(-vel:x())
            vel_off:y(-vel:y())
            vel_off:z(-vel:z())
        end
        if vel_act then
            local k = CBF1_VFB_K:get()
            vel_off:x(vel_off:x() - k * vel_act:x())
            vel_off:y(vel_off:y() - k * vel_act:y())
            vel_off:z(vel_off:z() - k * vel_act:z())
        end
        local vel_cmd = Vector3f()
        if vel then
            vel_cmd:x(vel:x() + vel_off:x())
            vel_cmd:y(vel:y() + vel_off:y())
            vel_cmd:z(vel:z() + vel_off:z())
        end
        if not vehicle:set_target_velocity_NED(vel_cmd) then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": failed to set target velocity")
        end
        local off_str = string.format("v(%.2f,%.2f,%.2f)", vel_off:x(), vel_off:y(), vel_off:z())
        local cmd_str = string.format("v(%.2f,%.2f,%.2f)", vel_cmd:x(), vel_cmd:y(), vel_cmd:z())
        log_status(string.format("mode=%d dist=%.2f vel=%s vact=%s acc=%s off=%s cmd=%s",
                                 mode, dist, vel_str, vel_act_str, acc_str, off_str, cmd_str))
    else
        log_status(string.format("mode=%d dist=%.2f vel=%s vact=%s acc=%s off=0,0,0",
                                 mode, dist, vel_str, vel_act_str, acc_str))
    end

    return update, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": loaded")

return update()
