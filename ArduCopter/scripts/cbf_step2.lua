-- CBF step 2: fence distance/dir diagnostics and per-axis CBF velocity clipping

local SCRIPT_NAME = "cbf_step2"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 99
local PARAM_TABLE_PREFIX = "CBF2_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT_NAME, PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 9),
       string.format("%s: could not add param table", SCRIPT_NAME))

--[[
    // @Param: CBF2_ENABLE
    // @DisplayName: CBF step2 enable
    // @Description: Enable Guided position control with optional CBF clipping
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local CBF2_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
    // @Param: CBF2_TYPES
    // @DisplayName: Fence types bitmask
    // @Description: Fence types to use (1:AltMax,2:Circle,4:Polygon,8:AltMin)
    // @Range: 1 15
    // @User: Advanced
--]]
local CBF2_TYPES = bind_add_param("TYPES", 2, 4)

--[[
    // @Param: CBF2_SAFE_D
    // @DisplayName: Safety distance
    // @Description: Distance (m) used in h = d - d_safe for CBF
    // @Units: m
    // @Range: 0.1 20
    // @User: Standard
--]]
local CBF2_SAFE_D = bind_add_param("SAFE_D", 3, 1.0)

--[[
    // @Param: CBF2_LOG_MS
    // @DisplayName: Status log interval
    // @Description: Interval (ms) for status text, 0 disables
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
--]]
local CBF2_LOG_MS = bind_add_param("LOG_MS", 4, 500)

--[[
    // @Param: CBF2_ALPHA
    // @DisplayName: CBF alpha gain
    // @Description: Gain used in per-axis CBF clipping (u <= alpha * h)
    // @Range: 0.1 20
    // @User: Standard
--]]
local CBF2_ALPHA = bind_add_param("ALPHA", 5, 1.0)

--[[
    // @Param: CBF2_KP
    // @DisplayName: Position P gain
    // @Description: Gain for position error to velocity command (XY)
    // @Range: 0.05 2
    // @User: Standard
--]]
local CBF2_KP = bind_add_param("KP", 6, 1.0)

--[[
    // @Param: CBF2_KD
    // @DisplayName: Velocity damping gain
    // @Description: Gain applied to current velocity for damping (XY)
    // @Range: 0 5
    // @User: Standard
--]]
local CBF2_KD = bind_add_param("KD", 7, 0.25)

--[[
    // @Param: CBF2_VMAX
    // @DisplayName: Velocity command max
    // @Description: Max horizontal speed from position P controller
    // @Units: m/s
    // @Range: 0.1 15
    // @User: Standard
--]]
local CBF2_VMAX = bind_add_param("VMAX", 8, 3.0)

--[[
    // @Param: CBF2_CBF_ENABLE
    // @DisplayName: CBF clip enable
    // @Description: Enable/disable CBF clipping while keeping position control
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local CBF2_CBF_ENABLE = bind_add_param("CBF_ENABLE", 9, 1)

local GUIDED_MODE = 4
local UPDATE_MS = 10

local last_log_ms = 0
local last_target_loc = nil
local last_yaw_deg = nil

local function log_status(msg)
    local interval = CBF2_LOG_MS:get()
    if interval <= 0 then
        return
    end
    local now = millis()
    if now - last_log_ms > interval then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": " .. msg)
        last_log_ms = now
    end
end

local function get_fence_dir_m()
    local types = CBF2_TYPES:get()
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

local function get_target_err(home, current_rel)
    local target_loc = vehicle:get_target_location()
    if target_loc then
        last_target_loc = target_loc
    end
    if not last_target_loc then
        return nil
    end

    local target_rel = home:get_distance_NED(last_target_loc)
    target_rel:x(target_rel:x() * 100.0)
    target_rel:y(target_rel:y() * 100.0)

    local err = Vector3f()
    err:x(target_rel:x() - current_rel:x())
    err:y(target_rel:y() - current_rel:y())
    err:z(0.0)
    return err
end

local function get_vel_des(err)
    local kp = CBF2_KP:get()
    local kd = CBF2_KD:get()
    local vmax = CBF2_VMAX:get()
    local vel_cur = ahrs:get_velocity_NED()
    local velx = 0.0
    local vely = 0.0
    if vel_cur then
        velx = vel_cur:x()
        vely = vel_cur:y()
    end
    local vel_des = Vector3f()
    local vx = kp * err:x() - kd * velx
    local vy = kp * err:y() - kd * vely
    local vnorm = math.sqrt(vx * vx + vy * vy)
    if vnorm > vmax and vnorm > 0.0 then
        local scale = vmax / vnorm
        vx = vx * scale
        vy = vy * scale
    end
    vel_des:x(vx)
    vel_des:y(vy)
    vel_des:z(0.0)
    return vel_des
end

local function apply_cbf(vel_cmd, dir_ned)
    if CBF2_CBF_ENABLE:get() <= 0 then
        return vel_cmd
    end

    local safe_d = CBF2_SAFE_D:get()
    local alpha = CBF2_ALPHA:get()
    local dist = dir_ned:length()
    if dist <= 0.0 then
        return vel_cmd
    end

    local h = dist - safe_d
    local lim = math.max(0.0, alpha * h)

    local n = Vector3f()
    n:x(dir_ned:x() / dist)
    n:y(dir_ned:y() / dist)
    n:z(0.0)

    local v_perp = vel_cmd:x() * n:x() + vel_cmd:y() * n:y()
    local v_perp_clamped = math.min(v_perp, lim)

    vel_cmd:x(vel_cmd:x() + (v_perp_clamped - v_perp) * n:x())
    vel_cmd:y(vel_cmd:y() + (v_perp_clamped - v_perp) * n:y())

    return vel_cmd
end

local function get_yaw_hold_deg()
    local yaw_rad = ahrs:get_yaw_rad()
    if yaw_rad then
        local yaw_deg = math.deg(yaw_rad)
        last_yaw_deg = yaw_deg
        return yaw_deg
    end
    return last_yaw_deg
end

local function is_enabled()
    return CBF2_ENABLE:get() > 0
end

local function is_guided_mode()
    return vehicle:get_mode() == GUIDED_MODE
end

local function get_home_and_rel()
    local home = ahrs:get_home()
    local current_rel = ahrs:get_relative_position_NED_home()
    if not (home and current_rel) then
        return nil, nil
    end
    return home, current_rel
end

local function build_vel_command(err, dir_ned)
    local vel_des = get_vel_des(err)
    local vel_cmd = Vector3f()
    vel_cmd:x(vel_des:x())
    vel_cmd:y(vel_des:y())
    vel_cmd:z(0.0)
    vel_cmd = apply_cbf(vel_cmd, dir_ned)
    -- vel_cmd:z(get_z_passthrough())
    return vel_des, vel_cmd
end

local function send_velocity_command(vel_cmd, yaw_deg)
    local accel_cmd = Vector3f()
    accel_cmd:x(0.0)
    accel_cmd:y(0.0)
    accel_cmd:z(0.0)
    local yaw_ok = yaw_deg ~= nil
    if not vehicle:set_target_velaccel_NED(vel_cmd, accel_cmd, yaw_ok, yaw_deg or 0.0, false, 0.0, false) then
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": failed to set target vel/accel")
    end
end

local function log_summary(mode, dist, err, vel_des, vel_cmd, yaw_deg)
    local vel_str = string.format("v(%.2f,%.2f)", vel_des:x(), vel_des:y())
    local cmd_str = string.format("v(%.2f,%.2f)", vel_cmd:x(), vel_cmd:y())
    log_status(string.format("mode=%d dist=%.2f err=(%.2f,%.2f) des=%s cmd=%s cbf=%d yaw=%.1f",
                             mode, dist, err:x(), err:y(), vel_str, cmd_str, CBF2_CBF_ENABLE:get(), yaw_deg or 0.0))
end

local function update()
    if not is_enabled() then
        return update, 1000
    end

    local mode = vehicle:get_mode()
    if not is_guided_mode() then
        return update, 500
    end

    local dir_ned = get_fence_dir_m()
    if dir_ned == nil then
        return update, UPDATE_MS
    end

    dir_ned:z(0.0)
    local dist = dir_ned:length()

    local home, current_rel = get_home_and_rel()
    if not (home and current_rel) then
        log_status("mode=" .. mode .. " home/pos=nil")
        return update, UPDATE_MS
    end

    local err = get_target_err(home, current_rel)
    if not err then
        log_status("mode=" .. mode .. " target_loc=nil")
        return update, UPDATE_MS
    end

    local vel_des, vel_cmd = build_vel_command(err, dir_ned)
    local yaw_deg = get_yaw_hold_deg()
    send_velocity_command(vel_cmd, yaw_deg)
    log_summary(mode, dist, err, vel_des, vel_cmd, yaw_deg)

    return update, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": loaded")

return update()
