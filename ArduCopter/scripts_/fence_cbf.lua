-- Soft fence repulsion using poscontrol accel offsets (Guided only)

local SCRIPT_NAME = "fence_cbf"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 95
local PARAM_TABLE_PREFIX = "FCBF_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT_NAME, PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7),
       string.format("%s: could not add param table", SCRIPT_NAME))

--[[
    // @Param: FCBF_ENABLE
    // @DisplayName: Soft fence repulsion enable
    // @Description: Enable soft fence repulsion using accel offsets
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local FCBF_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
    // @Param: FCBF_TYPES
    // @DisplayName: Fence types bitmask
    // @Description: Fence types to use for repulsion (1:AltMax,2:Circle,4:Polygon,8:AltMin)
    // @Range: 1 15
    // @User: Advanced
--]]
local FCBF_TYPES = bind_add_param("TYPES", 2, 6) -- circle+polygon

--[[
    // @Param: FCBF_START
    // @DisplayName: Repulsion start distance
    // @Description: Distance (m) to start applying repulsion
    // @Units: m
    // @Range: 1 50
    // @User: Standard
--]]
local FCBF_START = bind_add_param("START", 3, 5.0)

--[[
    // @Param: FCBF_MIN
    // @DisplayName: Minimum distance for barrier
    // @Description: Minimum distance (m) used in barrier to avoid infinity
    // @Units: m
    // @Range: 0.1 5
    // @User: Standard
--]]
local FCBF_MIN = bind_add_param("MIN", 4, 0.5)

--[[
    // @Param: FCBF_K
    // @DisplayName: Repulsion gain
    // @Description: Gain used in barrier function
    // @Range: 0.1 10
    // @User: Advanced
--]]
local FCBF_K = bind_add_param("K", 5, 2.0)

--[[
    // @Param: FCBF_AMAX
    // @DisplayName: Maximum repulsion acceleration
    // @Description: Max accel (m/s/s) applied as repulsion
    // @Units: m/s/s
    // @Range: 0.1 10
    // @User: Standard
--]]
local FCBF_AMAX = bind_add_param("AMAX", 6, 2.0)

--[[
    // @Param: FCBF_LOG_MS
    // @DisplayName: Status log interval
    // @Description: Interval (ms) for status text, 0 disables
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
--]]
local FCBF_LOG_MS = bind_add_param("LOG_MS", 7, 1000)

local GUIDED_MODE = 4
local UPDATE_MS = 100

local last_log_ms = 0
local last_active = false
local last_diag_ms = 0

local function clamp(v, vmin, vmax)
    if v < vmin then
        return vmin
    end
    if v > vmax then
        return vmax
    end
    return v
end

local function clear_offsets()
    if last_active then
        poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), Vector3f())
        last_active = false
    end
end

local function log_status(msg)
    local interval = FCBF_LOG_MS:get()
    if interval <= 0 then
        return
    end
    local now = millis()
    if now - last_log_ms > interval then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": " .. msg)
        last_log_ms = now
    end
end

local function log_fence_diagnostics()
    local interval = FCBF_LOG_MS:get()
    if interval <= 0 then
        return
    end
    local now = millis()
    if now - last_diag_ms < interval then
        return
    end
    last_diag_ms = now

    local dir_circle = fence:get_breach_direction_NED(2)
    local dist_circle = dir_circle and dir_circle:length() or -1
    local dir_poly = fence:get_breach_direction_NED(4)
    local dist_poly = dir_poly and (dir_poly:length() * 0.01) or -1

    gcs:send_text(MAV_SEVERITY.INFO,
        string.format("%s: diag circle=%.2f poly=%.2f", SCRIPT_NAME, dist_circle, dist_poly))
end

local function get_dir_ned_meters()
    local types = FCBF_TYPES:get()
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
            local dist_poly = dir_poly:length() * 0.01 -- polygon direction is in cm
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
    if FCBF_ENABLE:get() <= 0 then
        clear_offsets()
        return update, 1000
    end

    if vehicle:get_mode() ~= GUIDED_MODE then
        clear_offsets()
        return update, 500
    end

    log_fence_diagnostics()

    local dir_ned = get_dir_ned_meters()
    if dir_ned == nil then
        clear_offsets()
        return update, UPDATE_MS
    end

    dir_ned:z(0.0)
    local dist = dir_ned:length()
    local start_dist = FCBF_START:get()
    if dist <= 0 or dist >= start_dist then
        clear_offsets()
        log_status(string.format("dist=%.2f idle", dist))
        return update, UPDATE_MS
    end

    local min_dist = clamp(FCBF_MIN:get(), 0.05, start_dist)
    local d = math.max(dist, min_dist)
    local gain = FCBF_K:get()
    local accel_mag = gain * (1.0 / d - 1.0 / start_dist)
    accel_mag = clamp(accel_mag, 0.0, FCBF_AMAX:get())

    dir_ned:normalize()
    local accel = Vector3f()
    accel:x(-dir_ned:x() * accel_mag)
    accel:y(-dir_ned:y() * accel_mag)
    accel:z(0.0)

    if not poscontrol:set_posvelaccel_offset(Vector3f(), Vector3f(), accel) then
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": failed to set accel offset")
    else
        last_active = true
    end

    local _, _, accel_off = poscontrol:get_posvelaccel_offset()
    local off_str = "nil"
    if accel_off then
        off_str = string.format("%.2f,%.2f", accel_off:x(), accel_off:y())
    end
    log_status(string.format("dist=%.2f a=%.2f off=%s", dist, accel_mag, off_str))
    return update, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": loaded")

return update()
