-- Fence diagnostics: report fence status, distances, and directions

local SCRIPT_NAME = "fence_diag"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 96
local PARAM_TABLE_PREFIX = "FDIAG_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT_NAME, PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3),
       string.format("%s: could not add param table", SCRIPT_NAME))

--[[
    // @Param: FDIAG_ENABLE
    // @DisplayName: Fence diagnostics enable
    // @Description: Enable periodic fence status logging
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local FDIAG_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
    // @Param: FDIAG_LOG_MS
    // @DisplayName: Fence diagnostics log interval
    // @Description: Interval (ms) for status text, 0 disables
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
--]]
local FDIAG_LOG_MS = bind_add_param("LOG_MS", 2, 1000)

--[[
    // @Param: FDIAG_TYPES
    // @DisplayName: Fence types bitmask
    // @Description: Fence types to report (1:AltMax,2:Circle,4:Polygon,8:AltMin)
    // @Range: 1 15
    // @User: Advanced
--]]
local FDIAG_TYPES = bind_add_param("TYPES", 3, 15)

local last_log_ms = 0

local function log_status(msg)
    local interval = FDIAG_LOG_MS:get()
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
    if FDIAG_ENABLE:get() <= 0 then
        return update, 1000
    end

    local types = FDIAG_TYPES:get()
    local breaches = fence:get_breaches()
    local margin = fence:get_margin_breaches()

    local circle_dir = fence:get_breach_direction_NED(2)
    local circle_dist = circle_dir and circle_dir:length() or -1
    local poly_dir = fence:get_breach_direction_NED(4)
    local poly_dist = poly_dir and (poly_dir:length() * 0.01) or -1

    local closest = -1
    if circle_dist >= 0 then
        closest = circle_dist
    end
    if poly_dist >= 0 and (closest < 0 or poly_dist < closest) then
        closest = poly_dist
    end

    log_status(string.format("types=%d breach=%d margin=%d closest=%.2f circle=%.2f poly=%.2f",
                             types, breaches, margin, closest, circle_dist, poly_dist))

    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": loaded")

return update()
