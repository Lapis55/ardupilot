-- Easy input shaping via ATC_INPUT_TC using an RC knob
-- Place in scripts/ and enable SCR_ENABLE to run

local SCRIPT_NAME = "easy_input_tc"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 94
local PARAM_TABLE_PREFIX = "EASYTC_"
local UPDATE_MS = 100
local WARN_INTERVAL_MS = 2000
local DEBUG_INTERVAL_MS = 1000
local TC_EPSILON = 0.002

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format("%s: could not add param %s", SCRIPT_NAME, PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4),
       string.format("%s: could not add param table", SCRIPT_NAME))

--[[
    // @Param: EASYTC_ENABLE
    // @DisplayName: Easy input shaping enable
    // @Description: Enable mapping an RC knob to ATC_INPUT_TC
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
--]]
local EASYTC_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
    // @Param: EASYTC_RC_OPT
    // @DisplayName: RCx_OPTION value for knob input
    // @Description: Set your knob channel's RCx_OPTION to this value (300=Scripting1)
    // @Values: 300:Scripting1,301:Scripting2,302:Scripting3,303:Scripting4,304:Scripting5,305:Scripting6
    // @User: Standard
--]]
local EASYTC_RC_OPT = bind_add_param("RC_OPT", 2, 300)

--[[
    // @Param: EASYTC_TC_MIN
    // @DisplayName: ATC_INPUT_TC minimum
    // @Description: Lower bound for ATC_INPUT_TC mapping
    // @Range: 0 1
    // @Units: s
    // @User: Standard
--]]
local EASYTC_TC_MIN = bind_add_param("TC_MIN", 3, 0.05)

--[[
    // @Param: EASYTC_TC_MAX
    // @DisplayName: ATC_INPUT_TC maximum
    // @Description: Upper bound for ATC_INPUT_TC mapping
    // @Range: 0 1
    // @Units: s
    // @User: Standard
--]]
local EASYTC_TC_MAX = bind_add_param("TC_MAX", 4, 0.5)

local last_opt = -1
local rc_ch = nil
local last_tc = nil
local last_warn_ms = 0
local last_dbg_ms = 0

local function clamp(v, vmin, vmax)
    if v < vmin then
        return vmin
    end
    if v > vmax then
        return vmax
    end
    return v
end

local function get_rc_channel()
    local opt = EASYTC_RC_OPT:get()
    if opt ~= last_opt then
        rc_ch = rc:find_channel_for_option(opt)
        last_opt = opt
    end
    return rc_ch
end

local function compute_tc(norm, tc_min, tc_max)
    if tc_max < tc_min then
        tc_min, tc_max = tc_max, tc_min
    end
    local t = (norm + 1.0) * 0.5
    return clamp(tc_min + (tc_max - tc_min) * t, 0.0, 1.0)
end

local function update()
    if EASYTC_ENABLE:get() <= 0 then
        return update, 500
    end

    local ch = get_rc_channel()
    if not ch then
        local now = millis()
        if now - last_warn_ms > WARN_INTERVAL_MS then
            gcs:send_text(MAV_SEVERITY.WARNING,
                string.format("%s: RCx_OPTION %d not found", SCRIPT_NAME, EASYTC_RC_OPT:get()))
            last_warn_ms = now
        end
        return update, 500
    end

    local norm = ch:norm_input()
    if norm == nil then
        return update, 200
    end

    local tc_min = EASYTC_TC_MIN:get()
    local tc_max = EASYTC_TC_MAX:get()
    local tc = compute_tc(norm, tc_min, tc_max)

    if last_tc == nil or math.abs(tc - last_tc) > TC_EPSILON then
        local ok = param:set("ATC_INPUT_TC", tc)
        if not ok then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": failed to set ATC_INPUT_TC")
        else
            last_tc = tc
        end
    end

    local now = millis()
    if now - last_dbg_ms > DEBUG_INTERVAL_MS then
        gcs:send_text(MAV_SEVERITY.INFO,
            string.format("%s: rc=%.2f tc=%.3f", SCRIPT_NAME, norm, tc))
        last_dbg_ms = now
    end

    return update, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": loaded")

return update()
