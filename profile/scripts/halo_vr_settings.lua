-- halo_vr_settings.lua -- in-game settings menus for the Halo: Campaign Evolved VR mod.
--
-- Three panels in the UEVR overlay (Script UI section):
--   "Halo VR User Settings"  every player-facing setting, grouped like the catalog; saves to
--                            halo_vr_user.cfg, so changes survive updates and apply in ~2 s.
--   "Halo VR DEV Settings"   the internal/research knobs from halo_vr_dev.cfg. Big warning:
--                            these can break things; updates overwrite that file on purpose.
--   "Halo VR Calibration"    buttons for the two calibration gestures + resets to the shipped
--                            fit. Arming a gesture hands the finish to the TRIGGERS (see below).
--
-- HOW IT TALKS TO THE MOD. UEVR sandboxes script file access to <profile>/data/, so this
-- script cannot touch the real cfg files. halo_vr.dll bridges on its ~2 s poll:
--   data/halo_vr_user_reference.txt  mirrored player catalog (sections/comments/defaults) READ
--   data/halo_vr_user_mirror.cfg     mirrored halo_vr_user.cfg                            READ
--   data/halo_vr_dev_mirror.cfg      mirrored halo_vr_dev.cfg (catalog + active overrides) READ
--   data/halo_vr_calib_mirror.cfg    mirrored halo_vr_calib.cfg (empty = shipped fit)     READ
--   data/halo_vr_status.txt          plugin status (armed calibration mode)               READ
--   data/halo_vr_menu_set.txt        command file this script WRITES; the plugin applies it:
--                                      key=value / -key            -> halo_vr_user.cfg
--                                      dev:key=value / dev:-key    -> halo_vr_dev.cfg
--                                      calib:pose|aim|off          -> arm/disarm calibration
--                                      calibreset:pose|aim|all     -> strip/delete calib file
-- If the catalog mirror is missing, the plugin is not loaded/current -- the menu says so.

local REF_FILE    = "halo_vr_user_reference.txt"
local USER_FILE   = "halo_vr_user_mirror.cfg"
local DEV_FILE    = "halo_vr_dev_mirror.cfg"
local CALIB_FILE  = "halo_vr_calib_mirror.cfg"
local STATUS_FILE = "halo_vr_status.txt"
local CMD_FILE    = "halo_vr_menu_set.txt"

-- Which calibration keys belong to which gesture (mirror of the plugin's own lists) -- used
-- only to show the per-gesture "overridden" state and its 'x'.
local POSE_KEYS = { "calibver", "grip", "gripyaw", "griproll", "dirgrip", "dirgripyaw",
                    "dirgriproll", "diroffx", "diroffy", "diroffz", "offx", "offy", "offz",
                    "pivauto", "pivx", "pivy", "pivz" }
local AIM_KEYS  = { "aimcalibver", "aimoffyaw", "aimoffpitch" }

-- Players never see raw codes: key-binding and button settings render as named dropdowns, and
-- the file keeps the numeric form. {value, display name} pairs.
local KEY_NAMES = {
    { 0x00, "disabled" },
    { 0x23, "End" }, { 0x24, "Home" }, { 0x21, "Page Up" }, { 0x22, "Page Down" },
    { 0x2D, "Insert" }, { 0x2E, "Delete" },
    { 0x70, "F1" }, { 0x71, "F2" }, { 0x72, "F3" }, { 0x73, "F4" },
    { 0x74, "F5" }, { 0x75, "F6" }, { 0x76, "F7" }, { 0x77, "F8" },
    { 0x78, "F9" }, { 0x79, "F10" }, { 0x7A, "F11" }, { 0x7B, "F12" },
}
local BTN_NAMES = {
    { 0x0000, "disabled" },
    { 0x1000, "A" }, { 0x2000, "B" }, { 0x4000, "X" }, { 0x8000, "Y" },
    { 0x0100, "LB" }, { 0x0200, "RB" },
    { 0x0040, "L3 (left stick click)" }, { 0x0080, "R3 (right stick click)" },
    { 0x0010, "Start" }, { 0x0020, "Back / View" },
    { 0x0001, "D-pad up" }, { 0x0002, "D-pad down" },
    { 0x0004, "D-pad left" }, { 0x0008, "D-pad right" },
}
local function named_lookup(list, v)
    for _, e in ipairs(list) do
        if e[1] == v then return e[2] end
    end
    return nil
end

-- ---------------------------------------------------------------- UI hints (widget shapes only;
-- defaults and grouping come from the catalog files, so a key with no hint still renders).
-- hide = no widget at all (the section text is expected to cover it); textdesc = print the
-- description as plain text above the widget instead of a hover tooltip.
local HINTS = {
    turnmode       = { t = "enum", items = { "0 - off", "1 - snap turn", "2 - smooth turn" }, values = { 0, 1, 2 } },
    snapdeg        = { t = "slider", min = 5, max = 90, int = true },
    smoothdps      = { t = "slider", min = 10, max = 360, int = true },
    turndz         = { t = "slider", min = 0.1, max = 0.95 },
    mapdpadshift   = { t = "bool" },
    mapdpaddz      = { t = "slider", min = 0.2, max = 0.95 },
    maprstickdz    = { t = "slider", min = 0.2, max = 0.95 },
    maprstickdown  = { t = "btn" },
    mapfrom        = { t = "btn" },
    mapto          = { t = "btn" },
    mapbtnlog      = { t = "bool" },
    mapmenuback    = { t = "btn" },
    menusuppress   = { t = "bool" },
    menudetect     = { t = "bool" },
    calibkey       = { t = "key" },
    aimcalibkey    = { t = "key" },
    hmdleash       = { t = "bool" },
    hmdleashlat    = { t = "slider", min = 0, max = 100 },   -- cm
    hmdleashvert   = { t = "slider", min = 0, max = 100 },   -- cm
    aimreticule    = { t = "bool" },
    aimreticuletrace = { t = "bool" },
    aimreticulemaxdist    = { t = "drag", min = 50, max = 100000 },
    aimreticulesurfaceoff = { t = "drag", min = 0, max = 500 },
    aimreticuleminscale     = { t = "drag", min = 0, max = 10 },
    aimreticuleminscaledist = { t = "drag", min = 1, max = 10000 },
    aimreticulemaxscale     = { t = "drag", min = 0.001, max = 20 },
    aimreticuledist       = { t = "drag", min = 100, max = 10000 },
    aimreticuledistveh    = { t = "drag", min = 0, max = 20000 },
    aimreticulescaleveh   = { t = "drag", min = 0.05, max = 5 },
    aimreticulesmoothms     = { t = "slider", min = 0, max = 500, int = true },
    aimreticulesmoothctrlms = { t = "slider", min = 0, max = 500, int = true },
    aimwidget      = { t = "bool" },
    aimwidgetscale = { t = "drag", min = 0.005, max = 5 },
    aimwidgetgain  = { t = "drag", min = 0, max = 1024 },
    aimwidgettint  = { t = "drag", min = 0, max = 1024 },
    hudhide        = { t = "bool" },
    aimmesh        = { t = "bool" },
    aimmeshscale   = { t = "drag", min = 0.005, max = 5 },
    aimmeshcr      = { t = "slider", min = 0, max = 1 },
    aimmeshcg      = { t = "slider", min = 0, max = 1 },
    aimmeshcb      = { t = "slider", min = 0, max = 1 },
    aimtexfile     = { t = "text" },
    scope          = { t = "bool" },
    scopezoom      = { t = "drag", min = 1.05, max = 300, speed = 0.5 },
    scoperes       = { t = "drag", min = 128, max = 2048, int = true },
    scopediv       = { t = "slider", min = 1, max = 8, int = true },
    scopeshape     = { t = "bool" },
    scopecalibkey  = { t = "key" },
    scopecamroll   = { t = "drag", min = -180, max = 180 },
    scopecamlock   = { t = "bool" },
    scopedist      = { t = "drag", min = 10, max = 300 },
    scopesize      = { t = "drag", min = 5, max = 100 },
    scoperight     = { t = "drag", min = -100, max = 100 },
    scopeup        = { t = "drag", min = -100, max = 100 },
    scopebright    = { t = "drag", min = 0, max = 10 },
    scopethresh    = { t = "slider", min = 0.05, max = 1 },
    perflog        = { t = "bool" },
    rigfast        = { t = "bool" },
}

-- ---------------------------------------------------------------- state
local user_catalog = nil   -- sections parsed from the reference
local dev_catalog  = nil   -- sections parsed from the dev mirror
local user_over    = {}    -- key -> value (active lines of halo_vr_user.cfg)
local dev_over     = {}    -- key -> value (active lines of halo_vr_dev.cfg)
local calib_over   = {}    -- key -> value (active lines of halo_vr_calib.cfg)
local pending_user = {}    -- key -> value string, or false = remove
local pending_dev  = {}
local once_cmds    = {}    -- one-shot commands (calib arm/reset), flushed on next write
local editbuf     = {}     -- "layer:key" -> in-progress text for text/hex fields
local calib_mode  = 0      -- plugin-reported armed calibration mode
local frame       = 0

local function trim(s)
    return (s:gsub("^%s+", ""):gsub("%s+$", ""))
end

-- Parse a catalog-shaped file: centered or left-aligned `# ==== NAME ====` banners open a
-- section; comment lines document; `#key=value` entries carry the defaults; ACTIVE `key=value`
-- lines are collected as overrides (the dev mirror holds catalog and overrides in one file).
-- Comment lines between a banner and an empty `#` line become the SECTION's own text.
local function parse_file(text)
    local sections = {}
    local active = {}
    if text == nil or text == "" then return sections, active end
    local current = nil
    local desc = {}
    local preamble = nil   -- non-nil while collecting section text
    local last_was_key = false
    for line in text:gmatch("[^\r\n]+") do
        local banner = line:match("^#%s*=+%s(.-)%s=+%s*$")
        if banner == nil then
            local m = line:match("^#%s*=+%s+(.+)$")
            if m ~= nil then banner = (m:gsub("%s*=+%s*$", "")) end
        end
        local ckey, cdef = line:match("^#([%w_]+)=(.*)$")
        local akey, aval = line:match("^([%w_]+)=(.*)$")
        if banner ~= nil and banner ~= "" and not banner:match("^=+$") then
            if banner ~= "FULL SETTINGS CATALOG" then
                current = { name = banner, text = nil, keys = {} }
                sections[#sections + 1] = current
                preamble = {}
            else
                current = nil
                preamble = nil
            end
            desc = {}
            last_was_key = false
        elseif ckey ~= nil and current ~= nil then
            if preamble ~= nil then
                -- no separator seen: the collected lines were this key's description
                desc = preamble
                preamble = nil
            end
            current.keys[#current.keys + 1] = { key = ckey, default = trim(cdef), desc = table.concat(desc, "\n") }
            last_was_key = true
        elseif akey ~= nil then
            active[akey:lower()] = trim(aval)
        elseif line:match("^#") then
            local c = line:match("^#%s?(.*)$")
            if c ~= nil and not c:match("^=+$") then
                if preamble ~= nil then
                    if c == "" then
                        -- empty comment closes the section preamble
                        if #preamble > 0 and current ~= nil then current.text = table.concat(preamble, "\n") end
                        preamble = nil
                    else
                        preamble[#preamble + 1] = c
                    end
                elseif c ~= "" then
                    if last_was_key then
                        desc = {}
                        last_was_key = false
                    end
                    desc[#desc + 1] = c
                end
            end
        end
    end
    return sections, active
end

local function write_commands()
    local lines = {}
    for k, v in pairs(pending_user) do
        lines[#lines + 1] = (v == false) and ("-" .. k) or (k .. "=" .. v)
    end
    for k, v in pairs(pending_dev) do
        lines[#lines + 1] = (v == false) and ("dev:-" .. k) or ("dev:" .. k .. "=" .. v)
    end
    for _, c in ipairs(once_cmds) do
        lines[#lines + 1] = c
    end
    fs.write(CMD_FILE, table.concat(lines, "\n") .. "\n")
    once_cmds = {}
end

local function queue(layer, key, value)
    if layer == "dev" then pending_dev[key] = value else pending_user[key] = value end
    write_commands()
end

local function fire(cmd)
    once_cmds[#once_cmds + 1] = cmd
    write_commands()
end

local function fmt_num(n, as_int)
    if as_int then return string.format("%d", math.floor(n + 0.5)) end
    return string.format("%.6g", n)
end

local function refresh()
    frame = frame + 1
    if (frame % 60) ~= 1 then return end
    -- command file gone = the plugin consumed it; the mirrors refresh in the same poll
    if (next(pending_user) ~= nil or next(pending_dev) ~= nil) and fs.read(CMD_FILE) == "" then
        pending_user = {}
        pending_dev = {}
    end
    local _
    _, user_over  = parse_file(fs.read(USER_FILE))
    _, calib_over = parse_file(fs.read(CALIB_FILE))
    dev_catalog, dev_over = parse_file(fs.read(DEV_FILE))
    if dev_catalog ~= nil and #dev_catalog == 0 then dev_catalog = nil end
    if user_catalog == nil then
        local s = parse_file(fs.read(REF_FILE))
        if #s > 0 then user_catalog = s end
    end
    local status = fs.read(STATUS_FILE)
    calib_mode = tonumber(status:match("calibmode=(%d+)") or "0") or 0
end

-- current value as a STRING: queued, else the file's override, else the catalog default
local function effective(entry, layer)
    local pend = (layer == "dev") and pending_dev or pending_user
    local over = (layer == "dev") and dev_over or user_over
    local p = pend[entry.key]
    if p ~= nil and p ~= false then return p, true end
    if p == false then return entry.default, false end
    local o = over[entry.key]
    if o ~= nil then return o, true end
    return entry.default, false
end

-- ---------------------------------------------------------------- text wrapping
-- imgui.text does not wrap, and this build's Lua bindings expose no wrapped-text call, so long
-- lines clip at the panel edge. Wrap to the panel's MEASURED width instead: each source line is
-- kept as its own line (so aligned value tables survive), and only over-long lines are
-- soft-wrapped at word boundaries with a two-space hanging indent.
local char_w = nil
local function avail_cols()
    if char_w == nil then
        local ok, w = pcall(function()
            local probe = "The quick brown fox jumps over the lazy dog, 0123456789 -- (measure)"
            return imgui.calc_text_size(probe).x / 69
        end)
        char_w = (ok and w ~= nil and w > 0) and w or 8
    end
    local ok, cols = pcall(function()
        local avail = imgui.get_window_size().x - imgui.get_cursor_pos().x - 24
        return math.floor(avail / char_w)
    end)
    if not ok or cols == nil or cols < 24 then return 24 end
    return cols
end

-- Lines that must keep their own line: indented text, numbered/dashed list items, banners.
local function is_verbatim(line)
    return line:match("^%s") ~= nil or line:match("^%d+%s*[=%.%)]") ~= nil
        or line:match("^[-%*>]") ~= nil
end

local function print_text_block(s)
    if s == nil or s == "" then return end
    -- REFLOW first: the catalog files hard-wrap their prose at file width, which reads as a
    -- sawtooth once re-wrapped to the panel. Consecutive plain prose lines merge into one
    -- paragraph so the panel's own wrap decides every break; verbatim lines stay as they are.
    local units = {}
    local prose = nil
    for line in s:gmatch("[^\n]+") do
        if is_verbatim(line) then
            if prose ~= nil then units[#units + 1] = prose; prose = nil end
            units[#units + 1] = line
        elseif prose == nil then
            prose = line
        else
            prose = prose .. " " .. line
        end
    end
    if prose ~= nil then units[#units + 1] = prose end

    local cols = avail_cols()
    for _, unit in ipairs(units) do
        local line = unit
        -- continuation lines hang under a list item's text, not under its number
        local lead = line:match("^(%s*%d+[%.%)=]%s)") or line:match("^(%s*[-*]%s)")
        local hang = lead ~= nil and string.rep(" ", #lead) or "  "
        while #line > cols do
            local cut = nil
            for i = cols, math.max(1, cols - 40), -1 do
                if line:sub(i, i) == " " then cut = i break end
            end
            if cut == nil or cut < 8 then cut = cols end
            imgui.text(line:sub(1, cut))
            line = hang .. trim(line:sub(cut + 1))
            if trim(line) == "" then line = "" end
        end
        if #line > 0 and trim(line) ~= "" then imgui.text(line) end
    end
end

-- ---------------------------------------------------------------- widgets
-- Binding/button defaults show by NAME in tooltips; everything else shows its raw value.
local function default_label(entry, hint)
    if entry.default == "" then return "(empty)" end
    if hint ~= nil and (hint.t == "key" or hint.t == "btn") then
        local n = tonumber(entry.default)
        if n ~= nil then
            local nm = named_lookup(hint.t == "key" and KEY_NAMES or BTN_NAMES, math.floor(n))
            if nm ~= nil then return nm end
        end
    end
    return entry.default
end

local function draw_entry(entry, layer)
    local key = entry.key
    local hint = HINTS[key]
    if layer == "dev" then hint = nil end   -- dev knobs render generically; hints are player UX

    imgui.push_id(layer .. ":" .. key)

    if hint ~= nil and hint.hide then
        imgui.pop_id()
        return
    end
    if hint ~= nil and hint.textdesc then
        print_text_block(entry.desc)
    end

    local cur, overridden = effective(entry, layer)
    local num = tonumber(cur)

    if hint ~= nil and hint.t == "bool" then
        local changed, v = imgui.checkbox(key, (num or 0) ~= 0)
        if changed then queue(layer, key, v and "1" or "0") end
    elseif hint ~= nil and hint.t == "slider" and hint.int then
        local changed, v = imgui.slider_int(key, math.floor((num or 0) + 0.5), hint.min, hint.max)
        if changed then queue(layer, key, fmt_num(v, true)) end
    elseif hint ~= nil and hint.t == "slider" then
        local changed, v = imgui.slider_float(key, num or 0, hint.min, hint.max)
        if changed then queue(layer, key, fmt_num(v, false)) end
    elseif hint ~= nil and hint.t == "drag" and hint.int then
        local changed, v = imgui.drag_int(key, math.floor((num or 0) + 0.5), 1, hint.min, hint.max)
        if changed then queue(layer, key, fmt_num(v, true)) end
    elseif hint ~= nil and hint.t == "drag" then
        local speed = hint.speed or ((hint.max - hint.min) / 250)
        local changed, v = imgui.drag_float(key, num or 0, speed, hint.min, hint.max)
        if changed then queue(layer, key, fmt_num(v, false)) end
    elseif hint ~= nil and hint.t == "enum" then
        local idx = 1
        for i, val in ipairs(hint.values) do
            if val == math.floor((num or 0) + 0.5) then idx = i end
        end
        local changed, sel = imgui.combo(key, idx, hint.items)
        if changed and hint.values[sel] ~= nil then queue(layer, key, fmt_num(hint.values[sel], true)) end
    elseif hint ~= nil and hint.t == "choice" then
        local idx = 1
        for i, s in ipairs(hint.items) do
            if s == cur then idx = i end
        end
        local changed, sel = imgui.combo(key, idx, hint.items)
        if changed and hint.items[sel] ~= nil then queue(layer, key, hint.items[sel]) end
    elseif hint ~= nil and (hint.t == "key" or hint.t == "btn") then
        -- named dropdown: the file stores a numeric code, the player picks a name
        local list = (hint.t == "key") and KEY_NAMES or BTN_NAMES
        local v = math.floor((num or 0) + 0.5)
        local items = {}
        local idx = nil
        for i, e in ipairs(list) do
            items[i] = e[2]
            if e[1] == v then idx = i end
        end
        if idx == nil then
            items[#items + 1] = "(custom value from the file)"
            idx = #items
        end
        local changed, sel = imgui.combo(key, idx, items)
        if changed and list[sel] ~= nil then
            queue(layer, key, string.format("0x%04X", list[sel][1]))
        end
    elseif (hint ~= nil and (hint.t == "hex" or hint.t == "text")) or num == nil then
        -- buffered text: commit with the Set button so half-typed values never reach the file
        local bkey = layer .. ":" .. key
        local buf = editbuf[bkey]
        if buf == nil then buf = cur end
        local changed, v = imgui.input_text(key, buf, 0)
        if changed then editbuf[bkey] = v end
        imgui.same_line()
        if imgui.small_button("set") then
            local text = trim(editbuf[bkey] or cur)
            if hint ~= nil and hint.t == "hex" then
                local n = tonumber(text)
                if n ~= nil then queue(layer, key, string.format("0x%04X", math.floor(n))) end
            else
                queue(layer, key, text)
            end
            editbuf[bkey] = nil
        end
    else
        -- no hint (dev knobs, or a key added by a newer release): generic numeric widget
        local is_int = (cur:match("^-?%d+$") ~= nil)
        if is_int then
            local changed, v = imgui.drag_int(key, math.floor(num + 0.5), 1, -1000000, 1000000)
            if changed then queue(layer, key, fmt_num(v, true)) end
        else
            local changed, v = imgui.drag_float(key, num, 0.01, -1000000.0, 1000000.0)
            if changed then queue(layer, key, fmt_num(v, false)) end
        end
    end

    if not (hint ~= nil and hint.textdesc) and imgui.is_item_hovered() then
        local tip = entry.desc
        if tip == nil or tip == "" then tip = key end
        imgui.set_tooltip(tip .. "\n\ndefault: " .. default_label(entry, hint))
    end

    if overridden then
        imgui.same_line()
        if imgui.small_button("x") then
            queue(layer, key, false)
            editbuf[layer .. ":" .. key] = nil
        end
        if imgui.is_item_hovered() then
            imgui.set_tooltip("Remove this override -> back to the default (" ..
                default_label(entry, hint) .. ")")
        end
    end

    imgui.pop_id()
end

local function draw_catalog(catalog, layer)
    for _, section in ipairs(catalog) do
        if imgui.collapsing_header(section.name) then
            imgui.indent(8)
            print_text_block(section.text)
            for _, entry in ipairs(section.keys) do
                draw_entry(entry, layer)
            end
            imgui.unindent(8)
            imgui.spacing()
        end
    end
end

-- ---------------------------------------------------------------- panels
local function draw_user()
    if user_catalog == nil then
        print_text_block("Waiting for halo_vr.dll to publish the settings catalog.\n(data/" ..
                         REF_FILE .. " missing -- is the halo_vr plugin loaded and current?)")
        return
    end
    print_text_block("Changes save to halo_vr_user.cfg and apply live within ~2 s. " ..
                     "Overridden settings show an 'x' button: press it to return to the default.")
    imgui.spacing()
    draw_catalog(user_catalog, "user")
end

local function draw_dev()
    print_text_block("*** WARNING *** These are internal research and debugging knobs, primarily " ..
                     "for development. Wrong values can tank framerate (nausea), break aim, or " ..
                     "invalidate a calibration. Change something only if you know exactly what you " ..
                     "are doing, or troubleshooting asked you to. Changes save to halo_vr_dev.cfg, " ..
                     "which mod updates OVERWRITE -- deliberately, so experiments cannot linger. " ..
                     "Keys tagged [dev build] do nothing on a normal (release) build of the mod.")
    imgui.spacing()
    if dev_catalog == nil then
        print_text_block("Waiting for the dev catalog mirror (data/" .. DEV_FILE .. ").")
        return
    end
    draw_catalog(dev_catalog, "dev")
end

local function any_key_active(keys)
    for _, k in ipairs(keys) do
        if calib_over[k] ~= nil then return true end
    end
    return false
end

local function draw_calib()
    print_text_block("The shipped fit was measured on Quest Touch controllers; calibrating " ..
                     "writes YOUR fit to halo_vr_calib.cfg, which overrides it and survives updates.")
    imgui.spacing()
    print_text_block("RECOMMENDED: the KEYBOARD gestures. Hold End (pose match) or Page Down " ..
                     "(aim ray), align, release to save. Holding the key IS the calibration state " ..
                     "-- nothing stays armed, nothing to cancel, no ambiguity about whether you " ..
                     "are calibrating.")
    print_text_block("NOTE: the keys only register while the GAME WINDOW IS FOCUSED (background " ..
                     "keystrokes are deliberately ignored -- click the game window first).")
    imgui.spacing()
    print_text_block("No keyboard in reach? The buttons below arm the same gestures:")
    print_text_block("  1. Press a Calibrate button, then CLOSE this menu -- controller input " ..
                     "does not reach the game (or the mod) while the UEVR menu is open.\n" ..
                     "  2. Align: pose match = hold your controller on the on-screen weapon; " ..
                     "aim ray = point your controller at the frozen reticle.\n" ..
                     "  3. RIGHT trigger = save & finish. LEFT trigger = save & re-arm on " ..
                     "release, for consecutive passes. Triggers will not fire your weapon while " ..
                     "armed.")
    print_text_block("The mode STAYS ARMED until a right-trigger save or Cancel -- if you get " ..
                     "distracted, check back here for the ARMED banner.")
    imgui.spacing()

    if calib_mode ~= 0 then
        print_text_block(">>> ARMED: " .. (calib_mode == 1 and "WEAPON POSE (hold controller on the weapon)"
                                                            or "AIM RAY (point at the frozen reticle)") ..
                         "\n>>> Close this menu, align, then use the triggers.")
        if imgui.button("Cancel calibration mode") then fire("calib:off") end
        imgui.spacing()
    end

    if imgui.button("Use shipped calibration for EVERYTHING") then fire("calibreset:all") end
    if imgui.is_item_hovered() then
        imgui.set_tooltip("Deletes halo_vr_calib.cfg (this hand) -- the shipped Quest Touch fit\n" ..
                          "applies again within ~2 s. Your own fit is gone; recalibrate to redo it.")
    end
    imgui.spacing()

    imgui.push_id("calpose")
    if imgui.button("Calibrate weapon pose (End)") then fire("calib:pose") end
    if imgui.is_item_hovered() then
        imgui.set_tooltip("How the weapon sits in your hand. Hold the controller so it lines up\n" ..
                          "with the on-screen weapon, then save with the trigger.")
    end
    if any_key_active(POSE_KEYS) then
        imgui.same_line()
        if imgui.small_button("x") then fire("calibreset:pose") end
        if imgui.is_item_hovered() then
            imgui.set_tooltip("Your pose calibration is active. Remove it -> back to the shipped fit.")
        end
    end
    imgui.pop_id()

    imgui.push_id("calaim")
    if imgui.button("Calibrate aim ray (Page Down)") then fire("calib:aim") end
    if imgui.is_item_hovered() then
        imgui.set_tooltip("Where shots go relative to the weapon. The reticle freezes; point your\n" ..
                          "controller at it, then save with the trigger.")
    end
    if any_key_active(AIM_KEYS) then
        imgui.same_line()
        if imgui.small_button("x") then fire("calibreset:aim") end
        if imgui.is_item_hovered() then
            imgui.set_tooltip("Your aim calibration is active. Remove it -> back to the shipped fit.")
        end
    end
    imgui.pop_id()
end

uevr.sdk.callbacks.on_draw_ui(function()
    local ok, err = pcall(function()
        refresh()
        if imgui.collapsing_header("Halo VR User Settings") then
            imgui.indent(4)
            draw_user()
            imgui.unindent(4)
        end
        if imgui.collapsing_header("Halo VR DEV Settings") then
            imgui.indent(4)
            draw_dev()
            imgui.unindent(4)
        end
        if imgui.collapsing_header("Halo VR Calibration") then
            imgui.indent(4)
            draw_calib()
            imgui.unindent(4)
        end
    end)
    if not ok then
        imgui.text("halo_vr_settings.lua error: " .. tostring(err))
    end
end)
