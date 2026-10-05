-- Whoop Pilot radio bridge (EdgeTX mixer script), protocol v1
--
-- Lets the Whoop Pilot app on your Mac steer the quad through this radio's own
-- ELRS link. The app talks to this script over the USB-C cable:
--   SYS > Hardware > USB-VCP = LUA, then plug in and pick "USB Serial (VCP)".
--
-- Safety design (read this):
--   * The app only has control while your AI switch is ON *and* your mixes use
--     this script's outputs in REPLACE lines gated by that same switch.
--     Flip the switch OFF = your sticks, instantly, even if this script dies.
--   * Arming is never touched. Your arm switch stays in your hand.
--   * If app packets stop for 0.3 s while engaged, FAILSAFE latches: roll,
--     pitch and yaw go back to your sticks; if the app was flying throttle it
--     descends slowly for 3 s, then idles. Flip the AI switch off/on to clear.
--   * Pushing roll/pitch/yaw past ~30%, or moving the throttle stick ~1/8 of
--     its travel from where it was when you engaged, hands that axis back to
--     you until you flip the AI switch off and on again.

local TIMEOUT = 30        -- 10 ms ticks without a valid packet => link lost
local STICK_OVR = 300     -- roll/pitch/yaw override threshold (of 1024)
local THR_OVR = 256       -- throttle override: movement since engaging (256 = 1/8 of travel)
local TELEM_EVERY = 10    -- 10 ms ticks between telemetry lines (10 Hz)
local DESCEND_DROP = 160  -- failsafe descent: this far below the app's hover estimate
local DESCEND_TIME = 300  -- ticks of gentle descent before cutting (3 s)
local CUT_TIME = 150      -- ticks to ramp from descent throttle to idle (1.5 s)

local inputs = {
  { "Ail", SOURCE }, { "Ele", SOURCE }, { "Thr", SOURCE }, { "Rud", SOURCE },
  { "AI sw", SOURCE }, { "PTT", SOURCE },
}
local outputs = { "AIro", "AIpi", "AIth", "AIya", "AImd" }

local buf = ""
local cmd = { seq = 0, mask = 0, r = 0, p = 0, t = -1024, y = 0, hov = -1024 }
local lastRx = -100000
local engaged, failsafe, hadLink = false, false, false
local fsStart, fsThr = 0, -1024
local thrAtEngage = 0
local ovrR, ovrP, ovrT, ovrY = false, false, false, false
local lastTelem = 0
local rxOk, rxBad = 0, 0

local function clamp(v)
  if v > 1024 then return 1024 end
  if v < -1024 then return -1024 end
  return v
end

local function int(v)
  return math.floor(v + 0.5)
end

-- mask bits: 1 roll, 2 pitch, 4 throttle, 8 yaw (no bit ops in Lua 5.2 without bit32)
local function has(mask, bit)
  return math.floor(mask / bit) % 2 == 1
end

local function checksum(s)
  local sum = 0
  for i = 1, #s do sum = (sum + string.byte(s, i)) % 256 end
  return sum
end

-- "$C,seq,mask,roll,pitch,thr,yaw,hover*CS" (CS = byte sum of the part between $ and *, mod 256, hex)
local function parse(line)
  local body, cs = string.match(line, "^%$(C,[%d%-,]+)%*(%x%x)")
  if not body or checksum(body) ~= tonumber(cs, 16) then return false end
  local f, n = {}, 0
  for v in string.gmatch(body, "%-?%d+") do
    n = n + 1
    f[n] = tonumber(v)
  end
  if n ~= 7 then return false end
  cmd.seq, cmd.mask = f[1], f[2]
  cmd.r, cmd.p, cmd.t, cmd.y, cmd.hov = clamp(f[3]), clamp(f[4]), clamp(f[5]), clamp(f[6]), clamp(f[7])
  return true
end

local function readSerial(now)
  if serialRead == nil then return end
  for _ = 1, 10 do
    local s = serialRead()
    if s == nil or s == "" then return end
    buf = buf .. s
    local nl = string.find(buf, "[\r\n]")
    while nl do
      local line = string.sub(buf, 1, nl - 1)
      buf = string.sub(buf, nl + 1)
      if #line > 0 then
        if parse(line) then
          rxOk = rxOk + 1
          lastRx = now
        else
          rxBad = rxBad + 1
        end
      end
      nl = string.find(buf, "[\r\n]")
    end
    if #buf > 120 then buf = "" end
  end
end

local function sensor(name, scale)
  local v = getValue(name)
  if type(v) ~= "number" then return 0 end
  return int(v * scale)
end

local function flightMode()
  local fm = getValue("FM")
  if type(fm) ~= "string" then return "" end
  return (string.gsub(fm, "[^%w%*!]", ""))
end

local function sendTelemetry(ail, ele, thr, rud, linkOk, ptt)
  if serialWrite == nil then return end
  local flags = (engaged and 1 or 0) + (linkOk and 2 or 0) + (failsafe and 4 or 0) + (ptt and 8 or 0)
    + (ovrR and 16 or 0) + (ovrP and 32 or 0) + (ovrT and 64 or 0) + (ovrY and 128 or 0)
  local body = string.format("T,1,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s",
    cmd.seq, flags, int(ail), int(ele), int(thr), int(rud),
    sensor("RxBt", 100), sensor("RQly", 1), sensor("1RSS", 1),
    sensor("Ptch", 1000), sensor("Roll", 1000), sensor("Yaw", 1000), sensor("Alt", 100),
    rxOk % 10000, rxBad % 10000, flightMode())
  serialWrite(string.format("$%s*%02X\n", body, checksum(body)))
end

-- Throttle while failsafe is latched and the app had been flying throttle.
local function failsafeThrottle(now)
  if fsThr <= -1024 then return -1024 end
  local dt = now - fsStart
  if dt < DESCEND_TIME then return fsThr end
  local k = (dt - DESCEND_TIME) / CUT_TIME
  if k >= 1 then return -1024 end
  return int(fsThr + (-1024 - fsThr) * k)
end

local function run(ail, ele, thr, rud, sw, ptt)
  local now = getTime()
  readSerial(now)
  local linkOk = (now - lastRx) < TIMEOUT

  if sw > 0 and not engaged then
    engaged, failsafe, hadLink = true, false, false
    ovrR, ovrP, ovrT, ovrY = false, false, false, false
    thrAtEngage = thr
    playTone(1400, 60, 20, PLAY_NOW)
    playTone(1900, 60, 0, PLAY_NOW)
  elseif sw <= 0 and engaged then
    engaged, failsafe = false, false
    playTone(900, 120, 0, PLAY_NOW)
  end

  if engaged then
    -- Failsafe only latches if the app had the link during this engagement;
    -- engaging before the app is connected just keeps your sticks in charge.
    if linkOk then hadLink = true end
    if hadLink and not failsafe and not linkOk then
      failsafe = true
      fsStart = now
      if has(cmd.mask, 4) and cmd.t > -900 then
        fsThr = math.max(-1024, math.min(cmd.t, cmd.hov) - DESCEND_DROP)
      else
        fsThr = -1024
      end
      playHaptic(300, 100)
      playTone(600, 400, 100, PLAY_NOW)
    end
    if math.abs(ail) > STICK_OVR then ovrR = true end
    if math.abs(ele) > STICK_OVR then ovrP = true end
    if math.abs(rud) > STICK_OVR then ovrY = true end
    if math.abs(thr - thrAtEngage) > THR_OVR then ovrT = true end
  end

  local outR, outP, outT, outY = ail, ele, thr, rud
  if engaged then
    local m = cmd.mask
    if failsafe then
      if has(m, 4) and not ovrT then outT = failsafeThrottle(now) end
    else
      if has(m, 1) and not ovrR then outR = cmd.r end
      if has(m, 2) and not ovrP then outP = cmd.p end
      if has(m, 4) and not ovrT then outT = cmd.t end
      if has(m, 8) and not ovrY then outY = cmd.y end
    end
  end

  if now - lastTelem >= TELEM_EVERY then
    lastTelem = lastTelem + TELEM_EVERY
    if now - lastTelem > TELEM_EVERY then lastTelem = now end
    sendTelemetry(ail, ele, thr, rud, linkOk, ptt > 0)
  end

  return outR, outP, outT, outY, (engaged and 1024 or -1024)
end

return { input = inputs, output = outputs, run = run }
