local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

local calls = T.capture_notify()
diag.setup {
  log_file = T.tmpdir() .. "/d.jsonl",
  flush_ms = 60000, -- keep events in the queue so T.queued sees them
  collectors = {
    startup = false,
    stall = false,
    bigfile = false,
    lsp = false,
    notify = false,
    profile = false,
    memory = false,
    input = false,
    autoprofile = false,
  },
}
local capture = require "diagnostics.capture"
local profile = require "diagnostics.collectors.profile"

local function edge_hot_loop()
  local x = 0
  for i = 1, 3e7 do
    x = x + i % 3
  end
  return x
end

---@return string
local function last_file_text()
  local evs = T.queued "profile_capture"
  local f = assert(io.open(evs[#evs].data.file))
  local text = f:read "*a"
  f:close()
  return text
end

T.test("a GC during the second capture does not end it early", function()
  -- jit.p's finalizer stops whichever capture is current, not its own
  T.ok(capture.start { backend = "jit" })
  T.ok(capture.stop())
  T.ok(capture.start { backend = "jit" })
  collectgarbage()
  edge_hot_loop()
  T.ok(capture.stop())
  local text = last_file_text()
  T.eq(text:find("No samples", 1, true), nil)
  T.ok(text:find("test_capture_edges.lua", 1, true), text)
end)

T.test("captures within the same second get separate files", function()
  T.ok(capture.start { backend = "jit" })
  T.ok(capture.stop())
  T.ok(capture.start { backend = "jit" })
  T.ok(capture.stop())
  local evs = T.queued "profile_capture"
  T.ok(evs[#evs].data.file ~= evs[#evs - 1].data.file, "same file reused")
end)

T.test("starting the profile collector mid-capture is deferred", function()
  T.ok(diag.start_collector "profile")
  T.ok(capture.start { backend = "jit" })
  diag.stop_collector "profile" -- no-op: capture paused it
  T.eq(diag.start_collector "profile", false)
  T.eq(diag.is_running "profile", false)
  T.ok(capture.stop())
  T.ok(diag.is_running "profile", "not resumed")
  -- the collector must really be sampling again, not just flagged as running
  local t0 = vim.uv.hrtime()
  edge_hot_loop()
  T.ok(#profile.top(t0, vim.uv.hrtime(), 1) > 0, "no samples after resume")
end)

T.test("an error from jit.p.stop still resumes and logs", function()
  local real = require "jit.p"
  package.loaded["jit.p"] = {
    start = real.start,
    stop = function()
      real.stop()
      error "disk full"
    end,
  }
  T.ok(capture.start { backend = "jit" })
  local ok = capture.stop()
  package.loaded["jit.p"] = real
  T.eq(ok, true)
  T.eq(capture.running(), false)
  T.ok(diag.is_running "profile", "profile collector left off")
  local evs = T.queued "profile_capture"
  T.ok(evs[#evs].data.error:find("disk full", 1, true), "error not logged")
end)

T.test("snacks backend refuses a profile the user already started", function()
  local stops = 0
  package.loaded["snacks.profiler"] = {
    start = function() end,
    stop = function() stops = stops + 1 end,
    running = function() return true end,
  }
  local ok, err = capture.start { backend = "snacks" }
  T.eq(ok, false)
  T.ok(err:find("already running", 1, true), err)
  T.eq(capture.running(), false)
  T.eq(stops, 0)
end)

T.test("snacks start that throws is refused cleanly", function()
  package.loaded["snacks.profiler"] = {
    start = function() error "config not ready" end,
    stop = function() end,
    running = function() return false end,
  }
  local ok, err = capture.start { backend = "snacks" }
  T.eq(ok, false)
  T.ok(err:find("config not ready", 1, true), err)
  T.eq(capture.running(), false)
  package.loaded["snacks.profiler"] = nil
end)

T.done()
