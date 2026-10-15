local T = dofile "tests/diagnostics/helpers.lua"
local config = require "diagnostics.config"

T.test("defaults apply with no opts", function()
  T.eq(config.setup(), {})
  T.eq(config.get_value "thresholds.stall_ms", 200)
  T.eq(config.get_value "collectors.lsp", true)
  T.eq(config.get_value "max_queue", 2000)
end)

T.test("user values override defaults", function()
  T.eq(config.setup { thresholds = { stall_ms = 500 } }, {})
  T.eq(config.get_value "thresholds.stall_ms", 500)
  T.eq(config.get_value "thresholds.heartbeat_ms", 100)
end)

T.test("invalid values fall back to defaults with errors", function()
  local errors = config.setup {
    log_file = "",
    flush_ms = -1,
    thresholds = { stall_ms = "x", bogus = 1 },
    collectors = { nope = true },
  }
  T.eq(#errors, 5)
  T.eq(config.get_value "thresholds.stall_ms", 200)
  T.eq(config.get_value "thresholds.bogus", nil)
  T.eq(config.get_value "collectors.nope", nil)
  T.eq(config.get_value "flush_ms", 1000)
  T.ok(config.get_value("log_file"):match "diagnostics%.jsonl$")
end)

T.test(
  "get_value on missing path returns nil",
  function() T.eq(config.get_value "thresholds.nothing.deep", nil) end
)

T.done()
