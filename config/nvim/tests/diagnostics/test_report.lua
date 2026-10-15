local T = dofile "tests/diagnostics/helpers.lua"
local report = require "diagnostics.report"

local FIXTURE = "tests/diagnostics/fixtures/sample.jsonl"

T.test("load skips corrupt and unknown-version lines", function()
  local events, stats = report.load(FIXTURE)
  T.eq(#events, 11)
  T.eq(stats.skipped, 2)
end)

T.test(
  "lines with malformed ms or data are skipped, render survives",
  function()
    local path = T.tmpdir() .. "/d.jsonl"
    local f = assert(io.open(path, "w"))
    f:write [[
{"v":1,"ts":1,"sid":"s","kind":"stall","ms":null,"data":{}}
{"v":1,"ts":2,"sid":"s","kind":"stall","ms":"slow","data":{}}
{"v":1,"ts":3,"sid":"s","kind":"stall","ms":5,"data":7}
{"v":1,"ts":4,"sid":"s","kind":"stall","ms":300,"data":{"buf":"ok"}}
]]
    f:close()
    local events, stats = report.load(path)
    T.eq(#events, 1)
    T.eq(stats.skipped, 3)
    stats.dropped = 0
    T.ok(report.render(events, stats), "render returned")
  end
)

T.test("load reads the backup first and appends unflushed lines", function()
  local path = T.tmpdir() .. "/d.jsonl"
  local function write(file, ts, kind)
    local f = assert(io.open(file, "a"))
    f:write(vim.json.encode { v = 1, ts = ts, sid = "s", kind = kind } .. "\n")
    f:close()
  end
  write(path .. ".1", 1, "old")
  write(path, 2, "new")
  local extra =
    { vim.json.encode { v = 1, ts = 3, sid = "s", kind = "queued" } .. "\n" }
  local kinds = vim.tbl_map(
    function(e) return e.kind end,
    (report.load(path, extra))
  )
  T.eq(kinds, { "old", "new", "queued" })
end)

T.test("filter by session and age", function()
  local events = report.load(FIXTURE)
  T.eq(#report.filter(events, { session = "current" }, "s1"), 4)
  T.eq(#report.filter(events, { session = "last" }, "s2"), 4)
  T.eq(#report.filter(events, {}, "x"), 11)
  T.eq(#report.filter(events, { since = 60 }, "x"), 0)
end)

T.test("render has every section with counts", function()
  local events, stats = report.load(FIXTURE)
  stats.dropped = 0
  local text = table.concat(report.render(events, stats), "\n")
  for _, header in ipairs {
    "## Summary (11)",
    "## Startup (1)",
    "## Stalls (1)",
    "## Big files (1)",
    "## LSP (3)",
    "## Notifications (2)",
  } do
    T.ok(text:find(header, 1, true), "missing " .. header)
  end
  T.ok(text:find("42.0 ms  nvim-treesitter", 1, true), "plugin row")
  T.ok(text:find("skipped lines: 2", 1, true), "skipped count")
  T.ok(text:find("50.0 MB", 1, true), "bigfile size")
  T.ok(text:find("ts=false", 1, true), "attach joined")
  T.ok(text:find("clangd  textDocument/hover  n=2", 1, true), "lsp group")
  T.ok(text:find("overdue  clangd  textDocument/references", 1, true))
  T.ok(text:find("2x  ERROR  boom", 1, true), "notify group")
end)

T.test("render with no events says so", function()
  local text =
    table.concat(report.render({}, { skipped = 0, dropped = 0 }), "\n")
  T.ok(text:find("no events yet", 1, true))
end)

T.test("parse_args", function()
  T.eq(report.parse_args {}, { since = 86400 })
  T.eq(report.parse_args { "--since", "30m" }, { since = 1800 })
  T.eq(
    report.parse_args { "--since=all", "--session=last" },
    { session = "last" }
  )
  local opts, err = report.parse_args { "--since", "5x" }
  T.eq(opts, nil)
  T.ok(err:find("since", 1, true))
  T.eq(report.parse_args { "--since" }, nil)
  T.eq(report.parse_args { "stray" }, nil)
  T.eq(report.parse_args { "--session=yesterday" }, nil)
end)

T.test("load and render 20k events under 1s", function()
  local path = T.tmpdir() .. "/big.jsonl"
  local f = assert(io.open(path, "w"))
  for i = 1, 20000 do
    f:write(vim.json.encode {
      v = 1,
      ts = 1790000000 + i,
      sid = "s" .. (i % 5),
      kind = i % 2 == 0 and "stall" or "lsp",
      ms = i % 900,
      data = {
        client = "c",
        method = "m" .. (i % 7),
        buf = "b",
        ft = "lua",
        mode = "n",
        recent = {},
      },
    } .. "\n")
  end
  f:close()
  local start = vim.uv.hrtime()
  local events, stats = report.load(path)
  stats.dropped = 0
  report.render(events, stats)
  local ms = (vim.uv.hrtime() - start) / 1e6
  T.ok(ms < 1000, string.format("took %.0f ms", ms))
end)

T.done()
