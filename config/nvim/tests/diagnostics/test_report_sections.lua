local T = dofile "tests/diagnostics/helpers.lua"
local report = require "diagnostics.report"

local ts = 1790000000
---@param kind string
---@param ms number|nil
---@param data table|nil
---@param sid string|nil
---@return table
local function ev(kind, ms, data, sid)
  ts = ts + 1
  return { v = 1, ts = ts, sid = sid or "s1", kind = kind, ms = ms, data = data }
end

---@param events table[]
---@return string
local function render(events)
  return table.concat(report.render(events, { skipped = 0, dropped = 0 }), "\n")
end

local function has(text, needle)
  T.ok(text:find(needle, 1, true), "missing: " .. needle)
end

T.test(
  "startup regression flagged against the median of earlier runs",
  function()
    local events = {}
    for _, ms in ipairs { 100, 100, 100, 100, 150 } do
      events[#events + 1] = ev("startup", ms, { available = true })
    end
    has(render(events), "regression: latest 150.0 ms vs median 100.0 ms (+50%)")
    events[#events].ms = 110
    T.eq(render(events):find("regression", 1, true), nil)
  end
)

T.test("stall rows show the hot frame", function()
  local text = render {
    ev("stall", 450, {
      buf = "a.lua",
      ft = "lua",
      mode = "n",
      recent = {},
      stacks = {
        {
          stack = "/home/u/.local/share/nvim/lazy/lspsaga.nvim/lua/lspsaga/hover.lua:212;/x/y.lua:3;",
          count = 18,
          vm = "I",
        },
      },
    }),
  }
  has(text, "at lspsaga.nvim/lua/lspsaga/hover.lua:212 (18 samples, I)")
end)

T.test("runtime helper frames are skipped for the hot frame", function()
  local text = render {
    ev("stall", 400, {
      mode = "n",
      recent = {},
      profiled = true,
      stacks = {
        {
          stack = "/usr/local/share/nvim/runtime/lua/vim/_editor.lua:10;"
            .. "/home/u/.local/share/nvim/lazy/foo.nvim/lua/foo.lua:5;",
          count = 9,
          vm = "I",
        },
      },
    }),
  }
  has(text, "at foo.nvim/lua/foo.lua:5 (9 samples, I)")
end)

T.test("profiled stall without samples is called out as waiting", function()
  local text =
    render { ev("stall", 700, { mode = "n", recent = {}, profiled = true }) }
  has(text, "no Lua samples: blocked waiting (I/O, job, sync request), not CPU")
end)

T.test("deferred stall stacks are marked as uncertain", function()
  local text = render {
    ev("stall", 600, {
      mode = "n",
      recent = {},
      stacks = {
        { stack = "a/b/next.lua:1;", count = 3, vm = "C", deferred = true },
      },
    }),
  }
  has(
    text,
    "at a/b/next.lua:1 (3 samples, C; blocked outside Lua, may be the next code run)"
  )
end)

T.test("big files show line length and mitigation", function()
  local text = render {
    ev("bigfile", 900, {
      buf = 4,
      file = "/t/min.json",
      size = 5001,
      lines = 1,
      max_line_len = 5000,
    }),
    ev("bigfile_attach", nil, {
      buf = 4,
      ft = "json",
      clients = {},
      treesitter = true,
      mitigations = {},
    }),
  }
  has(text, "maxline=5000")
  has(text, "mitigation=none")
  has(text, "1 big-file open(s) with no mitigation engaged")
end)

T.test("lsp progress and exits are listed", function()
  local text = render {
    ev("lsp_progress", 48000, { client = "clangd", title = "Indexing" }),
    ev("lsp_exit", nil, { client = "clangd" }),
  }
  has(text, "## LSP (2)")
  has(text, "progress  clangd  Indexing  48.0 s")
  has(text, "exited  clangd")
end)

T.test("memory section summarizes sessions", function()
  local text = render {
    ev("memory", nil, { growth_mb = 100, rss_mb = 300, heap_mb = 50 }),
    ev("memory", nil, { growth_mb = 200, rss_mb = 400, heap_mb = 60 }),
    ev("session_summary", nil, {
      duration_s = 720,
      utime_s = 30.5,
      stime_s = 2,
      maxrss_mb = 410,
      majflt = 0,
      nivcsw = 5,
      rss_mb = 400,
      heap_mb = 60,
    }),
  }
  has(text, "## Memory (1)")
  has(text, "session s1  12 min  cpu 32.5 s  maxrss 410.0 MB  growth events 2")
end)

T.test(
  "input latency section aggregates summaries per mode and filetype",
  function()
    local text = render {
      ev(
        "input_summary",
        nil,
        { mode = "i", ft = "lua", count = 10, p50 = 5, p95 = 60, max = 120 }
      ),
      ev(
        "input_summary",
        nil,
        { mode = "i", ft = "lua", count = 5, p50 = 4, p95 = 40, max = 70 }
      ),
      ev("input_latency", 120, { mode = "i", ft = "lua" }),
      ev("input_latency", 70, { mode = "i", ft = "lua" }),
    }
    has(text, "## Input latency (2)")
    has(text, "- i lua  keys=15  worst p95=60 ms  max=120 ms  spikes=2")
  end
)

T.done()
