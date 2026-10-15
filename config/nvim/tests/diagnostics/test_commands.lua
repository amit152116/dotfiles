local T = dofile "tests/diagnostics/helpers.lua"

require("diagnostics").setup {
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
  },
}

T.test("all commands registered", function()
  local cmds = vim.api.nvim_get_commands {}
  for _, name in ipairs {
    "DiagReport",
    "DiagOpen",
    "DiagClear",
    "DiagToggle",
    "DiagStatus",
  } do
    T.ok(cmds[name], name .. " missing")
  end
end)

T.test("DiagReport rejects malformed args without opening a buffer", function()
  local calls = T.capture_notify()
  local before = #vim.api.nvim_list_bufs()
  for _, args in ipairs {
    "--since 5x",
    "--since",
    "stray",
    "--session=yesterday",
  } do
    vim.cmd("DiagReport " .. args)
  end
  local own = T.own(calls)
  T.eq(#own, 4)
  for _, call in ipairs(own) do
    T.eq(call.level, vim.log.levels.ERROR)
  end
  T.eq(#vim.api.nvim_list_bufs(), before)
end)

T.test("DiagReport opens a report buffer with unflushed events", function()
  require("diagnostics.writer").emit(
    "stall",
    300,
    { buf = "x", ft = "lua", mode = "n", recent = {} }
  )
  vim.cmd "DiagReport --since all"
  local lines = vim.api.nvim_buf_get_lines(0, 0, -1, false)
  T.eq(lines[1], "# Nvim diagnostics")
  T.ok(table.concat(lines, "\n"):find("## Stalls (1)", 1, true), "stall shown")
  T.eq(vim.bo.buftype, "nofile")
  vim.cmd "close"
end)

T.test("DiagStatus reports writer state, DiagToggle rejects unknown", function()
  local calls = T.capture_notify()
  vim.cmd "DiagStatus"
  vim.cmd "DiagToggle nope"
  local own = T.own(calls)
  T.ok(own[1].msg:find("queued", 1, true), "status text")
  T.eq(own[2].level, vim.log.levels.ERROR)
end)

T.done()
