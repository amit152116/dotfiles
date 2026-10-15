local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

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
  },
  thresholds = { bigfile_bytes = 1024 * 1024 },
}
T.ok(diag.start_collector "bigfile", "bigfile collector failed to start")

local dir = T.tmpdir()

---@param name string
---@param lines integer each line is 8 bytes
---@return string path
local function make(name, lines)
  local path = dir .. "/" .. name
  local f = assert(io.open(path, "w"))
  f:write(string.rep("abcdefg\n", lines))
  f:close()
  return path
end

T.test("big file read is timed with metadata", function()
  vim.cmd.edit(make("big.log", 262144)) -- 2 MiB
  T.ok(
    vim.wait(1500, function() return #T.queued "bigfile_attach" > 0 end, 20),
    "no attach event"
  )
  local ev = T.queued("bigfile")[1]
  T.eq(ev.data.size, 2097152)
  T.eq(ev.data.size_source, "stat")
  T.eq(ev.data.lines, 262144)
  T.ok(ev.ms > 0, "read time recorded")
  local attach = T.queued("bigfile_attach")[1]
  T.eq(attach.data.buf, ev.data.buf)
  T.eq(attach.data.treesitter, false)
  T.eq(attach.data.clients, {})
end)

T.test("small file is ignored", function()
  vim.cmd.edit(make("small.txt", 2))
  vim.wait(700)
  T.eq(#T.queued "bigfile", 1)
end)

T.test("unnamed and special buffers are ignored", function()
  local scratch = vim.api.nvim_create_buf(false, true)
  vim.api.nvim_exec_autocmds("BufReadPre", { buffer = scratch })
  vim.api.nvim_exec_autocmds("BufReadPost", { buffer = scratch })
  local named = vim.api.nvim_create_buf(false, true)
  vim.api.nvim_buf_set_name(named, dir .. "/special.txt")
  vim.api.nvim_exec_autocmds("BufReadPre", { buffer = named })
  vim.api.nvim_exec_autocmds("BufReadPost", { buffer = named })
  vim.wait(700)
  T.eq(#T.queued "bigfile", 1)
  T.ok(diag.is_running "bigfile", "no error disabled it")
end)

T.test("unstattable file is logged as size unavailable", function()
  local buf = vim.api.nvim_create_buf(true, false)
  vim.api.nvim_buf_set_name(buf, dir .. "/vanished.txt")
  vim.api.nvim_exec_autocmds("BufReadPre", { buffer = buf })
  vim.api.nvim_exec_autocmds("BufReadPost", { buffer = buf })
  local evs = T.queued "bigfile"
  T.eq(#evs, 2)
  T.eq(evs[2].data.size_source, "unavailable")
  T.eq(evs[2].data.size, nil)
end)

T.done()
