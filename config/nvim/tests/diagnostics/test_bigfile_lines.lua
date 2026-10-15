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
  thresholds = { bigfile_bytes = 1024 * 1024, bigfile_line_length = 1000 },
}
T.ok(diag.start_collector "bigfile", "bigfile collector failed to start")

local dir = T.tmpdir()

---@param name string
---@param content string
---@return string path
local function write(name, content)
  local path = dir .. "/" .. name
  local f = assert(io.open(path, "w"))
  f:write(content)
  f:close()
  return path
end

T.test("small minified file with a long line is reported", function()
  vim.cmd.edit(write("min.json", string.rep("a", 5000) .. "\n"))
  local evs = T.queued "bigfile"
  T.eq(#evs, 1)
  T.eq(evs[1].data.max_line_len, 5000)
  T.eq(evs[1].data.line_len_sampled, false)
end)

T.test("short-line small file is still ignored", function()
  vim.cmd.edit(write("plain.txt", string.rep("short line\n", 50)))
  T.eq(#T.queued "bigfile", 1)
end)

T.test("many-line file samples line lengths", function()
  vim.cmd.edit(write("long.log", string.rep("abcdefg\n", 262144)))
  local ev = T.queued("bigfile")[2]
  T.eq(ev.data.line_len_sampled, true)
  T.eq(ev.data.max_line_len, 7)
end)

T.test("line lengths are measured without copying lines", function()
  local bigfile = require "diagnostics.collectors.bigfile"
  local buf = vim.api.nvim_create_buf(false, true)
  vim.api.nvim_buf_set_lines(
    buf,
    0,
    -1,
    false,
    { string.rep("z", 20 * 1024 * 1024) }
  )
  collectgarbage "collect"
  collectgarbage "stop"
  local before = collectgarbage "count"
  local max_len = bigfile._max_line_len(buf, 1)
  local grew_kb = collectgarbage "count" - before
  collectgarbage "restart"
  T.eq(max_len, 20 * 1024 * 1024)
  T.ok(grew_kb < 1024, string.format("heap grew %.0f KB", grew_kb))
  vim.api.nvim_buf_delete(buf, { force = true })
end)

T.test("lengths are right for noeol and dos files", function()
  local bigfile = require "diagnostics.collectors.bigfile"
  vim.cmd.edit(write("noeol.txt", "ab\nabcdef"))
  local buf = vim.api.nvim_get_current_buf()
  T.eq(bigfile._max_line_len(buf, vim.api.nvim_buf_line_count(buf)), 6)
  vim.cmd.edit(write("dos.txt", "abc\r\nabcd\r\n"))
  buf = vim.api.nvim_get_current_buf()
  T.eq(vim.bo[buf].fileformat, "dos")
  T.eq(bigfile._max_line_len(buf, vim.api.nvim_buf_line_count(buf)), 4)
end)

T.test("attach reports which mitigations engaged", function()
  local buf = vim.fn.bufnr(dir .. "/long.log")
  vim.b[buf].large_buf = true
  -- bigfile.nvim marks every checked buffer, 0 when it is not big
  vim.b[vim.fn.bufnr(dir .. "/min.json")].bigfile_detected = 0
  T.ok(
    vim.wait(1500, function() return #T.queued "bigfile_attach" >= 2 end, 20),
    "no attach events"
  )
  local attach = T.queued("bigfile_attach")[2].data
  T.eq(attach.mitigations, { "large_buf" })
  local first = T.queued("bigfile_attach")[1].data
  T.eq(first.mitigations, {})
end)

T.done()
