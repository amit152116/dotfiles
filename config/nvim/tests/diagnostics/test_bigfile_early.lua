local T = dofile "tests/diagnostics/helpers.lua"

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

-- plugin/diagnostics.lua does this at load, before argv files are read
require("diagnostics.collectors.bigfile").capture_early()
vim.cmd.edit(make("argv-big.log", 262144)) -- 2 MiB, read before setup
vim.cmd.edit(make("argv-small.txt", 2))

require("diagnostics").setup {
  log_file = T.tmpdir() .. "/d.jsonl",
  flush_ms = 60000, -- keep events in the queue so T.queued sees them
  collectors = {
    startup = false,
    stall = false,
    bigfile = true,
    lsp = false,
    notify = false,
    profile = false,
    memory = false,
    input = false,
  },
  thresholds = { bigfile_bytes = 1024 * 1024 },
}

T.test("big file read before setup is reported with its read time", function()
  T.ok(
    vim.wait(1500, function() return #T.queued "bigfile_attach" > 0 end, 20),
    "no attach event"
  )
  local evs = T.queued "bigfile"
  T.eq(#evs, 1)
  T.ok(evs[1].data.file:find("argv-big.log", 1, true), "right file")
  T.eq(evs[1].data.size, 2097152)
  T.ok(evs[1].ms > 0, "read time kept")
end)

T.test("early capture stops once setup ran", function()
  -- group deleted, so looking it up errors
  T.eq(
    pcall(vim.api.nvim_get_autocmds, { group = "Diagnostics_bigfile_early" }),
    false
  )
  vim.cmd.edit(make("later-big.log", 262144))
  T.eq(#T.queued "bigfile", 2)
end)

T.done()
