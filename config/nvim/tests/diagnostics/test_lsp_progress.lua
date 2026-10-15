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
  thresholds = { lsp_progress_ms = 50 },
}
T.ok(diag.start_collector "lsp", "lsp collector failed to start")

local alive = { [55] = true }
local real_get = vim.lsp.get_client_by_id
vim.lsp.get_client_by_id = function(id)
  if alive[id] then return { name = "clangd" } end
  return real_get(id)
end

---@param kind "begin"|"report"|"end"
---@param token string
---@param title string|nil
local function progress(kind, token, title)
  vim.api.nvim_exec_autocmds("LspProgress", {
    pattern = kind,
    modeline = false,
    data = {
      client_id = 55,
      params = { token = token, value = { kind = kind, title = title } },
    },
  })
end

T.test("long progress work is timed begin to end", function()
  progress("begin", "idx", "Indexing")
  progress("report", "idx")
  T.busy(100)
  progress("end", "idx")
  local evs = T.queued "lsp_progress"
  T.eq(#evs, 1)
  T.ok(evs[1].ms >= 100, "ms " .. evs[1].ms)
  T.eq(evs[1].data.client, "clangd")
  T.eq(evs[1].data.title, "Indexing")
end)

T.test("short progress work is not logged", function()
  progress("begin", "quick", "Loading")
  progress("end", "quick")
  T.eq(#T.queued "lsp_progress", 1)
end)

T.test("end without begin is ignored", function()
  progress("end", "never-began")
  T.eq(#T.queued "lsp_progress", 1)
end)

T.test("progress left open by a dead server does not block tracking", function()
  alive[57] = true
  for i = 1, 100 do
    vim.api.nvim_exec_autocmds("LspProgress", {
      pattern = "begin",
      modeline = false,
      data = {
        client_id = 57,
        params = {
          token = "orphan" .. i,
          value = { kind = "begin", title = "x" },
        },
      },
    })
  end
  alive[57] = nil -- server died mid-indexing, ends never arrive
  require("diagnostics.collectors.lsp").sweep()
  progress("begin", "after", "Reindex")
  T.busy(60)
  progress("end", "after")
  T.eq(#T.queued "lsp_progress", 2)
end)

T.test("detach of a client that is gone logs lsp_exit", function()
  alive[55] = nil
  vim.api.nvim_exec_autocmds(
    "LspDetach",
    { modeline = false, data = { client_id = 55 } }
  )
  T.ok(
    vim.wait(200, function() return #T.queued "lsp_exit" > 0 end, 10),
    "no lsp_exit"
  )
  T.eq(T.queued("lsp_exit")[1].data.client, "clangd")
end)

T.test("requested stop (:LspRestart, :LspStop) is not an exit", function()
  local stopping = { name = "clangd", _is_stopping = true }
  vim.lsp.get_client_by_id = function(id)
    if id == 58 then return stopping end
    if alive[id] then return { name = "clangd" } end
    return real_get(id)
  end
  vim.api.nvim_exec_autocmds(
    "LspDetach",
    { modeline = false, data = { client_id = 58 } }
  )
  stopping = nil -- runtime removes the client after detach
  vim.wait(200)
  T.eq(#T.queued "lsp_exit", 1)
end)

T.test("detach while the client lives on is not an exit", function()
  alive[56] = true
  vim.api.nvim_exec_autocmds(
    "LspDetach",
    { modeline = false, data = { client_id = 56 } }
  )
  vim.wait(100)
  T.eq(#T.queued "lsp_exit", 1)
end)

T.done()
