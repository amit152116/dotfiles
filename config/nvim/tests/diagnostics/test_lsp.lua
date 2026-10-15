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
  },
  thresholds = { lsp_ms = 50, lsp_overdue_ms = 50 },
}
T.ok(diag.start_collector "lsp", "lsp collector failed to start")
local lsp = require "diagnostics.collectors.lsp"

---@param id integer
---@param kind "pending"|"complete"|"cancel"
---@param method string|nil
local function fire(id, kind, method)
  vim.api.nvim_exec_autocmds("LspRequest", {
    modeline = false,
    data = {
      client_id = 99,
      request_id = id,
      request = {
        type = kind,
        method = method or "textDocument/hover",
        bufnr = 0,
      },
    },
  })
end

T.test("slow completion emits lsp event", function()
  fire(1, "pending")
  T.busy(100)
  fire(1, "complete")
  local ev = T.queued("lsp")[1]
  T.ok(ev and ev.ms >= 100, "slow request recorded")
  T.eq(ev.data.client, "client 99")
  T.eq(ev.data.method, "textDocument/hover")
  T.eq(ev.data.cancelled, nil)
end)

T.test("fast completion emits nothing", function()
  fire(2, "pending")
  fire(2, "complete")
  T.eq(#T.queued "lsp", 1)
end)

T.test("slow cancel is marked cancelled", function()
  fire(3, "pending")
  T.busy(100)
  fire(3, "cancel")
  T.eq(T.queued("lsp")[2].data.cancelled, true)
end)

T.test("overdue request emits once and is dropped", function()
  fire(4, "pending", "textDocument/references")
  T.busy(100)
  lsp.sweep()
  lsp.sweep()
  local overdue = T.queued "lsp_overdue"
  T.eq(#overdue, 1)
  T.eq(overdue[1].data.method, "textDocument/references")
  fire(4, "complete")
  T.eq(#T.queued "lsp", 2)
end)

T.test("pending table is capped at 500", function()
  for id = 100, 650 do
    fire(id, "pending")
  end
  T.busy(100)
  lsp.sweep()
  T.eq(#T.queued "lsp_overdue", 1 + 500)
end)

T.test("requests of a client that went away are not overdue", function()
  local real_get = vim.lsp.get_client_by_id
  local alive = true
  vim.lsp.get_client_by_id = function(id)
    if id == 77 and alive then return { name = "gone-soon" } end
    return real_get(id)
  end
  vim.api.nvim_exec_autocmds("LspRequest", {
    modeline = false,
    data = {
      client_id = 77,
      request_id = 1,
      request = { type = "pending", method = "textDocument/hover", bufnr = 0 },
    },
  })
  alive = false -- e.g. :LspRestart; nvim never completes its requests
  local before = #T.queued "lsp_overdue"
  T.busy(100)
  lsp.sweep()
  vim.lsp.get_client_by_id = real_get
  T.eq(#T.queued "lsp_overdue", before)
end)

T.test("malformed event data is ignored", function()
  vim.api.nvim_exec_autocmds(
    "LspRequest",
    { modeline = false, data = { client_id = 1 } }
  )
  vim.wait(50)
  T.ok(diag.is_running "lsp", "still running")
end)

T.done()
