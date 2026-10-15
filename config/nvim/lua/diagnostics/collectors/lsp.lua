local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

local GROUP = "Diagnostics_lsp"
local MAX_PENDING = 500
local SWEEP_MS = 5000

local pending, count, timer = {}, 0, nil

---@param id integer
---@return string
local function client_name(id)
  local client = vim.lsp.get_client_by_id(id)
  return client and client.name or ("client " .. tostring(id))
end

local function drop_oldest()
  local oldest_key, oldest_start
  for key, p in pairs(pending) do
    if not oldest_start or p.start < oldest_start then
      oldest_key, oldest_start = key, p.start
    end
  end
  if oldest_key then
    pending[oldest_key] = nil
    count = count - 1
  end
end

---@param ev {data: any}
local function on_request(ev)
  local d = ev.data
  if type(d) ~= "table" or type(d.request) ~= "table" then return end
  local key = tostring(d.client_id) .. ":" .. tostring(d.request_id)
  local kind = d.request.type

  if kind == "pending" then
    if pending[key] then return end
    if count >= MAX_PENDING then drop_oldest() end
    pending[key] = {
      start = uv.hrtime(),
      method = d.request.method,
      client = client_name(d.client_id),
      client_id = d.client_id,
      known = vim.lsp.get_client_by_id(d.client_id) ~= nil,
    }
    count = count + 1
    return
  end

  local p = pending[key]
  if not p then return end
  pending[key] = nil
  count = count - 1
  local ms = (uv.hrtime() - p.start) / 1e6
  if ms > config.get_value "thresholds.lsp_ms" then
    writer.emit("lsp", ms, {
      client = p.client,
      method = p.method,
      cancelled = kind == "cancel" or nil,
    })
  end
end

--- Report and forget requests that never got a reply.
function M.sweep()
  local now = uv.hrtime()
  local limit = config.get_value "thresholds.lsp_overdue_ms"
  for key, p in pairs(pending) do
    local age = (now - p.start) / 1e6
    -- a stopped client's requests never complete; not a slow server
    local gone = p.known and vim.lsp.get_client_by_id(p.client_id) == nil
    if gone or age > limit then
      pending[key] = nil
      count = count - 1
      if not gone then
        writer.emit(
          "lsp_overdue",
          age,
          { client = p.client, method = p.method }
        )
      end
    end
  end
end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  if timer then return end
  -- built-in LspRequest event: measures real reply latency without patching the client
  vim.api.nvim_create_autocmd("LspRequest", {
    group = vim.api.nvim_create_augroup(GROUP, { clear = true }),
    callback = ctx.guard(on_request),
  })
  timer = uv.new_timer()
  timer:start(SWEEP_MS, SWEEP_MS, ctx.guard(M.sweep))
end

function M.stop()
  pcall(vim.api.nvim_del_augroup_by_name, GROUP)
  if timer then
    timer:stop()
    timer:close()
    timer = nil
  end
  pending, count = {}, 0
end

return M
