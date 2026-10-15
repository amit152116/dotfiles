local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

local GROUP = "Diagnostics_lsp"
local MAX_PENDING = 500
local SWEEP_MS = 5000

local MAX_PROGRESS = 100
local PROGRESS_MAX_AGE_MS = 60 * 60 * 1000
-- clients can take a while to exit after detaching; recheck before giving up
local EXIT_CHECKS_MS = { 100, 2000 }

local pending, count, timer = {}, 0, nil
local progress, progress_count, exited = {}, 0, {}
local names = {}

---@param id integer
---@return string
local function client_name(id)
  local client = vim.lsp.get_client_by_id(id)
  if client then names[id] = client.name end
  return names[id] or ("client " .. tostring(id))
end

--- Pair work-done progress begin/end by token, e.g. clangd indexing.
---@param ev {data: any}
local function on_progress(ev)
  local d = ev.data
  if type(d) ~= "table" or type(d.params) ~= "table" then return end
  local value = d.params.value
  if type(value) ~= "table" then return end
  local key = tostring(d.client_id) .. ":" .. tostring(d.params.token)
  if value.kind == "begin" then
    if progress[key] or progress_count >= MAX_PROGRESS then return end
    progress[key] = {
      start = uv.hrtime(),
      title = value.title,
      client = client_name(d.client_id),
      client_id = d.client_id,
    }
    progress_count = progress_count + 1
  elseif value.kind == "end" then
    local p = progress[key]
    if not p then return end
    progress[key] = nil
    progress_count = progress_count - 1
    local ms = (uv.hrtime() - p.start) / 1e6
    if ms > config.get_value "thresholds.lsp_progress_ms" then
      writer.emit("lsp_progress", ms, { client = p.client, title = p.title })
    end
  end
end

-- detach happens per buffer; the client being gone afterwards means it stopped or crashed
---@param ctx {guard: fun(fn: function): function}
---@param ev {data: any}
local function on_detach(ctx, ev)
  local id = type(ev.data) == "table" and ev.data.client_id
  if not id or exited[id] then return end
  local name = client_name(id)
  -- the client still exists at detach; _is_stopping is set only by a requested stop
  -- (:LspRestart, :LspStop, plugins), so only unrequested exits count as crashes
  local client = vim.lsp.get_client_by_id(id)
  if client and client._is_stopping then return end
  local function check(attempt)
    if exited[id] then return end
    if vim.lsp.get_client_by_id(id) == nil then
      exited[id] = true
      writer.emit("lsp_exit", nil, { client = name })
    elseif attempt < #EXIT_CHECKS_MS then
      vim.defer_fn(
        ctx.guard(function() check(attempt + 1) end),
        EXIT_CHECKS_MS[attempt + 1]
      )
    end
  end
  vim.defer_fn(ctx.guard(function() check(1) end), EXIT_CHECKS_MS[1])
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
  -- a dead server never sends "end"; without this the cap would block all later progress
  for key, p in pairs(progress) do
    local stale = (now - p.start) / 1e6 > PROGRESS_MAX_AGE_MS
    if stale or vim.lsp.get_client_by_id(p.client_id) == nil then
      progress[key] = nil
      progress_count = progress_count - 1
    end
  end
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
  local group = vim.api.nvim_create_augroup(GROUP, { clear = false })
  vim.api.nvim_create_autocmd("LspProgress", {
    group = group,
    callback = ctx.guard(on_progress),
  })
  vim.api.nvim_create_autocmd("LspDetach", {
    group = group,
    callback = ctx.guard(function(ev) on_detach(ctx, ev) end),
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
  progress, progress_count, exited = {}, 0, {}
end

return M
