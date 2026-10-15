local M = {}

---@param msg string
local function err(msg)
  vim.notify("[diagnostics] " .. msg, vim.log.levels.ERROR)
end

---@param msg string
local function info(msg)
  vim.notify("[diagnostics] " .. msg, vim.log.levels.INFO)
end

---@return string[]
local function status_lines()
  local diag = require "diagnostics"
  local st = require("diagnostics.writer").status()
  local lines = {}
  for _, name in ipairs(diag.COLLECTORS) do
    lines[#lines + 1] =
      string.format("%-8s %s", name, diag.is_running(name) and "on" or "off")
  end
  lines[#lines + 1] = string.format(
    "log: %s (%d KB)",
    st.path or "closed",
    math.floor(st.bytes / 1024)
  )
  lines[#lines + 1] = string.format(
    "writer: %s, queued %d, dropped %d",
    st.enabled and "ok" or "disabled",
    st.queued,
    st.dropped
  )
  return lines
end

function M.register()
  local cmd = vim.api.nvim_create_user_command

  cmd("DiagReport", function(args)
    local report = require "diagnostics.report"
    local opts, msg = report.parse_args(args.fargs)
    if not opts then return err(msg) end
    report.open(opts)
  end, {
    nargs = "*",
    complete = function()
      return {
        "--since",
        "--since=1h",
        "--since=1d",
        "--since=all",
        "--session=current",
        "--session=last",
      }
    end,
    desc = "Show diagnostics report",
  })

  cmd(
    "DiagOpen",
    function()
      vim.cmd.edit(
        vim.fn.fnameescape(require("diagnostics.config").options.log_file)
      )
    end,
    { desc = "Open raw diagnostics log" }
  )

  cmd("DiagClear", function()
    if vim.fn.confirm("Clear diagnostics log?", "&Yes\n&No", 2) ~= 1 then
      return
    end
    if require("diagnostics.writer").clear() then info "log cleared" end
  end, { desc = "Clear diagnostics log" })

  cmd("DiagToggle", function(args)
    local diag = require "diagnostics"
    local name = args.args
    if not vim.tbl_contains(diag.COLLECTORS, name) then
      return err("unknown collector: " .. name)
    end
    if diag.is_running(name) then
      diag.stop_collector(name)
      info(name .. " off")
    elseif diag.start_collector(name) then
      info(name .. " on")
    end
  end, {
    nargs = 1,
    complete = function() return require("diagnostics").COLLECTORS end,
    desc = "Toggle a diagnostics collector",
  })

  cmd(
    "DiagStatus",
    function() info(table.concat(status_lines(), "\n")) end,
    { desc = "Show diagnostics status" }
  )
end

return M
