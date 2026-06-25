-- needs wakatime-cli on PATH; heirline display wired directly in plugins/heirline.lua

-- wakatime-cli has no notion of "current nvim cwd" -- filter by project name
-- (git root folder name, matching what heartbeats record) so the stat is
-- scoped to this project, not a --today total across every project tracked
local function current_project()
  local root = vim.fn.systemlist("git rev-parse --show-toplevel")[1]
  return vim.fs.basename(vim.v.shell_error == 0 and root or vim.fn.getcwd())
end

return {
  "fiqryq/wakastat.nvim",
  event = "VeryLazy",
  cmd = { "WakastatRefresh", "WakastatStatus" },
  opts = {
    args = { "--today", "--project", current_project() },
    format = "󰥔 %s", -- icon-only,
    update_interval = 300,
    enable_timer = true,
  },
  config = function(_, opts)
    require("wakastat").setup(opts)
    vim.api.nvim_create_autocmd("DirChanged", {
      desc = "Re-scope wakastat to the new project on dir change",
      callback = function()
        opts.args = { "--today", "--project", current_project() }
        require("wakastat").setup(opts)
      end,
    })
  end,
}
