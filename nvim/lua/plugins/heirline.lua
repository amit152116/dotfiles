return {
  "rebelot/heirline.nvim",
  opts = function(_, opts)
    local status = require "astroui.status"
    opts.statusline = { -- statusline
      hl = { fg = "fg", bg = "bg" },
      status.component.mode {
        mode_text = {
          icon = { kind = "VimIcon", padding = { right = 1 } },
          padding = { right = 1, left = 1 },
          hl = { bold = true, italic = true },
        },
        -- surround the component with a separators
        surround = {
          -- it's a left element, so use the left separator
          separator = "none",
          -- set the color of the surrounding based on the current mode using astronvim.utils.status module
          color = function() return { main = status.hl.mode_bg() } end,
        },
      }, -- add the mode text

      -- add a spacer here
      status.component.builder {
        { provider = "  " },
      },
      status.component.git_branch(),
      status.component.file_info {
        filename = { modify = ":." }, -- show relative path to cwd
        filetype = false, -- no filetype
      },
      status.component.git_diff(),
      status.component.diagnostics(),
      status.component.fill(),
      status.component.cmd_info(),
      status.component.fill(),
      status.component.lsp(),
      status.component.virtual_env(),
      status.component.treesitter(),
      status.component.nav(),
      -- remove the 2nd mode indicator on the right
    }
  end,
}
