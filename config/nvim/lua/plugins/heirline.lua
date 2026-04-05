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

    -- local path_func =
    --   status.provider.filename { modify = ":.:h", fallback = "" }

    -- opts.winbar = { -- create custom winbar
    --   -- store the current buffer number
    --   init = function(self) self.bufnr = vim.api.nvim_get_current_buf() end,
    --   fallthrough = false, -- pick the correct winbar based on condition
    --   -- inactive winbar
    --   {
    --     condition = function() return not status.condition.is_active() end,
    --     -- show the path to the file relative to the working directory
    --     status.component.separated_path { path_func = path_func },
    --     -- add the file name and icon
    --     status.component.file_info {
    --       file_icon = {
    --         hl = status.hl.file_icon "winbar",
    --         padding = { left = 0 },
    --       },
    --       filename = {},
    --       filetype = false,
    --       file_modified = false,
    --       file_read_only = false,
    --       hl = status.hl.get_attributes("winbarnc", true),
    --       surround = false,
    --       update = "BufEnter",
    --     },
    --   },
    --   -- active winbar
    --   {
    --     -- show the path to the file relative to the working directory
    --     status.component.separated_path { path_func = path_func },
    --     -- add the file name and icon
    --     status.component.file_info { -- add file_info to breadcrumbs
    --       file_icon = { hl = status.hl.filetype_color, padding = { left = 0 } },
    --       filename = {},
    --       filetype = false,
    --       file_modified = false,
    --       file_read_only = false,
    --       hl = status.hl.get_attributes("winbar", true),
    --       surround = false,
    --       update = "BufEnter",
    --     },
    --     -- show the breadcrumbs
    --     status.component.breadcrumbs {
    --       icon = { hl = true },
    --       hl = status.hl.get_attributes("winbar", true),
    --       prefix = true,
    --       padding = { left = 0 },
    --     },
    --   },
    -- }
  end,
}
