-- AstroUI provides the basis for configuring the AstroNvim User Interface
-- Configuration documentation can be found with `:h astroui`
-- NOTE: We highly recommend setting up the Lua Language Server (`:LspInstall lua_ls`)
--       as this provides autocomplete and documentation while editing

---@type LazySpec
return {
  "AstroNvim/astroui",
  ---@type AstroUIOpts
  opts = {
    -- change colorscheme
    colorscheme = "astrodark",
    -- AstroUI allows you to easily modify highlight groups easily for any and all colorschemes
    highlights = {
      init = { -- this table overrides highlights in all themes
        -- Normal = { bg = "NONE" }, -- Normal: Main editor area
        -- NormalNC = { bg = "NONE" }, -- NormalNC: Non-current windows
        -- NormalFloat = { bg = "NONE" }, -- NormalFloat: Floating windows
        -- FloatBorder = { bg = "NONE" }, -- FloatBorder: Borders of floating windows
        -- SignColumn = { bg = "NONE" }, -- SignColumn: Column where signs appear (like git diff)
        -- Folded = { bg = "NONE" }, -- Folded: Folded text
        -- FoldColumn = { bg = "NONE" }, -- FoldColumn: Column showing fold markers
      },
      astrodark = { -- a table of overrides/changes when applying the astrotheme theme
        -- #1a1b26 (a deep navy blue with low brightness)
        -- #11121d (deep space)
        -- - `#1a1a2a` (Moonfly)
        -- - `#1d2021` (Gruvbox material dark)
        -- - `#16161d` (Oxocarbon)
        -- - `#0f111a` (Material Palenight darker variant)
        -- - `#131a24` (dark navy/slate)
        -- - `#14191f` (deep charcoal with a slight blue tint)
        Normal = { bg = "#14191f" },
        NormalFloat = { bg = "#14191f" },
        FloatBorder = { bg = "#14191f" },
      },
    },
    text_icons = {
      -- text icons can be used in place of glyphs in the interface
      -- these are used in the statusline, tabline, and other places
      -- you can use any unicode character or string here
      -- File = "󰈙 ",
      -- Folder = "󰉋 ",
      LSP = "󰒘 ",
      Git = "󰊢 ",
      Diagnostics = "󰗣 ",
      Search = "󰊄 ",
    },
    -- Icons can be configured throughout the interface
    icons = {
      -- configure the loading of the lsp in the status line
      LSPLoading1 = "⠋",
      LSPLoading2 = "⠙",
      LSPLoading3 = "⠹",
      LSPLoading4 = "⠸",
      LSPLoading5 = "⠼",
      LSPLoading6 = "⠴",
      LSPLoading7 = "⠦",
      LSPLoading8 = "⠧",
      LSPLoading9 = "⠇",
      LSPLoading10 = "⠏",
      VimIcon = "",
    },
    status = {},
  },
}
