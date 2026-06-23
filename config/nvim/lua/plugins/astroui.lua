---@type LazySpec
return {
  "AstroNvim/astroui",
  ---@type AstroUIOpts
  opts = {
    colorscheme = "astrodark",
    highlights = {
      init = {}, -- overrides applied to every colorscheme
      astrodark = { -- deep charcoal bg, lower brightness than stock astrodark
        Normal = { bg = "#14191f" },
        NormalFloat = { bg = "#14191f" },
        FloatBorder = { bg = "#14191f" },
      },
    },
    text_icons = {
      LSP = "󰒘 ",
      Git = "󰊢 ",
      Diagnostics = "󰗣 ",
      Search = "󰊄 ",
    },
    icons = {
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
