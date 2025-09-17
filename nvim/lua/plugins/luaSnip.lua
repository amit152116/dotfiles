return {
  {
    -- You can also easily customize additional setup of plugins that is outside of the plugin's setup call
    "L3MON4D3/LuaSnip",
    config = function(plugin, opts)
      require "astronvim.plugins.configs.luasnip"(plugin, opts) -- include the default astronvim config that calls the setup call
      -- add more custom luasnip configuration such as filetype extend or custom snippets
      local luasnip = require "luasnip"

      local s = luasnip.snippet
      local t = luasnip.text_node
      local i = luasnip.insert_node

      -- Extend filetypes if needed
      luasnip.filetype_extend("javascript", { "javascriptreact" })

      -- 🔥 Add your custom snippets
      luasnip.add_snippets("cpp", {
        s("algo_doc", {
          t {
            "/*",
            " * Definition:",
            " *   ",
          },
          i(1, "One-liner about what it does"),
          t {
            "",
            " *",
            " * Allowed Operations / Rules:",
            " *   ",
          },
          i(2, "Operations"),
          t {
            "",
            " *",
            " * Recurrence Relation / Formula:",
            " *   ",
          },
          i(3, "Formula/Recurrence"),
          t {
            "",
            " *",
            " * Complexity:",
            " *   Time: ",
          },
          i(4, "O(n)"),
          t {
            "",
            " *   Space: ",
          },
          i(5, "O(1)"),
          t {
            "",
            " *",
            " * Example:",
            " *   ",
          },
          i(6, "Example input/output"),
          t {
            "",
            " *",
            " * Use Cases:",
            " *   ",
          },
          i(7, "Real-world applications"),
          t {
            "",
            " *",
            " * Comparison:",
            " *   ",
          },
          i(8, "Similar algos/data structures"),
          t {
            "",
            " */",
          },
        }),
      })
    end,
  },
}
