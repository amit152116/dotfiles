return {
  {
    -- You can also easily customize additional setup of plugins that is outside of the plugin's setup call
    "L3MON4D3/LuaSnip",
    config = function(plugin, opts)
      require "astronvim.plugins.configs.luasnip"(plugin, opts) -- include the default astronvim config that calls the setup call
      -- add more custom luasnip configuration such as filetype extend or custom snippets
      local luasnip = require "luasnip"

      local snippet = luasnip.snippet
      local text = luasnip.text_node
      local insert = luasnip.insert_node

      -- Extend filetypes if needed
      luasnip.filetype_extend("javascript", { "javascriptreact" })

      -- 🔥 Add your custom snippets
      luasnip.add_snippets("cpp", {
        snippet("stdc", {
          text { "#include <bits/stdc++.h>", "", "" },
          text { "int main(int argc, char* argv[]) {", "    " },
          insert(0),
          text { "", "    return 0;", "}" },
        }),
        snippet("algo_doc", {
          text {
            "/*",
            " * Definition:",
            " *   ",
          },
          insert(1, "One-liner about what it does"),
          text {
            "",
            " *",
            " * Allowed Operations / Rules:",
            " *   ",
          },
          insert(2, "Operations"),
          text {
            "",
            " *",
            " * Recurrence Relation / Formula:",
            " *   ",
          },
          insert(3, "Formula/Recurrence"),
          text {
            "",
            " *",
            " * Complexity:",
            " *   Time: ",
          },
          insert(4, "O(n)"),
          text {
            "",
            " *   Space: ",
          },
          insert(5, "O(1)"),
          text {
            "",
            " *",
            " * Example:",
            " *   ",
          },
          insert(6, "Example input/output"),
          text {
            "",
            " *",
            " * Use Cases:",
            " *   ",
          },
          insert(7, "Real-world applications"),
          text {
            "",
            " *",
            " * Comparison:",
            " *   ",
          },
          insert(8, "Similar algos/data structures"),
          text {
            "",
            " */",
          },
        }),
      })
    end,
  },
}
