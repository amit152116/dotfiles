-- Inline variable values next to source lines while debugging (C++/Python/Go).
-- Complements dap-ui (scopes panel) by annotating the actual code lines.
return {
  "theHamsta/nvim-dap-virtual-text",
  dependencies = { "mfussenegger/nvim-dap" },
  -- load alongside the dap stack pulled in by the astrocommunity packs
  event = "User AstroFile",
  opts = {
    enabled = true,
    enabled_commands = true, -- create DapVirtualText* user commands
    highlight_changed_variables = true, -- highlight vars that changed value
    highlight_new_as_changed = false,
    show_stop_reason = true, -- show stop reason as virtual text on exception
    commented = false, -- prefix virtual text with comment string
    only_first_definition = true,
    all_references = false,
    clear_on_continue = false,
    display_callback = function(variable, _buf, _stackframe, _node, options)
      if options.virt_text_pos == "inline" then
        return " = " .. variable.value
      end
      return variable.name .. " = " .. variable.value
    end,
    -- nvim 0.10+ supports inline virtual text (anchored to the variable)
    virt_text_pos = vim.fn.has "nvim-0.10" == 1 and "inline" or "eol",
    all_frames = false, -- only the focused stack frame
    virt_lines = false,
    virt_text_win_col = nil,
  },
}
