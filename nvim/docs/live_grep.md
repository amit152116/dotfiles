Telescope’s built-in **`live_grep`** is just a wrapper around its
`builtin.grep_string` logic + some special finder settings, so you can tweak a
_lot_ of things via the `opts` table when calling it.

Here’s the breakdown of what you can modify in `live_grep(opts)`:

______________________________________________________________________

## **1. Search-related options**

These affect _what_ ripgrep searches:

| Option | Type | Purpose |
| ----------------- | -------- | ---------------------------------------------------------------------------- |
| `grep_open_files` | boolean | Search only in currently open buffers. |
| `cwd` | string | Change working directory for the search. |
| `search_dirs` | table | Restrict search to specific directories. Example: `{ "src", "tests" }` |
| `type_filter` | string | Ripgrep `--type` filter (like `"rust"`, `"lua"`, `"py"`). |
| `additional_args` | function | Append custom ripgrep args. Example: `function() return { "--hidden" } end`. |

______________________________________________________________________

## **2. Prompt & UI**

These affect how Telescope displays:

| Option | Type | Purpose |
| ------------------ | ---------------- | ------------------------------------------------------- |
| `prompt_title` | string | Title above the prompt. |
| `prompt_prefix` | string | Symbol/emoji before prompt. |
| `results_title` | string | Title above results list. |
| `layout_strategy` | string | `"horizontal"`, `"vertical"`, `"flex"`, `"center"`. |
| `layout_config` | table | Size & positioning details for preview/results windows. |
| `sorting_strategy` | string | `"ascending"` or `"descending"`. |
| `previewer` | boolean/function | Enable/disable custom previewer. |

______________________________________________________________________

## **3. Entry parsing**

These affect _how_ results are transformed into entries:

| Option | Type | Purpose |
| -------------- | --------------------- | ----------------------------------------------------------------------------------------- |
| `entry_maker` | function | Custom parser for ripgrep output lines. Useful for capturing groups or custom formatting. |
| `path_display` | string/table/function | Control how paths are shown (`"hidden"`, `"tail"`, `"absolute"`). |

______________________________________________________________________

## **4. Sorter & Matching**

These affect filtering inside Telescope (after ripgrep returns):

| Option | Type | Purpose |
| ------------- | ------------- | ------------------------------------------------- |
| `sorter` | sorter object | Custom sorter (like `conf.generic_sorter(opts)`). |
| `file_sorter` | sorter object | How matched files are sorted. |

______________________________________________________________________

## **Example: heavily customized `live_grep`**

```lua
local builtin = require("telescope.builtin")

builtin.live_grep({
  prompt_title = "🔍 Search Project",
  cwd = "~/myproject",
  search_dirs = { "src", "include" },
  grep_open_files = false,
  additional_args = function()
    return { "--hidden", "--no-ignore", "--pcre2" }
  end,
  entry_maker = function(line)
    local filename, lnum, col, text = line:match("([^:]+):(%d+):(%d+):(.*)")
    return {
      value = text,
      display = filename .. ":" .. lnum .. ":" .. text,
      ordinal = text,
      filename = filename,
      lnum = tonumber(lnum),
      col = tonumber(col),
      text = text,
    }
  end,
  layout_strategy = "vertical",
  layout_config = {
    width = 0.9,
    preview_height = 0.6,
  },
})
```

______________________________________________________________________

If you want to **capture regex groups from ripgrep** in `live_grep`, you’d
mainly customize:

- `additional_args` → to add `--pcre2` and your regex pattern.
- `entry_maker` → to parse the line and extract capture groups.

______________________________________________________________________

If you want, I can give you a **ready `live_grep` override** that takes capture
groups from ripgrep and shows them _already replaced_ in the previewer while you
scroll results. That would combine your earlier capture idea with Telescope’s
built-in `live_grep`.
