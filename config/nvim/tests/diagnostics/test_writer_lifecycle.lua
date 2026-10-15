local T = dofile "tests/diagnostics/helpers.lua"
local writer = require "diagnostics.writer"
local uv = vim.uv

local function flush_all()
  writer.flush()
  T.ok(
    vim.wait(1000, function()
      local st = writer.status()
      return st.queued == 0 and not st.in_flight
    end, 10),
    "flush timed out"
  )
end

local PAD = string.rep("p", 40)

T.test("rotation keeps size under cap and keeps a backup", function()
  local path = T.open_writer { max_size = 300 }
  for i = 1, 10 do
    writer.emit("x", i, { pad = PAD })
    flush_all()
  end
  local active, backup = T.read_events(path), T.read_events(path .. ".1")
  T.ok(#backup > 0, "backup missing")
  T.ok(uv.fs_stat(path).size <= 300, "active file over cap")
  T.ok(#active + #backup <= 10, "no duplicated lines")
  T.eq(active[#active].ms, 10)
  writer.close()
end)

T.test("follows the path after another instance rotated it", function()
  local path = T.open_writer()
  writer.emit("mine", 1)
  flush_all()
  -- simulate a second nvim rotating the shared log
  uv.fs_rename(path, path .. ".1")
  local f = assert(io.open(path, "w"))
  f:write(
    vim.json.encode { v = 1, ts = 1, sid = "other", kind = "theirs" } .. "\n"
  )
  f:close()
  writer.emit("after", 2)
  flush_all()
  local kinds = vim.tbl_map(function(e) return e.kind end, T.read_events(path))
  T.eq(kinds, { "theirs", "after" })
  T.eq(#T.read_events(path .. ".1"), 1)
  writer.close()
end)

T.test("cap counts bytes other instances appended", function()
  local path = T.open_writer { max_size = 300 }
  local f = assert(io.open(path, "a"))
  f:write(string.rep("x", 299) .. "\n")
  f:close()
  writer.emit("x", 1, { pad = PAD })
  flush_all()
  T.ok(uv.fs_stat(path .. ".1"), "no rotation despite file over cap")
  T.eq(#T.read_events(path), 1)
  writer.close()
end)

T.test("shutdown drains the queue synchronously", function()
  local path = T.open_writer()
  for i = 1, 3 do
    writer.emit("x", i)
  end
  writer.shutdown()
  T.eq(#T.read_events(path), 3)
  T.eq(writer.status().enabled, false)
end)

T.test("shutdown waits for an in-flight write", function()
  local path = T.open_writer()
  writer.emit("a", 1)
  writer.flush()
  writer.emit("b", 2)
  writer.shutdown()
  local kinds = vim.tbl_map(function(e) return e.kind end, T.read_events(path))
  T.eq(kinds, { "a", "b" })
end)

T.test("clear empties both files and resets counters", function()
  local path = T.open_writer { max_size = 200, max_queue = 2 }
  for i = 1, 6 do
    writer.emit("x", i, { pad = PAD })
    flush_all()
  end
  writer.emit("q", 1)
  writer.emit("q", 2)
  writer.emit("q", 3)
  T.eq(writer.status().dropped, 1)
  T.ok(writer.clear(), "clear failed")
  T.eq(writer.status().queued, 0)
  T.eq(writer.status().dropped, 0)
  T.eq(uv.fs_stat(path .. ".1"), nil)
  T.eq(uv.fs_stat(path).size, 0)
  writer.emit("after", 1)
  flush_all()
  T.eq(#T.read_events(path), 1)
  writer.close()
end)

T.done()
