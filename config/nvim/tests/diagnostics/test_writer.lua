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

T.test("emit then flush writes ordered jsonl", function()
  local path = T.open_writer()
  writer.emit("stall", 12.345, { buf = "a.lua" })
  writer.emit("lsp", nil, nil)
  flush_all()
  local events = T.read_events(path)
  T.eq(#events, 2)
  T.eq(events[1].kind, "stall")
  T.eq(events[1].ms, 12.3)
  T.eq(events[1].v, 1)
  T.eq(events[1].sid, writer.sid())
  T.eq(events[1].data.buf, "a.lua")
  T.eq(events[2].kind, "lsp")
  T.eq(events[2].ms, nil)
  T.ok(events[1].ts > 1.7e9, "ts is unix seconds")
  T.ok(events[1].ts <= events[2].ts, "ordered")
  writer.close()
end)

T.test("log file is created owner-only", function()
  local path = T.open_writer()
  T.eq(bit.band(uv.fs_stat(path).mode, 511), 384)
  writer.close()
end)

T.test("open is idempotent", function()
  local path = T.open_writer()
  T.ok(writer.open {
    log_file = path .. ".other",
    max_size = 1,
    max_queue = 1,
    flush_ms = 1,
  })
  T.eq(writer.status().path, path)
  writer.close()
end)

T.test("queue overflow drops and counts", function()
  T.open_writer { max_queue = 3 }
  for i = 1, 5 do
    writer.emit("x", i)
  end
  T.eq(writer.status().queued, 3)
  T.eq(writer.status().dropped, 2)
  writer.close()
end)

T.test("unencodable data is dropped, never thrown", function()
  T.open_writer()
  writer.emit("x", 1, { fn = function() end })
  T.eq(writer.status().queued, 0)
  T.eq(writer.status().dropped, 1)
  writer.close()
end)

T.test("recent keeps the last 3 kinds", function()
  T.open_writer()
  for _, kind in ipairs { "a", "b", "c", "d" } do
    writer.emit(kind)
  end
  local kinds = vim.tbl_map(function(r) return r.kind end, writer.recent())
  T.eq(kinds, { "b", "c", "d" })
  writer.close()
end)

T.test("snapshot returns unflushed lines", function()
  T.open_writer()
  writer.emit("x", 1)
  T.eq(#writer.snapshot(), 1)
  T.eq(#T.queued "x", 1)
  writer.close()
end)

T.test("open failure notifies once and drops events", function()
  local calls = T.capture_notify()
  local blocker = T.tmpdir() .. "/file"
  io.open(blocker, "w"):close()
  T.eq(
    writer.open {
      log_file = blocker .. "/diagnostics.jsonl",
      max_size = 1,
      max_queue = 10,
      flush_ms = 60000,
    },
    false
  )
  writer.emit("x", 1)
  T.eq(writer.status().queued, 0)
  vim.wait(50)
  T.eq(#T.own(calls), 1)
  T.ok(calls[1].msg:find("[diagnostics]", 1, true), "prefixed")
  T.eq(calls[1].level, vim.log.levels.ERROR)
  writer.close()
end)

T.test("write failure disables once and keeps the queue", function()
  local calls = T.capture_notify()
  local path = T.open_writer()
  -- log removed and its dir read-only: reopening the path must fail
  uv.fs_unlink(path)
  uv.fs_chmod(vim.fs.dirname(path), 320) -- 0500
  writer.emit("x", 1)
  writer.flush()
  T.ok(
    vim.wait(1000, function() return not writer.status().enabled end, 10),
    "writer not disabled"
  )
  writer.emit("y", 2)
  writer.flush()
  vim.wait(100)
  T.eq(#T.own(calls), 1)
  T.eq(writer.status().queued, 2)
  uv.fs_chmod(vim.fs.dirname(path), 448) -- 0700, let tempdir cleanup work
  writer.close()
end)

T.test("closed fd is recovered by reopening the path", function()
  local path = T.open_writer()
  uv.fs_close(writer._state().fd)
  writer.emit("x", 1)
  writer.flush()
  T.ok(
    vim.wait(1000, function() return writer.status().queued == 0 end, 10),
    "flush did not complete"
  )
  T.eq(writer.status().enabled, true)
  T.eq(#T.read_events(path), 1)
  writer.close()
end)

T.done()
