function script_path()
  local str = debug.getinfo(2, "S").source:sub(2)
  return str:match("(.*/)") or "./"
end
-- allow us to run script from any directory
package.path = package.path .. ";" .. script_path() .. "?.lua"
-- add config directory to imports
package.path = package.path .. ";" .. script_path() .. "../../../management/astrobee/config/?.config"

-- import the config files we need
require "common_lua"
--require "robot"
require "gnc"

function toMatlab(v)
  if type(v) == "number" then
    return string.format("single(%g)", v)
  elseif type(v) == "boolean" then
    return string.format("uint8(%d)", v and "1" or "0")
  elseif type(v) == "table" then
    local s = ""
    for i, x in ipairs(v) do
      t = toMatlab(x)
      s = s .. t
      if i ~= #v then
        s = s .. ", "
        if #t > 20 then
          s = s .. ";\n"
        end
      end
    end
    return "[" .. s .. "]"
  else
    return "Error: Unexpected type."
  end
end

local filename = script_path() .. "lua_config.m"
local f = assert(io.open(filename, "w"))
for k in pairs(_G) do
  if k:find("^tun_") then
    f:write(k .. " = " .. toMatlab(_G[k]) .. ";\n")
  end
end
f:close()

