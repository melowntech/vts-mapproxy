-- This is an example of nginx lua header filter for mapproxy to transform
-- X-MapProxy-File-Class into Cache-Control
--
-- Usage: header_filter_by_lua_file "path/to/mapproxy-cache-control.lua";
--

-- Feel free to tweak this table
local cacheControlMapping = {
   -- config files (mapConfig.json, boundlayer.json, freelayer.json, surface
   -- tileset files: tileset.config, tileset.index) [ can change, better get
   -- fresh copy often than bang head agains the wall :P ]
   config = 60

   -- support files: 2d and 3d browser (index.html etc) [can change with
   -- different releases]
   , support = 3600

   -- registry files: geoids [can change with different releases]
   , registry = 3600

   -- data: persistent data files
   , data = 604800
}



--
-- Don't touch this unless you know what you are doing
--

-- header name
local header = "X-MapProxy-File-Class";

-- not 200 -> nothing to do
if (ngx.status ~= ngx.HTTP_OK) then return end

-- check header -> not found -> done
local fileClass = ngx.header[header]
if (not fileClass) then return end

-- remove header from request
ngx.header[header] = nil

-- map file-class to time
local time = cacheControlMapping[fileClass]
if (time == nil) then return end

-- and finally generate Cache-Control header
ngx.header["Cache-Control"] = string.format("max-age=%d", time)

-- done
