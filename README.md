# VTS-Mapproxy

[VTS-Mapproxy](https://github.com/melown/vts-mapproxy) is a HTTP server that
converts non-VTS resources (raster or vector) to VTS resources (surface,
boundlayer, freelayer) on the fly.

## User documentation

VTS-Mapproxy user documentation is available at
https://melown.readthedocs.io/

## Install

For installation instructions, see the
[buildsys-common](https://github.com/melown/buildsys-common) project on GitHub.

## Run VTS-Mapproxy server

First you need to create `mapproxy.conf` configuration file. You then can run

```
mapproxy --config mapproxy.conf
```

Description of the configuration file can be found in our [user documentation](http://melown.readthedocs.io/en/latest/server/mapproxy.html).

The server is not intended to be exposed to the Internet as it is, instead it's
advised to hide it behind e.g. [NGINX](https://www.nginx.com/) server.

## How to contribute

Check the [CONTRIBUTING.md](CONTRIBUTING.md) file.

---

## generatevrtwo: Generate virtual GDAL dataset with overviews.

Dataset format:
* directory with:
 * original: symlink to original dataset (anything user provided)
 * dataset: virtual dataset itself (VRT driver); use this file as an entry point
 * 0-N: directory with overview level
  - ovr.vrt: virtual dataset encapsulating individual tiles
  - x-y.tif: file with tile at (x, y)
  - bg.solid: (optional) dataset with uniform vaulu used in place of missing tiles

NB: symlink to original dataset is used to ease remapping of "vrtwo" dataset
when moving around filesystems/machines.

---

## Windyty driver

Dataset format:

```conf
# windyty dataset
[windyty]
# Base time in seconds from Epoch (Unix timestamp)
base = 0
# Period - forcast is available each 'period' seconds from 'base'
period = 10800
# Source TMS URL template. Uses strftime(3) for time formatting.
urlTemplate = http://origin-backup.windyty.com/ecmwf-hres/%Y/%m/%d/%H/256mt${z}/${y}/${x}/clouds2-surface.png
# Source data SRS as EPSG code
srs = EPSG:4326
# Source data extents.
extents = -180,-90:180,90
# Source (maximum) level of detail in source data.
maxLod = 8
# Number of overviews to generate.
overviews = 8
# Transparent images?
transparent = true
```

---

## Per-resource max-age

Full resource max-age definition:

```json
"maxAge": {
    "config": 60
    , "support": 3600
    , "registry": 3600
    , "data": 604800
}
```
Each entry specifies `max-age` (in seconds) set into HTTP `Cache-Control` header for given file class.
File classes without definition in resource use default value from server's configuration. Whole `maxAge`
section can be ommited.

Server's defaults can be changed by specifying one of `--max-age.XXX` where `XXX` is file class.

Supported file classes are
 * `config`: configuration files (mapConfig.json, boundlayer.json and freelayer.json so far)
 * `support`: support files (built-in 3D browser, built-in 2D browser etc.)
 * `registry`: registry files (e.g. geoid grid files from registry)
 * `data`: resource's data (metatiles, textures, meshes, masks, raw surface data, ...)

Negative value is translated into `Cache-Control: no-cache`.
