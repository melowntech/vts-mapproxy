# VTS Mapproxy resource WMTS interface

## Generail info

WMTS interface allowd VTS raster resource to be used in WMTS enabled WWW clients
or GIS tool.

For every WMTS-enabled reference frame mapproxy provides a
[WMTSCapabilities.xml](WMTSCapabilities.xml) file containing single layer for
this particular resource.

There's an [interactive browser](browser.html) provided by mapproxy for testing
and evaluation.

You can the use the [URL of this directory[(.)in tools like
[GDAL](https://www.gdal.org/) or [QGIS](https://qgis.org/en/site/).

## Examples

### GDAL

* GDAL: warp whole dataset into a TIF file 1024 px wide with aspect ratio kept

         # gdalwarp wmts:http://.../WMTSCapabilities.xml output.tif -ts 1024 0 -r cubic

     (please notice the `wmts:` URL prefix)

### QGIS
In QGIS desktop application:

* Open the "Add Layer(s) from a WM(T)S Server" dialog:
    * either via menu: `Layer` -> `Add Layer` -> `Add WMT/WMTS Layer...`
    * or by keyboard shortcut: `Ctrl+Shift+W`
* Click the `New` button to add new server connection
    * Choose name for new server in tha `Name` field
    * Paste the use [WMTSCapabilities.xml URL](WMTSCapabilities.xml) in the `URL` field
    * Click OK to save the connection
* New server connection should become selected in the drop-down list
* Click the `Connect` button
* Select the only layer available
* Click the `Add` butto

---

*Nota bene*: If you want to use WMTS interface outside of introspection browser
you have to have properly configured the `http.externalUrl` mapproxy
configuration options. For example:

    [http]
    externalUrl = http://myserver.mydomain.com/mapproxy/root
