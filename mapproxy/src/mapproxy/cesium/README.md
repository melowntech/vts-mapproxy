# VTS Mapproxy resource Cesium Terrain Provider interface

## Generail info

Cesium Terrain Proviver interface allowd VTS surface resource to be used in any
Cesium application as a terrain provider.

For every terrain-enabled reference frame mapproxy provides a
[layer.json](layer.json) file containing layer description and
appropriateterrain files.

If the original dataset doesn't cover whole Earth the undefined height is
replaced with zero height.

There's an [interactive browser](browser.html) provided by mapproxy for testing
and evaluation.

You can the use the [URL of this directory[(.) to add this resource as a terrain
provider to your Cesium application.

    var terrainProvider = new Cesium.CesiumTerrainProvider({
        url: "path-to-this-resource"
    });
