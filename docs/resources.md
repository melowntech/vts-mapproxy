# Supported resource types/drivers

Mapproxy supports following resource types:
 * tms: *tile map service* (bound layer)
 * surface: surfaces (i.e. tileset generator)
 * geodata: vector free layers

Mapproxy provides multiple drivers (*generators*) for each resource type.

## Basic resource layout

NB: In following, all resource definitions are documented by pseudo format. Actual configuration is either
a JSON file on disk (for JSON resource backend) or a python data tree. 
 
 * `{}`: JSON object/python dictionary
 * `[]` JSON/python array
 * `String`: string data type
 * `Int`: integral JSON number/python integer
 * `Double`: real JSON number/python double
 * `Boolean`: boolean JSON/python value
 * `Array<type>`: array of given type(s)
 * `Optional` given entry is optional
 * `?` no fixed name

Basic resource layout:

```javascript
resource = {
    String comment      // any comment, ignored
    String group        // group this resource belongs to
    String id           // resource identifier (withing group)
    String type         // data type (tms, surface, geodata)
    String driver       // data generator (see below)
    Object registry     // additional local resource registry, see below
    Array<String, Int> credits // list of credits identifiers (either textual or numeric)
    Object referenceFrames = { // range definitions for different reference frames
        Object ? = { // reference frame ID, for example melown2015
            lodRange: [ Int, Int ]               // LOD range this resource produces data
            tileRange: [[Int, Int], [Int, Int]]  // tile range: minx, miny, maxx, maxy; inclusive range
        }
    }
    Object definition = {...} // driver-dependent definition, see below
}
```

## TMS drivers

### tms-raster

Raster-based bound layer generator. Uses any raster GDAL dataset as its data source. Supports optional data masking.

```javascript
definition = {
    String dataset               // path to GDAL dataset
    Optional String mask         // path to RF mask or masking GDAL dataset
    Optional String format       // output image format, "jpg" or "png" (defaults to "jpg")
    Optional Boolean transparent // Boundlayer is transparent, forces format to "png"
}
```

### tms-raster-remote

Raster bound layer generator. Imagery is pointer to external resource via `remoteUrl` (a URL template). Supports optional data masking.

```javascript
definition = {
    String remoteUrl             // Imagery URL template.
    Optional String mask         // path to RF mask or masking GDAL dataset
}
```


### tms-patchwork

Simple raster bound layer generator. Generates color checkered tiles. Supports optional data masking.

```javascript
definition = {
    Optional String mask         // path to RF mask or masking GDAL dataset
    Optional String format       // output image format, "jpg" or "png" (defaults to "jpg")
}
```

### tms-bing

Bound layer generator for remote Bing data. Valid session is generated via metatada URL.

```javascript
definition = {
    String metadataUrl           // Bing API metadata URL. See Bing API documentation for more info.
}
```

## Surface drivers

### Commpon surdace driver configuration options

All surface drivers support these (optional) options:
```javascript
    Optional String geoidGrid           // name of Proj.4's geoid grid file (e.g. `egm96_15.gtx`).
    Optional Double nominalTexelSize    // nominal resolution (meter/pixel);
                                        // reported by mapproxy-calipers
    Optional Int mergeBottomLod         // Reported in generated tileset.conf, speeds up merge
                                        // with other surfaces
    Optional Object/Array introspection // Introspection info used when using mapConfig.json served
                                        // by mapproxy. See below.
```

Introspection is extending configuration for mapproxy served `mapConfig.json` (only when browsing is enabled).

```javascript
introspection = {
    Optional Array position;   // VTS position in JSON/python format
    Optional Object/Array tms  // bound layer(s) mapped on the surface, see below 
}
```

TMS in the above introspection is one or more TMS resource identifiers:
```javascript
tms = [
    {
        String group           // group part of TMS resource identifier
        Strting id             // ID part of TMS resource identifier
    }
    ...
]
```

If there is just one TMS resource used in the introspection then the enclosing array is optional.

### surface-spheroid

This driver generates meshed surface for reference frame's spheroid. If geoid grid is provided the resulting body
is in fact a geoid.

If a `textureLayerId` entry is present this ID is written into generated meshes as a default bound layer to use
if nothing else is mapped on the surface. Otherwise surface is completely texture less.

```javascript
definition = {
    Optional Int textureLayerId  // numeric bound layer ID
}
```

### surface-dem

This driver generates meshed surface from supplied GDAL raster DEM/DSM/DTM dataset.

Since GDAL raster formats are
unable to safely store vertical SRS component and thus it cannot tell whether data are in ellipsoidal or orthometric
verical datum. Therefore by default data are treated as data above ellipsoid (i.e. ellipsoidal vertical datum). By providing
a `geoidGrid` configuration option we can specify geoid grid for orthormetric vertical datum, i.e. to tell that the heights
store in the GDAL dataset are relative to given geoid.

Please be aware that due to such limitations the GDAL dataset's vertical system must be compatible with reference frame's
vertical system to use geoid support. I.e. either they share the same ellipsoid or the input data are in some
local system that approximates the geoid at given place. One working example is data in Krovak's projection that can
be reinterpreted as heights above WGS84+EGM96.

If a `textureLayerId` entry is present this ID is written into generated meshes as a default bound layer to use
if nothing else is mapped on the surface. Otherwise surface is completely texture less.

All `surface-dem` input datasets are registered in internal map od available DEM's under its `group-id` identifier
and can be referenced from various `geodata`resources for 2D features heightcofing. See meore in `geodata` resources
documentation. Optionially, input dataset can be registered in this map under an alias.

```javascript
definition = {
    String dataset                    // path to complex dataset
    Optional String mask              // optional mask, generated by mapproxy-rf-mask tool
    Optional Int textureLayerId       // numeric bound layer ID
    Optional String heightcodingAlias // dataset is registered under given alias
}
```

