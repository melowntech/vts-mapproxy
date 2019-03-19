# Preamble

This file is the authoritative mapproxy resource documentation. Please, keep it in sync with the actual implementation.

# Resource definition file

Most common way to define resources for mapproxy is to provide JSON resource definition file and point mapproxy to it by specifying its path in `resource-backend.path` configuration parameter.

The resource definition file can contain resource definitions or include of subsequent definition files:
```
[
    Resource,    // see Basic resource layout below
    Resource,
    ...
    { "include": String }, // see below
    ...
]
```
where include `String` can be either exact path or a glob pattern (expansion to zero files is not an error) and may be relative to current resource file.

☛ If the file contains single entry (resource or include) the surrounding array is not required.

# Supported resource types/drivers

Mapproxy supports following resource types:
 * tms: *tile map service* (bound layer)
 * surface: surfaces (i.e. tileset generator)
 * geodata: vector free layers, monolithic or tiled

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
 * `Enum` string data type limited to enumerated values
 * `?` no fixed name
 
Complex datatypes:

```javascript
ResourceId: {
    String group        // group this resource belongs to
    String id           // resource identifier (withing group)
}
```

```javascript
Resampling: Enum {
    <GDAL-supported resampling algorithms>
    texture
    dem
}
```
Where `texture` is resampling suitable for rextures (`average` for scales smaller than 0.5, `cubic` otherwise) 
and `dem` is suitable for terrain (`average` for scales smaller than 0.5, `cubicspline` otherwise).

```javascript
Credits: {
    Object <string-id> : {
        Int id
        String notice
    }
}
```
Both string and numeric id should be unique across particular VTS installation. Use numeric ids in range [200, 65535]. Notice supports `{Y}` and `{copy}` templates and hyperlinks using syntax `[<URL> <link text>]`.

```javascript
Registry: {
    Credits credits
}
```

Basic resource definition:

```javascript
Object Resource = {
    String comment      // any comment, ignored
    String group        // group this resource belongs to
    String id           // resource identifier (withing group)
    String type         // data type (tms, surface, geodata)
    String driver       // data generator (see below)
    Registry registry   // additional local resource registry, see above
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

Some resource generators do not support tiling (like `geodata-vector`). In that case tile and lod ranges
can be omitted and `referenceFrames` becomes an array of reference frame IDs.

## URL templates

This is documented elsewhere but as a convenience we provide URL template expansion documentation here.

Each tile has its global and local `tileId`. For simple reference frames (like `webmerc-projected`)
global and local identifers are the same.

For complex reference frames (`melown2015`, `earth-qsc`) global identifier is from tile tree root, i.e. from `0-0-0`.
Local identifier is tile identifier relative to its reference frame subtree.

For example (in `melown2015`):

 * tile with global ID `10-256-256` is in the `pseudomerc` subtree with root at `1-0-0` and its local ID is `9-256-256`
 * tile with global ID `10-768-512` is in the `steres-wgs84` subtree with root at `1-1-0` and its locaal ID is also `9-256-256`

Available expansion strings. Only some make sense for templates used in mapproxy.

 * `{lod}`    global tile LOD
 * `{x}`      global tile X index
 * `{y}`      global tile Y index
 * `{loclod}` local tile LOD
 * `{locx}`   local tile X index
 * `{locy}`   local tile Y index
 * `{sub}`    sub-tile identifier (e.g. submesh index in atlas image)
 * `{srs}`    symbolic name of SRS in the current reference frame subtree
 * `{alt(1,2,3,4)}` exands to one of given strings
 * `{ppx}`    tile's old PP space X index (makes sense only in ppspace)
 * `{ppy}`    tile's old PP space Y index (makes sense only in ppspace)
 * `{Y}`      current year (used in credits definition)
 * `{copy}`   copyright symbol (used in credits definition)
 * `{switch(var,src:dst,src:dst,...,*:dst}`:
     * if value == src then output dst
     * special source value `*` marks default handler
     * special destination value `*` means to output the value as is

## TMS drivers

### Introspection

If browsing is enabled mapproxy handles these URLs:
 * `index.html`: [Leaflet](https://leafletjs.com/)-based boundlayer browser
 * `index.js`: javascript support for (`index.html`)

### Driver: tms-raster

Raster-based bound layer generator. Uses any raster GDAL dataset as its data source. Supports optional data masking.

```javascript
definition = {
    String dataset                 // path to GDAL dataset
    Optional String mask           // path to RF mask or masking GDAL dataset
    Optional String format         // output image format, "jpg" or "png" (defaults to "jpg")
    Optional Boolean transparent   // Boundlayer is transparent, forces format to "png"
    Optional Resampling resampling // Resampling to use for tile texture generation, default 'texture'
}
```

### Driver: tms-raster-remote

Raster bound layer generator. Imagery is pointer to external resource via `remoteUrl` (a URL template). Supports optional data masking.

```javascript
definition = {
    String remoteUrl             // Imagery URL template.
    Optional String mask         // path to RF mask or masking GDAL dataset
}
```


### Driver: tms-patchwork

Simple raster bound layer generator. Generates color checkered tiles. Supports optional data masking.

```javascript
definition = {
    Optional String mask         // path to RF mask or masking GDAL dataset
    Optional String format       // output image format, "jpg" or "png" (defaults to "jpg")
}
```

### Driver: tms-bing

Bound layer generator for remote Bing data. Valid session is generated via metatada URL.

```javascript
definition = {
    String metadataUrl           // Bing API metadata URL. See Bing API documentation for more info.
}
```

## Surface drivers

Surface drivers generate a meshed surface that can be used directly as a single surface or merged into VTS storage as
a remote tileset.
In addition, a `freelayer.json` file is provided allowing generated surface to act as a `mesh-tiles` free layer.

### Introspection interface

If browsing is enabled mapproxy handles URLs:
 * `index.html`: built-in browser

### Cesium terrain provider

If resource's reference frame contains the TMS extension any surface driver will handle URLs for non-VTS [Cesium Terrain Provider](https://cesiumjs.org/Cesium/Build/Documentation/CesiumTerrainProvider.html):
 * `{lod}-{x}-{y}.terrain`: terrain [quantized meshes](https://github.com/AnalyticalGraphicsInc/quantized-mesh)
 * `layer.json`: terrain metadata
 
Note: URL provided tile ID (`url`: lod-x-y) is mapped via TMS extension configuration to real tile ID (`tileId`: lod'-x'-y'). I.e. `urlId`'s y-component is optionally flipped (based on `tms.flipY` option, defaults to `true`) and then shifted under TMS root (`tms.rootId`, defaults to `0-0-0`).

Generated mesh tiles are identical to VTS surface tiles, having these properties:
 * different encoding [format](https://github.com/AnalyticalGraphicsInc/quantized-mesh)
 * while there are no physical skirts border vertices are marked
 * no texture coordinates
 * no extensions (no normal vectors etc.)
 
In addition, if browsing is enabled these introspection URLs are handled
 * `cesium.html`: built-in terrain browser (available even without terrain support)
 * `cesium.js`: javascript support for (`cesium.html`)
 * `cesium.conf`: configuration for built-in introspection browser
 
Nota bene: While it is possible to add the TMS extension to any reference frame please not that it makes only sense for the dedicated `tms-global-geodetic` reference frame. Any other reference frame would not work with 

Nota bene: Please, do not use the `tms-global-geodetic` reference frame for anything else than terrain provider support. This reference frame is absolutely inappropriate for the VTS system because the underlying [projection (eqc)](https://en.wikipedia.org/wiki/Equirectangular_projection) is neither equal area nor [conformal](https://en.wikipedia.org/wiki/Conformal_map_projection). 

However, the only other use the `tms-global-geodetic` reference frame in VTS is the `tms` driver when generating data for [Cesium Imagery Provider](https://cesiumjs.org/Cesium/Build/Documentation/ImageryProvider.html); bear in mind that that there is no extra imagery provider interface and the URL translation must be done in the javascript driver. Introspection interface does it automatically, though.


### Common surface driver configuration options

All surface drivers support these (optional) options:
```javascript
    Optional String geoidGrid           // name of Proj.4's geoid grid file (e.g. `egm96_15.gtx`).
    Optional Double nominalTexelSize    // nominal resolution (meter/pixel);
                                        // reported by mapproxy-calipers
    Optional Int mergeBottomLod         // Reported in generated tileset.conf, speeds up merge
                                        // with other surfaces
    Optional Object heightFunction      // Height manipulation function. See below.
    Optional Object/Array introspection // Introspection info used when using mapConfig.json served
                                        // by mapproxy. See below.
```

Introspection is extended configuration for mapproxy served `mapConfig.json` (only when browsing is enabled).

```javascript
introspection = {
    Optional Array position;                       // VTS position in JSON/python format
    Optional ResourceId/Array<ResourceId> tms      // bound layer(s) mapped on the surface, see below 
    Optional ResourceId/Array<ResourceId> geodata  // free layer(s) (geodata) mapped on the surface, see below
    Optional Object browserOptions                 // browser options passed to mapConfig.json
}
```

Height function is a function that takes the original height value and modifies it. Currently, the only supported
height function is `superelevation`.

```javascript
heightFunction = {
    String function; // must be "superelevation"
    Array<double>[2] heightRange; // source mapping range
    Array<souble>[2] scaleRange; // destination mapping range
}
```

Superelevation maps height from `heightRange` to scale in `scaleRange` and outputs original height scaled by computed scale.
The `heightRange[0]` must be lower than `heightRange[1]` and heights below `heightRange[0]` and above `heightRange[1]` are lipped.

### Driver: surface-spheroid

This driver generates meshed surface for reference frame's spheroid. If geoid grid is provided the resulting body
is in fact a geoid.

```javascript
definition = {
    // see common surface driver configuration options above
}
```

☛ Since `superelevation` has no effect here (all heights are 0) the heightFunction is not implemented yet.

### Driver: surface-dem

This driver generates a meshed surface from the supplied GDAL raster DEM/DSM/DTM dataset.

Since GDAL raster formats are unable to safely store vertical SRS component it cannot tell whether data are
in ellipsoidal or orthometric verical datum. Therefore by default the heights are treated as if they are above the ellipsoid
(i.e. ellipsoidal vertical datum). By providing a `geoidGrid` configuration option we can specify geoid grid
for the orthormetric vertical datum, i.e. to tell that the heights store in the GDAL dataset are relative to given geoid.

Please be aware that due to such limitations the GDAL dataset's vertical system must be compatible with reference frame's
vertical system to use geoid support. I.e. either they share the same ellipsoid or the input data are in some
local system that approximates the geoid at given place. One working example is data in Krovak's projection that can
be reinterpreted as heights above WGS84+EGM96 without any significant error.

All `surface-dem` input datasets are registered in internal map od available DEM's under its `group-id` identifier
and can be referenced from various `geodata`resources for 2D features heightcofing. Optionially, input dataset can
be registered in this map under an alias. See more in the `geodata` resources documentation. 

```javascript
definition = {
    // see common surface driver configuration options above
    
    String dataset                    // path to complex dataset
    Optional String mask              // optional mask, generated by mapproxy-rf-mask tool
    Optional String heightcodingAlias // dataset is registered under given alias
}
```

## Geodata drivers

Geodata drivers generate vector geographic data in the form of VTS free layer.

### Introspection interface

The introspection interface (i.e. the built-in browser) is identical to surface drivers, sans the terrain provider-related stuff.

### Driver: geodata-vector

Generates monolithic free layer (`geodata` type) from an OGR-supported dataset (GeoJSON, shapefile, ...).
Purely 2D data are converted to full 3D data using process called heithgcoding: each 2D coordinate is extented
by height read from the accompanying DEM/DTM/DSM GDAL dataset.

Heightcoding DEM is in the same format a the dataset expected by `surface-dem` driver although only its `/dem`
part is used. This DEM can be accompanied with its geoid grid in the same way as `surface-dem` is.

By default all layers from the source dataset are served. Optionally, layer subset can be configured by providing list
of layer names.

Since there are not tiles generated by this generator the tile and lod ranges are ignored. To make resource
configuration more readable we can omit them completely and use array of reference frame IDs for `referenceFrame`.
For example:
```javascript
{
    ...
    referenceFrame = [ "melown2015", "earth-qsc" ],
    ...
```


The Z-coordinate of all points from the original dataset undergoes heightcoding operation.

```javascript
HeightcodingMode: Enum {
    never  // keep original Z coordinate from source dataset
    always // always replace Z coordinate from source dataset with height from DEM
    auto   // use height from DEM only for 2D points, keep Z coordinate in 3D points
}
```

```javascript
definition = {
    String dataset                 // path to OGR dataset
    String demDataset              // path to complex dem dataset
    Optional String geoidGrid      // name of Proj.4's geoid grid file (e.g. `egm96_15.gtx`)
    Optional Array<String> layers  // list of layer names
    Optional String format         // output file format, so far only "geodataJson" is supported (default)
    Optional Object formatConfig   // format-specific configuration, see below
    Optional String styleUrl       // URL to default geodata style
    Int displaySize                // Nominal size of tile in pixels.
    HeightcodingMode mode          // heightcoding mode (defaults to auto).
    Optional Object enhance        // Per-layer OGR dataset enhancement.
    Optional Object heightFunction // Height manipulation function. Same as for the surface drivers.
    Optional Object introspection  // Extended configuration for mapConfig.json served by mapproxy
}
```

`styleUrl` handling is as follows:
 * if there is no `styleUrl` element present mapproxy serves its built-in default style via `style.json` file under resources URL;
 * or if `styleUrl` element is present and starts with `file:` prefix then contents of this file (either absolute or relative to dataset directory) are server via the same `style.json` file (NB: this is not a file URI);
 * otherwise, the URL from `styleUrl` element is reported in the `freelayer.json` as is
 
Available format configurations:

* `geodataJson`:
```javascript
{
    Int resolution // number of samples the bounding box is divided to, defaults to 4096
}
```

Layer enhancement allows supplying additional data to features from sqlite database. Configuration:

```javascript
enhance = {
   Object ? = {      // layer name
       String key    // name of afeature attribute used as a key to database
       String db     // path to a sqlite database file, relative to dataset directory
       String table  // name of source table inside the sqlite database
   }
}
```

When generator encounteres layer with matching name it tries to get a row from `<db>.<table>` with column `id` (hardcoded name) equal to the value of the attribute `key` for each feature. If matching row is found all other columns are added as attributes to the output.


Height function support is not implemented yet.

Introspection can be used to serve mapConfig where geodata are show with some surface which in turn can have its own
introspection configuration.

```javascript

introspection = {
    Optional ResourceId surface    // optional surface mapping
    Optional Object browserOptions // browser options passed to mapConfig.json
}
```

Browser options override any browser options from associated surface.

### Driver: geodata-vector-tiled

Generates tiled geodata (`geodata-tiles` type) from pre-tiled data like MVT web service or `.mbtiles` archive. Input tiling must match reference frame's space division, at least in one of its nodes. For example, OSM tiles in pseudomerc projection can be used in `webmerc-projected` and `webmerc-unprojected` reference frames and in the `pseudomerc` subtree in in `melown2015` reference frame.

Configuration is the same as for `geodata-vector` driver but input interpretation is different: option `definition.dataset` is:
 * for web services: a URL template that is expanded (see above) for each requested tile before opening and processing.
 * for MBTiles: a path to `.mbtiles` archive with appended template for tiles: `path/to/myvectors.mbtiles/{loclod}-{locx}-{locy}`
Also, per-reference-frame tiling information is mandatory.

Geodata's metatiles are generated purely from heightcoding GDAL dataset.

Extra parameters available in this driver:

```javascript
definition += {
    Optional Int maxSourceLod // Maximum available LOD in the source data. Detailed LODs
                              // will be generated from coarser tiles at maxSourceLod.
                              // LOD is in local subtree.
    Optional Array<String> clipLayers // list of layers that are clipped to tile extents (in spatial division SRS)
}
```
