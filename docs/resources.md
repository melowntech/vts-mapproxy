# Preamble

This file is the authoritative mapproxy resource documentation. Please, keep it in sync with the actual implementation.

# Resource definition file

Most common way to define resources for mapproxy is to provide JSON resource definition file and point mapproxy to it by specifying its path in `resource-backend.path` configuration parameter.

The resource definition file can contain resource definitions or include
subsequent definition files:
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

Basic resource definition:

```javascript
Object Resource = {
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
 * `{sub}`    sub-tile identifier (e.g. submesh index in atlas image)'
 * `{alt(1,2,3,4)}` exands to one of given strings
 * `{ppx}`    tile's old PP space X index (makes sense only in ppspace)
 * `{ppy}`    tile's old PP space Y index (makes sense only in ppspace)
 
## TMS drivers

### tms-raster

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

Surface drivers generate a meshed surface that can be used directly as a single surface or merged into VTS storage as
a remote tileset.
In addition, a `freelayer.json` file is provided allowing generated surface to act as a `mesh-tiles` free layer.

### Common surface driver configuration options

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

Introspection is extended configuration for mapproxy served `mapConfig.json` (only when browsing is enabled).

```javascript
introspection = {
    Optional Array position;                       // VTS position in JSON/python format
    Optional ResourceId/Array<ResourceId> tms      // bound layer(s) mapped on the surface, see below 
    Optional ResourceId/Array<ResourceId> geodata  // free layer(s) (geodata) mapped on the surface, see below
    Optional Object browserOptions                 // browser options passed to mapConfig.json
}
```

### surface-spheroid

This driver generates meshed surface for reference frame's spheroid. If geoid grid is provided the resulting body
is in fact a geoid.

```javascript
definition = {
    // see common surface driver configuration options above
}
```

### surface-dem

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

### geodata-vector

Generates monolithic free layer (`geodata` type) from an OGR-supported dataset (GeoJSON, shapefile, ...).
Purely 2D data are converted to full 3D data using process called heithgcoding: each 2D coordinate is extented
by height read from the accompanying DEM/DTM/DSM GDAL dataset.

Heightcoding DEM is in the same format a the dataset expected by `surface-dem` driver although only its `/dem`
part is used. This DEM can be accompanied with its geoid grid in the same way as `surface-dem` is.

By default all layers from the source dataset are served. Optionally, layer subset can be configured by providing list
of layer names.

```javascript
definition = {
    String dataset                 // path to OGR dataset
    String demDataset              // path to complex dem dataset
    Optional String geoidGrid      // name of Proj.4's geoid grid file (e.g. `egm96_15.gtx`)
    Optional Array<String> layers  // list of layers names
    Optional String format         // output file format, so far only "geodataJson" is supported (default)
    Optional String styleUrl       // URL to default geodata style
    Int displaySize                // Nominal size of tile in pixels.
    Optional Object introspection  // Extended configuration for mapConfig.json served by mapproxy
}
```

`styleUrl` handling is as follows:
 * if there is no `styleUrl` element present mapproxy serves its built-in default style via `style.json` file under resources URL;
 * or if `styleUrl` element is present and starts with `file:` prefix then contents of this file (either absolute or relative to dataset directory) are server via the same `style.json` file (NB: this is not a file URI);
 * otherwise, the URL from `styleUrl` element is reported in the `freelayer.json` as is

Introspection can be used to serve mapConfig where geodata are show with some surface which in turn can have its own
introspection configuration.

```javascript

introspection = {
    Optional ResourceId surface    // optional surface mapping
    Optional Object browserOptions // browser options passed to mapConfig.json
}
```

Browser options override any browser options from associated surface.

### geodata-vector-tiled

Generates tiled geodata (`geodata-tiles` type) from pre-tiled data like MVT web service or `.mbtiles` archive. Input tiling must match reference frame's space division, at least in one of its nodes. For example, OSM tiles in pseudomerc projection can be used in `webmerc-projected` and `webmerc-unprojected` reference frames and in the `pseudomerc` subtree in in `melown2015` reference frame.

Configuration is the same as for `geodata-vector` driver but input interpretation is different: option `definition.dataset` is:
 * for web services: a URL template that is expanded (see above) for each requested tile before opening and processing.
 * for MBTiles: a path to `.mbtiles` archive with appended template for tiles: `path/to/myvectors.mbtiles/{loclod}-{locx}-{locy}`

Geodata's metatiles are generated purely from heightcoding GDAL dataset.

