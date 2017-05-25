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
    String dataset         // path to GDAL dataset
    Optional String mask   // path to RF mask or masking GDAL dataset
    Optional String format // output image format, "jpg" or "png" (defaults to "jpg")
    Optional Boolean transparent // Boundlayer is transparent, forces format to "png"
}
```