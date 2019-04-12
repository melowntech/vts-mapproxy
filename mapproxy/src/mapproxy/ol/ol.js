function createMap(xml) {
    var getWidth = ol.extent.getWidth;
    var getProjection = ol.proj.get;
    var transform = ol.proj.transform;
    var WMTS = ol.source.WMTS;

    var Map = ol.Map;
    var View = ol.View;
    var WMTSCapabilities = ol.format.WMTSCapabilities;
    var TileLayer = ol.layer.Tile;
    var WMTS = ol.source.WMTS;
    var optionsFromCapabilities = WMTS.optionsFromCapabilities;

    var map;

    var capabilities = (new WMTSCapabilities()).read(xml);

    // use first layer
    var layer = capabilities.Contents.Layer[0];

    // use single tile matrix
    var options = optionsFromCapabilities(capabilities, {
        layer: layer.Identifier
    });

    // get center of WGS84 bounding box and convert it to SRS of tile matrix
    var bb = layer.WGS84BoundingBox;
    var center = transform([ (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2 ]
                           , getProjection("CRS:84")
                           ,  options.projection);

    // calculate allowed resolutions for tile-matrix-set
    var tms = capabilities.Contents.TileMatrixSet.find(
        x => x.Identifier === layer.TileMatrixSetLink[0].TileMatrixSet
    );
    var projection = getProjection(options.projection);
    var matrix = tms.TileMatrix[0];
    const width = getWidth(projection.getExtent());
    var resolutions = tms.TileMatrix.map(
        m => width / m.TileWidth / m.MatrixWidth
    );

    map = new Map({
        layers: [
            new TileLayer({
                opacity: 1
                , source: new WMTS(options)
            })
        ],
        target: 'map',
        view: new View({
            center: center
            , projection: options.projection
            , resolutions: resolutions
            , resolution: resolutions[0]
        })
    });
}

function startBrowser() {
    fetch('./WMTSCapabilities.xml?is=1')
        .then((response) => response.text())
        .then((text) => createMap(text))
    ;
}
