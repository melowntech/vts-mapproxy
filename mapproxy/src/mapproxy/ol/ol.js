var map;

function startBrowser() {
    var getWidth = ol.extent.getWidth;
    var getTopLeft = ol.extent.getTopLeft;
    var getProjection = ol.proj.get;
    var WMTS = ol.source.WMTS;

    var Map = ol.Map;
    var View = ol.View;
    var WMTSCapabilities = ol.format.WMTSCapabilities;
    var TileLayer = ol.layer.Tile;
    var WMTS = ol.source.WMTS;
    var optionsFromCapabilities = WMTS.optionsFromCapabilities;

    var parser = new WMTSCapabilities();

    fetch('./WMTSCapabilities.xml').then(function(response) {
        return response.text();
    }).then(function(text) {
        var capabilities = parser.read(text);

        // TODO: use first layer from capabilities

        var options = optionsFromCapabilities(capabilities, {
            layer: 'bmng',
            matrixSet: 'EPSG:3857'
        });

        map = new Map({
            layers: [
                new TileLayer({
                    opacity: 1,
                    source: new WMTS(options)
                })
            ],
            target: 'map',
            view: new View({
                center: [0, 0],
                zoom: 0
            })
        });
    });
}
