// disable Cesium ion
Cesium.Ion.defaultAccessToken = "";
Cesium.Ion.defaultServer = "";

function loadJson(url, type, callback) {
    var x = new XMLHttpRequest();
    x.overrideMimeType(type);
    x.open("GET", url, true);
    x.onreadystatechange = function () {
        if ((x.readyState) == 4 && ((x.status == 200) || (x.status == 0)))
        {
            callback(JSON.parse(x.responseText));
        }
    };
    x.send(null);
}

function launchCesium(imageryProvider) {
    // custom terrain provider
    var terrainProvider = new Cesium.CesiumTerrainProvider({
        url: "."
    });

    // viewer: disable almost everything, use custom imagery and terrain providers
    var viewer = new Cesium.Viewer('cesiumContainer', {
        geocoder: false
        , baseLayerPicker: false
        , sceneModePicker: false
        , homeButton: false
        , infoBox: false
        , timeline: false
        , animation: false
        , scene3DOnly: true
        , navigationInstructionsInitiallyVisible: false
        , selectionIndicator: false
        , requestRenderMode : true
        , imageryProvider: imageryProvider
        , terrainProvider: terrainProvider
    });

    var melownCredit = new Cesium.Credit
        ('<a href="https://www.melown.com/">'
         + '<img src="melowntech.png" title="Melown Technologies SE" />'
         + '</a>');
    viewer.scene.frameState.creditDisplay.addDefaultCredit(melownCredit);
}

function resolveUrl(url, base) {
    return (new URL(url, base)).href;
}

function processBoundLayer(config, bl) {
    // resolve tile URL templace
    var tileUrl = resolveUrl(bl.url, config.boundLayer);

    // default
    var imageryProvider;

    if (config.tms.profile == "global-geodetic") {
        imageryProvider = new Cesium.UrlTemplateImageryProvider({
            url: tileUrl
            , minimumLevel: bl.lodRange[0] - config.tms.rootId.lod
            , maximumLevel: bl.lodRange[1] - config.tms.rootId.lod
            , tilingScheme: new Cesium.GeographicTilingScheme()
            , customTags: {
                "lod": function(ip, x, y, level) {
                    return level + config.tms.rootId.lod
                }
                // TODO: apply rootId to x and y as well if needed
            }
        });

        // TODO: add "global-mercator" handling
    } else {
        imageryProvider = new Cesium.GridImageryProvider({});
    }

    launchCesium(imageryProvider);
}

function processConfig(config) {
    if (config.boundLayer) {
        config.boundLayer = resolveUrl(config.boundLayer, document.URL);
        return loadJson(config.boundLayer, "application/json"
                        , processBoundLayer.bind(processBoundLayer, config));
    }
    launchCesium(new Cesium.GridImageryProvider({}));
}

function startBrowser() {
    loadJson("cesium.conf", "application/json", processConfig);
}
