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

function credit2html(credit) {
    var notice = credit.notice;
    var out = "";
    var i = 0;

    var escaped = function(c) {
        switch (c) {
        case '&': return '&amp;';
        case '<': return '&lt;';
        case '>': return '&gt;';
        case '"': return '&quot;';
        }
        return c;
    }

    var escapedString = function(text) {
        for (var i = 0; i < text.length; ++i) {
            out += escaped(text[i]);
        }
    }

    var replace = function(what) {
        if (what == "{copy}") { out += "&copy;"; return; }
        if (what == "{Y}") { out += new Date().getFullYear(); return; }
    }

    var beginA = function(url) {
        out += '<a href="';
        escapedString(url);
        out += '">';
    }

    var endA = function() {
        out += '</a>';
    }

    var parseTo = function(end) {
        var out = "";
        while (i < notice.length) {
            var c = notice[i];
            out += c;
            if (c == end) { break; }
            ++i;
        }
        return out;
    }

    var url = function(what) {
        var url = ""
        var text = ""
        var split = false;

        for (var i = 0; i < what.length; ++i) {
            var c = what[i];
            switch (c) {
            case '[': case ']': continue;
            case ' ':
                if (!split) {
                    split = true;
                } else {
                    text += c;
                }
                break;
            default:
                if (split) {
                    text += c;
                } else {
                    url += c;
                }
                break;
            }
        }

        beginA(url);
        escapedString(text.length ? text : url);
        endA();
    }

    var hasUrl = (typeof credit.url !== "undefined");
    if (hasUrl) { beginA(credit.url); }

    for (i = 0; i < notice.length; ++i) {
        var c = credit.notice[i];

        switch (c) {
        case '{': replace(parseTo('}')); break;
        case '[': url(parseTo(']')); break;
        default: out += escaped(c); break;
        }
    }

    if (hasUrl) { endA(); }

    return out;
}

function credits2html(credits) {
    var out = ""
    var separator = "";
    for (key in credits) {
        if (!credits.hasOwnProperty(key)) continue;

        var credit = credits[key];
        out += separator;
        out += credit2html(credit);
        separator = "<br>";
    }
    return out;
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
            , credit: credits2html(bl.credits)
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
