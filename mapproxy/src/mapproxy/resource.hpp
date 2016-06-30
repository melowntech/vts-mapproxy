#ifndef mapproxy_resource_hpp_included_
#define mapproxy_resource_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>
#include <boost/any.hpp>
#include <boost/optional.hpp>

#include "utility/enum-io.hpp"

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/basetypes.hpp"

namespace vr = vadstena::registry;
namespace vts = vadstena::vts;

struct DualId {
    std::string id;
    int numId;

    DualId(const std::string &id = "", int numId = 0)
        : id(id), numId(numId)
    {}

    operator std::string() const { return id; }
    operator int() const { return numId; }

    bool operator<(const DualId &o) const {
        return (id < o.id);
    }

    typedef std::set<DualId> set;
};

inline vr::StringIdSet asStringSet(const DualId::set &set)
{
    return vr::StringIdSet(set.begin(), set.end());
}

inline vr::Credits asCredits(const DualId::set &set)
{
    vr::Credits credits;
    for (auto &id : set) {
        credits.set(id, boost::none);
    }
    return credits;
}

vr::Credits asInlineCredits(const DualId::set &set);

inline vr::IdSet asIntSet(const DualId::set &set)
{
    return vr::IdSet(set.begin(), set.end());
}

struct Resource {
    struct Id {
        std::string referenceFrame;
        std::string group;
        std::string id;

        std::string fullId() const { return group + "-" + id; }

        Id() {}
        Id(const std::string &referenceFrame, const std::string &group
           , const std::string &id)
            : referenceFrame(referenceFrame), group(group), id(id) {}
        bool operator<(const Id &o) const;
        bool operator==(const Id &o) const;
    };

    struct Generator {
        enum Type { tms, surface };
        Type type;
        std::string driver;

        Generator() {}
        Generator(Type type, const std::string &driver)
            : type(type), driver(driver) {}
        bool operator<(const Generator &o) const;
        bool operator==(const Generator &o) const;
    };

    Id id;
    Generator generator;

    /** Data root from configuration.
     */
    boost::filesystem::path root;

    DualId::set credits;

    const vr::ReferenceFrame *referenceFrame;
    vr::LodRange lodRange;
    vr::TileRange tileRange;

    template <typename T> const T& definition() const {
        return boost::any_cast<const T&>(definition_);
    }

    template <typename T> T& assignDefinition(const T &definition = T()) {
        return boost::any_cast<T&>(definition_ = definition);
    }

    typedef std::map<Id, Resource> map;
    typedef std::vector<Resource> list;

    Resource() {}

    bool operator==(const Resource &o) const;
    bool operator!=(const Resource &o) const;

private:
    /** Definition: based on type and driver, created by resource
     *  parser/generator and interpreted by driver.
     */
    boost::any definition_;
};

UTILITY_GENERATE_ENUM_IO(Resource::Generator::Type,
    ((tms))
    ((surface))
)

UTILITY_GENERATE_ENUM(RasterFormat,
    ((jpg))
    ((png))
)

constexpr RasterFormat MaskFormat = RasterFormat::png;
constexpr RasterFormat RasterMetatileFormat = RasterFormat::png;

/** What directory is resource root:
 */
UTILITY_GENERATE_ENUM(ResourceRoot,
    ((referenceFrame))
    ((type))
    ((group))
    ((id))
    ((none))
)

namespace resdef {

struct TmsRaster {
    static Resource::Generator generator;

    std::string dataset;
    boost::optional<std::string> mask;
    RasterFormat format;

    TmsRaster(): format(RasterFormat::jpg) {}
    bool operator==(const TmsRaster &o) const;
};

struct TmsRasterRemote {
    static Resource::Generator generator;

    std::string remoteUrl;
    boost::optional<std::string> mask;

    TmsRasterRemote() {}
    bool operator==(const TmsRasterRemote &o) const;
};

struct SurfaceSpheroid {
    static Resource::Generator generator;

    unsigned int textureLayerId;
    boost::optional<std::string> geoidGrid;

    SurfaceSpheroid() : textureLayerId() {}
    bool operator==(const SurfaceSpheroid &o) const;
};

struct SurfaceDem {
    static Resource::Generator generator;

    std::string dataset;
    boost::optional<boost::filesystem::path> mask;
    unsigned int textureLayerId;
    boost::optional<std::string> geoidGrid;

    SurfaceDem() : textureLayerId() {}
    bool operator==(const SurfaceDem &o) const;
};

} // namespace resdef

/** Load resources from given path.
 */
Resource::map loadResources(const boost::filesystem::path &path);

/** Load single resource from given path.
 */
Resource::list loadResource(const boost::filesystem::path &path);

/** Save single resource to given path.
 */
void save(const boost::filesystem::path &path, const Resource &resource);

boost::filesystem::path prependRoot(const boost::filesystem::path &path
                                    , const Resource &resource
                                    , ResourceRoot root);

std::string prependRoot(const std::string &path, const Resource &resource
                        , ResourceRoot root);

std::string contentType(RasterFormat format);

enum class RangeType { lod, tileId };

bool checkRanges(const Resource &resource, const vts::TileId &tileId
                 , RangeType rangeType = RangeType::tileId);

// inlines + IO

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Resource::Id &rid)
{
    return os << rid.referenceFrame << '/' << rid.group << '/' << rid.id;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Resource::Generator &g)
{
    return os << g.type << '/' << g.driver;
}

inline bool Resource::Id::operator<(const Id &o) const {
    if (referenceFrame < o.referenceFrame) { return true; }
    else if (o.referenceFrame < referenceFrame) { return false; }

    if (group < o.group) { return true; }
    else if (o.group < group) { return false; }

    return id < o.id;
}

inline bool Resource::Id::operator==(const Id &o) const {
    return ((referenceFrame == o.referenceFrame)
            && (group == o.group)
            && (id == o.id));
}

inline bool Resource::Generator::operator<(const Generator &o) const {
    if (type < o.type) { return true; }
    else if (o.type < type) { return false; }
    return driver < o.driver;
}

inline bool Resource::Generator::operator==(const Generator &o) const {
    return ((type == o.type)
            && (driver == o.driver));
}

inline bool Resource::operator!=(const Resource &o) const
{
    return !operator==(o);
}

#endif // mapproxy_resource_hpp_included_
