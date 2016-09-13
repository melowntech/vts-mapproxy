#ifndef mapproxy_resource_hpp_included_
#define mapproxy_resource_hpp_included_

#include <memory>
#include <iostream>

#include <boost/filesystem/path.hpp>
#include <boost/any.hpp>
#include <boost/optional.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/basetypes.hpp"

#include "./error.hpp"

#include "./support/fileclass.hpp"

/** Base of all resource definitions.
 */
class DefinitionBase {
public:
    typedef std::shared_ptr<DefinitionBase> pointer;
    virtual ~DefinitionBase() {}

    void from(const boost::any &value) { from_impl(value); }
    void to(boost::any &value) const { to_impl(value); }
    bool same(const DefinitionBase &other) const { return same_impl(other); }

    /** Are credits frozen in the resources's dataset?
     */
    bool frozenCredits() const { return frozenCredits_impl(); }

    template <typename T> const T& as() const {
        if (const auto *value = dynamic_cast<const T*>(this)) {
            return *value;
        }
        LOGTHROW(err1, Error)
            << "Incompatible resource definitions: cannot convert <"
            << typeid(*this).name() << "> into <" << typeid(T).name() << ">.";
        throw;
    }

private:
    /** Fills in this definition from given input value.
     *  Value can be either Json::Value or boost::python::dict.
     */
    virtual void from_impl(const boost::any &value) = 0;

    /** Fills in this definition into given output value.
     *  Value can be either Json::Value or boost::python::dict.
     */
    virtual void to_impl(boost::any &value) const = 0;

    /** Compares this resource definition the other one and check whether they
     *  are basically the same. The definitions can differ but the difference
     *  must not affect resource generation.
     */
    virtual bool same_impl(const DefinitionBase &other) const = 0;

    /** If generated dataset freezes credit info into its published data then
     *  credits cannot be changed.
     */
    virtual bool frozenCredits_impl() const { return true; }
};

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
        enum Type { tms, surface, geodata };
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
    std::string comment;

    /** Data root from configuration.
     */
    boost::filesystem::path root;

    DualId::set credits;

    const vr::ReferenceFrame *referenceFrame;
    vr::LodRange lodRange;
    vr::TileRange tileRange;

    vr::RegistryBase registry;

    FileClassSettings fileClassSettings;

    DefinitionBase::pointer definition() const {
        return definition_;
    }

    template <typename T> const T& definition() const {
        return definition_->as<T>();
    }

    void definition(const DefinitionBase::pointer &definition) {
        definition_ = definition;
    }

    typedef std::map<Id, Resource> map;
    typedef std::vector<Resource> list;

    Resource(const FileClassSettings &fileClassSettings)
        : fileClassSettings(fileClassSettings)
    {}

    bool operator==(const Resource &o) const;
    bool operator!=(const Resource &o) const;

private:
    /** Definition: based on type and driver, created by resource
     *  parser/generator and interpreted by driver.
     */
    DefinitionBase::pointer definition_;
};

vr::Credits asInlineCredits(const Resource &res);

UTILITY_GENERATE_ENUM_IO(Resource::Generator::Type,
    ((tms))
    ((surface))
    ((geodata))
)

UTILITY_GENERATE_ENUM(RasterFormat,
    ((jpg))
    ((png))
)

constexpr RasterFormat MaskFormat = RasterFormat::png;
constexpr RasterFormat RasterMetatileFormat = RasterFormat::png;

/** Resource root: used to build relative paths from given root.
 */
struct ResourceRoot {
    /** Depth in the virtual filesystem tree.
     *
     * NB: no enum class to allow usage Resource::xxx (as it is used throughout
     * the code.
     */
    enum Depth : int {
        referenceFrame = 0, type = 1, group = 2, id = 3, none = 4
    };

    /** Root location.
     */
    Depth depth;

    /** How many times to go up the directory tree before adding current root
     *  directory. Each level is one ".."
     */
    int backup;

    ResourceRoot(Depth depth = none, int backup = 0)
        : depth(depth), backup(backup)
    {}

    operator Depth() const { return depth; }

    int depthDifference(ResourceRoot::Depth other) const {
        return depth - other;
    }
};

/** Computes path from this resource to that resource.
 */
ResourceRoot resolveRoot(const Resource &thisResource
                         , const Resource &thatResource
                         , ResourceRoot::Depth thisDepth = ResourceRoot::none);

/** What directory is resource root:
 */
UTILITY_GENERATE_ENUM_IO(ResourceRoot::Depth,
    ((referenceFrame))
    ((type))
    ((group))
    ((id))
    ((none))
)

typedef boost::function<void(const Resource::Id&, const std::string&)>
ResourceLoadErrorCallback;

/** Load resources from given path.
 */
Resource::map loadResources(const boost::filesystem::path &path
                            , ResourceLoadErrorCallback error
                            , const FileClassSettings &fileClassSettings
                            = FileClassSettings());

/** Load single resource from given path.
 */
Resource::list loadResource(const boost::filesystem::path &path
                            , const FileClassSettings &fileClassSettings
                            = FileClassSettings());

/** Load resources from Python list (passed as a boost::any to hide
 *  implementation)
 */
Resource::map loadResourcesFromPython(const boost::any &pylist
                                      , ResourceLoadErrorCallback error
                                      , const FileClassSettings
                                      &fileClassSettings
                                      = FileClassSettings());

/** Save single resource to given path.
 */
void save(const boost::filesystem::path &path, const Resource &resource);

boost::filesystem::path prependRoot(const boost::filesystem::path &path
                                    , const Resource &resource
                                    , const ResourceRoot &root);

std::string prependRoot(const std::string &path, const Resource &resource
                        , const ResourceRoot &root);

std::string contentType(RasterFormat format);

enum class RangeType { lod, tileId };

bool checkRanges(const Resource &resource, const vts::TileId &tileId
                 , RangeType rangeType = RangeType::tileId);

/** Combine resource with given reference frame.
 */
Resource::Id addReferenceFrame(Resource::Id rid, std::string referenceFrame);

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

inline Resource::Id addReferenceFrame(Resource::Id rid
                                      , std::string referenceFrame)
{
    rid.referenceFrame = std::move(referenceFrame);
    return rid;
}

#endif // mapproxy_resource_hpp_included_
