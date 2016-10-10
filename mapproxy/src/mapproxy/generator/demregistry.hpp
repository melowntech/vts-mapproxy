#ifndef mapproxy_demregistry_hpp_included_
#define mapproxy_demregistry_hpp_included_

#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "../resource.hpp"
#include "../support/geo.hpp"

/** Special class to store links to all available DEMs.
 */
class DemRegistry {
public:
    typedef std::shared_ptr<DemRegistry> pointer;

    DemRegistry();
    ~DemRegistry();

    /** Simplified resource ID.
     */
    struct Id {
        std::string referenceFrame;
        std::string id;

        Id(const std::string &referenceFrame, const std::string &id)
            : referenceFrame(referenceFrame), id(id) {}

        bool operator<(const Id &o) const;
        bool operator==(const Id &o) const;

        typedef std::vector<Id> list;
    };

    struct Record {
        Id id;
        DemDataset dataset;
        Resource::Id resourceId;

        Record(const Id &id, const DemDataset &dataset
               , const Resource::Id &resourceId)
            : id(id), dataset(dataset), resourceId(resourceId)
        {}

        typedef std::vector<Record> list;
    };

    /** Tries to find list of datasets. May return less then asked for or even
     *  an empty set.
     *  Datasets have the same order as ids.
     */
    DemDataset::list find(const std::string &referenceFrame
                          , const std::vector<std::string> &ids) const;

    /** Registers DEM under given ID.
     */
    void add(const Record &record);

    /** De-registers DEM under given ID.
     */
    void remove(const Id &id);

    Record::list records(const std::string &referenceFrame) const;

    // internals
    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

// inlines

inline bool DemRegistry::Id::operator<(const Id &o) const {
    if (referenceFrame < o.referenceFrame) { return true; }
    else if (o.referenceFrame < referenceFrame) { return false; }

    return id < o.id;
}

inline bool DemRegistry::Id::operator==(const Id &o) const {
    return ((referenceFrame == o.referenceFrame)
            && (id == o.id));
}

#endif // mapproxy_demregistry_hpp_included_
