#include <map>
#include <mutex>

#include "./demregistry.hpp"

class DemRegistry::Detail {
public:
    Detail() {}

    Datasets find(const std::string &referenceFrame
                  , const std::vector<std::string> &ids) const
    {
        std::unique_lock<std::mutex> lock(mutex_);

        Datasets datasets;
        for (const auto &id : ids) {
            auto fmap(map_.find({ referenceFrame, id }));
            if (fmap != map_.end()) {
                datasets.push_back(fmap->second.dataset);
            }
        }
        return datasets;
    }

    void add(const Record &record) {
        std::unique_lock<std::mutex> lock(mutex_);
        map_.insert(Map::value_type(record.id, record));
    }

    void remove(const Id &id) {
        std::unique_lock<std::mutex> lock(mutex_);
        map_.erase(id);
    }

    Record::list records(const std::string &referenceFrame) const {
        Record::list records;

        std::unique_lock<std::mutex> lock(mutex_);
        for (const auto &item : map_) {
            if (item.first.referenceFrame == referenceFrame) {
                records.push_back(item.second);
            }
        }

        return records;
    }

private:
    mutable std::mutex mutex_;
    typedef std::map<Id, Record> Map;
    Map map_;
};

DemRegistry::DemRegistry()
    : detail_(std::make_shared<Detail>())
{}

DemRegistry::~DemRegistry() {}

DemRegistry::Datasets DemRegistry::find(const std::string &referenceFrame
                                        , const std::vector<std::string> &ids)
    const
{
    return detail().find(referenceFrame, ids);
}

void DemRegistry::add(const Record &record)
{
    return detail().add(record);
}

void DemRegistry::remove(const Id &id)
{
    return detail().remove(id);
}

DemRegistry::Record::list
DemRegistry::records(const std::string &referenceFrame) const
{
    return detail().records(referenceFrame);
}
