/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
     *  an empty set. Result's second value is true only when all arguments have
     *  been found.
     *
     *  Duplicates are automatically removed (first occurence is returned).
     *
     * Datasets have the same order as ids.
     */
    std::pair<DemDataset::list, bool>
    find(const std::string &referenceFrame
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
