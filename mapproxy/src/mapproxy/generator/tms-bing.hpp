#ifndef mapproxy_generator_tms_bing_hpp_included_
#define mapproxy_generator_tms_bing_hpp_included_

#include <functional>

#include "../generator.hpp"

namespace generator {

class TmsBing : public Generator {
public:
    TmsBing(const Params &params);

    struct Definition : public DefinitionBase {
        std::string metadataUrl;

        Definition() {}

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual Changed changed_impl(const DefinitionBase &other) const;
        virtual bool frozenCredits_impl() const { return false; }
    };

private:
    virtual void prepare_impl(Arsenal &arsenal);
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    vr::BoundLayer boundLayer(ResourceRoot root, const std::string &url) const;

    const Definition &definition_;
};

} // namespace generator

#endif // mapproxy_generator_tms_bing_hpp_included_
