/**
 * Copyright (c) 2018 Melown Technologies SE
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

#ifndef mapproxy_generator_heightfunction_hpp_included_
#define mapproxy_generator_heightfunction_hpp_included_

#include <memory>
#include <boost/any.hpp>

#include "vts-libs/storage/range.hpp"

/** Generic height function for surface manipulation.
 */
class HeightFunction {
public:
    typedef std::shared_ptr<HeightFunction> pointer;

    virtual ~HeightFunction() {}
    virtual double operator()(double h) const = 0;

    static HeightFunction::pointer parse(const Json::Value &value
                                         , const std::string &key);
    static bool changed(const HeightFunction::pointer &l
                        , const HeightFunction::pointer &r);

    virtual void build(Json::Value &value) const = 0;

    virtual bool changed(const HeightFunction::pointer &other) const = 0;
};

typedef vtslibs::storage::Range<double> HeightRange;
typedef vtslibs::storage::Range<double> ScaleRange;

class SuperElevation : public HeightFunction {
public:
    struct Config {
        HeightRange heightRange;
        ScaleRange scaleRange;

        bool changed(const Config &other) const;
    };

    SuperElevation(const Config &config);

    virtual ~SuperElevation() {}

    virtual double operator()(double h) const {
        if (h < config_.heightRange.min) { return limits_.min; }
        else if (h > config_.heightRange.max) { return limits_.max; }

        return apply(h);
    }

    virtual void build(Json::Value &value) const;
    virtual bool changed(const HeightFunction::pointer &other) const;

private:
    double apply(double h) const {
        return h * ((h - config_.heightRange.min) * s_
                    + config_.scaleRange.min);
    }

    /** Configuration;
     */
    Config config_;

    /** Pre-computed scaling factor.
     */
    double s_;

    /** Pre-computed value for heightRange extremes.
     */
    HeightRange limits_;
};

#endif // mapproxy_generator_heightfunction_hpp_included_
