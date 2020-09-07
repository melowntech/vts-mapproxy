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

#ifndef mapproxy_definition_factory_hpp_included_
#define mapproxy_definition_factory_hpp_included_

#include <functional>

#include "utility/premain.hpp"
#include "utility/gccversion.hpp"

#include "../resource.hpp"

namespace resource {

/** Create definition based on the generator type.
 */
DefinitionBase::pointer definition(const Resource::Generator &type);

/** Register definition based on generator type.
 */
void registerDefinition(const Resource::Generator &type
                        , const std::function<DefinitionBase::pointer()>
                        &factory);

template <typename Definition> void registerDefinition();

// inlines

template <typename Definition> void registerDefinition() {
    registerDefinition({ Definition::type, Definition::driverName }
                       , []() -> DefinitionBase::pointer {
                           return std::make_shared<Definition>();
                       });
}

#define MAPPROXY_DEFINITION_REGISTER_ALN(x, y)      \
    MAPPROXY_DEFINITION_REGISTER_ALN1(x, y)
#define MAPPROXY_DEFINITION_REGISTER_ALN1(x, y) x##y

#define MAPPROXY_DEFINITION_REGISTER(DEFINITION)                        \
    namespace {                                                         \
        utility::PreMain MAPPROXY_DEFINITION_REGISTER_ALN               \
            (mapproxy_regdef_, DEFINITION)                              \
            ([]() { registerDefinition<DEFINITION>(); }); \
    } // namespace

} // namespace resource

#endif // mapproxy_definition_factory_hpp_included_
