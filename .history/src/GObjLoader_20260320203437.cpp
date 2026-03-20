#ifndef G_OBJ_LOADER_HPP
#define G_OBJ_LOADER_HPP

#include <string>

#include "GModel.hpp"

namespace G {

    class ObjLoader {
    public:
        [[nodiscard]] static Model load(const std::string& objPath);
    };

} // namespace G

#endif