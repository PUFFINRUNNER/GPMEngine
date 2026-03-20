#include "GObjLoader.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "GAllegroTexture.hpp"

namespace {
    [[nodiscard]] std::string trim(const std::string& s) {
        std::size_t a = 0;
        while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) {
            ++a;
        }

        std::size_t b = s.size();
        while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) {
            --b;
        }

        return s.substr(a, b - a);
    }

    [[nodiscard]] std::string directoryOf(const std::string& path) {
        const std::size_t p = path.find_last_of("/\\");
        if (p == std::string::npos) {
            return ".";
        }
        return path.substr(0, p);
    }

    [[nodiscard]] std::string joinPath(const std::string& dir, const std::string& file) {
        if (file.empty()) {
            return dir;
        }
        if (dir.empty() || dir == ".") {
            return file;
        }
        return dir + "/" + file;
    }

    struct ObjIndex {
        int v = 0;
        int vt = 0;
        int vn = 0;

        [[nodiscard]] bool operator==(const ObjIndex& other) const noexcept {
            return v == other.v && vt == other.vt && vn == other.vn;
        }
    };

    struct ObjIndexHash {
        [[nodiscard]] std::size_t operator()(const ObjIndex& x) const noexcept {
            std::size_t h = 1469598103934665603ull;
            auto mix = [&](int value) {
                h ^= static_cast<std::size_t>(value + 0x9e3779b9);
                h *= 1099511628211ull;
            };
            mix(x.v);
            mix(x.vt);
            mix(x.vn);
            return h;
        }
    };

    struct ParsedMaterial {
        G::Color albedo = G::Color::white();
        std::string diffuseTexturePath;
    };

    struct ObjGroup {
        std::string materialName;
        std::vector<G::Triangle> triangles;
    };

    [[nodiscard]] int resolveObjIndex(int idx, int count) {
        if (idx > 0) {
            return idx - 1;
        }
        if (idx < 0) {
            return count + idx;
        }
        return -1;
    }

    [[nodiscard]] ObjIndex parseFaceIndex(const std::string& token) {
        ObjIndex out{0, 0, 0};

        std::size_t p1 = token.find('/');
        if (p1 == std::string::npos) {
            out.v = std::stoi(token);
            return out;
        }

        std::size_t p2 = token.find('/', p1 + 1);

        out.v = std::stoi(token.substr(0, p1));

        if (p2 == std::string::npos) {
            const std::string t1 = token.substr(p1 + 1);
            if (!t1.empty()) {
                out.vt = std::stoi(t1);
            }
            return out;
        }

        const std::string t1 = token.substr(p1 + 1, p2 - p1 - 1);
        const std::string t2 = token.substr(p2 + 1);

        if (!t1.empty()) {
            out.vt = std::stoi(t1);
        }
        if (!t2.empty()) {
            out.vn = std::stoi(t2);
        }

        return out;
    }

    [[nodiscard]] std::unordered_map<std::string, ParsedMaterial>
    loadMtl(const std::string& mtlPath) {
        std::unordered_map<std::string, ParsedMaterial> materials;

        std::ifstream file(mtlPath);
        if (!file) {
            throw std::std
            return materials;
        }

        const std::string baseDir = directoryOf(mtlPath);

        std::string line;
        std::string currentName;
        ParsedMaterial currentMat;

        auto flushCurrent = [&]() {
            if (!currentName.empty()) {
                materials[currentName] = currentMat;
            }
        };

        while (std::getline(file, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::istringstream iss(line);
            std::string cmd;
            iss >> cmd;

            if (cmd == "newmtl") {
                flushCurrent();
                currentName.clear();
                currentMat = ParsedMaterial{};
                iss >> currentName;
            } else if (cmd == "Kd") {
                float r = 1.0f, g = 1.0f, b = 1.0f;
                iss >> r >> g >> b;
                currentMat.albedo = G::Color(r, g, b, 1.0f);
            } else if (cmd == "map_Kd") {
                std::string tex;
                std::getline(iss, tex);
                tex = trim(tex);
                if (!tex.empty()) {
                    currentMat.diffuseTexturePath = joinPath(baseDir, tex);
                }
            }
        }

        flushCurrent();
        return materials;
    }

    [[nodiscard]] G::Material3D buildMaterial(
        const ParsedMaterial* parsed) {
        G::Material3D material;
        material.albedo = parsed ? parsed->albedo : G::Color::white();
        material.opacity = 1.0f;
        material.unlit = true;

        if (parsed && !parsed->diffuseTexturePath.empty()) {
            material.diffuseMap = G::AllegroTexture2D::loadFromFile(parsed->diffuseTexturePath);
        }

        return material;
    }
}

namespace G {

    Model ObjLoader::load(const std::string& objPath) {
        std::ifstream file(objPath);
        if (!file) {
            throw std::runtime_error("Failed to open OBJ: " + objPath);
        }

        const std::string baseDir = directoryOf(objPath);

        std::vector<M::Point3D> positions;
        std::vector<M::Vector2D> texcoords;
        std::vector<M::Vector3D> normals;

        std::unordered_map<std::string, ParsedMaterial> materials;
        std::vector<ObjGroup> groups;

        std::shared_ptr<Mesh> currentMesh = std::make_shared<Mesh>();
        std::unordered_map<ObjIndex, int, ObjIndexHash> vertexMap;
        std::string currentMaterialName;

        auto flushGroup = [&]() {
            if (!currentMesh->vertices.empty() && !currentMesh->triangles.empty()) {
                ModelPart part;
                part.mesh = currentMesh;

                const auto it = materials.find(currentMaterialName);
                if (it != materials.end()) {
                    part.material = buildMaterial(&it->second);
                } else {
                    part.material = buildMaterial(nullptr);
                }

                groups.push_back(ObjGroup{currentMaterialName, currentMesh->triangles});
            }
        };

        auto pushCurrentPart = [&](Model& model) {
            if (!currentMesh->vertices.empty() && !currentMesh->triangles.empty()) {
                ModelPart part;
                part.mesh = currentMesh;

                const auto it = materials.find(currentMaterialName);
                if (it != materials.end()) {
                    part.material = buildMaterial(&it->second);
                } else {
                    part.material = buildMaterial(nullptr);
                }

                model.parts.push_back(std::move(part));
            }
        };

        auto newPart = [&]() {
            currentMesh = std::make_shared<Mesh>();
            vertexMap.clear();
        };

        auto getVertexIndex = [&](const ObjIndex& idx) -> int {
            const auto found = vertexMap.find(idx);
            if (found != vertexMap.end()) {
                return found->second;
            }

            const int pv = resolveObjIndex(idx.v, static_cast<int>(positions.size()));
            const int pt = (idx.vt != 0) ? resolveObjIndex(idx.vt, static_cast<int>(texcoords.size())) : -1;
            const int pn = (idx.vn != 0) ? resolveObjIndex(idx.vn, static_cast<int>(normals.size())) : -1;

            if (pv < 0 || pv >= static_cast<int>(positions.size())) {
                throw std::runtime_error("OBJ vertex index out of range in: " + objPath);
            }

            Vertex vtx;
            vtx.position = positions[static_cast<std::size_t>(pv)];
            vtx.uv = (pt >= 0 && pt < static_cast<int>(texcoords.size()))
                ? texcoords[static_cast<std::size_t>(pt)]
                : M::Vector2D(0.0f, 0.0f);
            vtx.normal = (pn >= 0 && pn < static_cast<int>(normals.size()))
                ? normals[static_cast<std::size_t>(pn)]
                : M::Vector3D(0.0f, 1.0f, 0.0f);

            const int outIndex = static_cast<int>(currentMesh->vertices.size());
            currentMesh->vertices.push_back(vtx);
            vertexMap[idx] = outIndex;
            return outIndex;
        };

        Model model;
        newPart();

        std::string line;
        while (std::getline(file, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::istringstream iss(line);
            std::string cmd;
            iss >> cmd;

            if (cmd == "v") {
                float x = 0.0f, y = 0.0f, z = 0.0f;
                iss >> x >> y >> z;
                positions.emplace_back(x, y, z);

            } else if (cmd == "vt") {
                float u = 0.0f, v = 0.0f;
                iss >> u >> v;
                texcoords.emplace_back(u, v);

            } else if (cmd == "vn") {
                float x = 0.0f, y = 0.0f, z = 0.0f;
                iss >> x >> y >> z;
                normals.emplace_back(x, y, z);

            } else if (cmd == "mtllib") {
                std::string mtlFile;
                iss >> mtlFile;
                if (!mtlFile.empty()) {
                    materials = loadMtl(joinPath(baseDir, mtlFile));
                }

            } else if (cmd == "usemtl") {
                // Start a new part when material changes.
                pushCurrentPart(model);
                newPart();

                iss >> currentMaterialName;

            } else if (cmd == "f") {
                std::vector<ObjIndex> face;
                std::string token;

                while (iss >> token) {
                    face.push_back(parseFaceIndex(token));
                }

                if (face.size() < 3) {
                    continue;
                }

                // Fan triangulation.
                for (std::size_t i = 1; i + 1 < face.size(); ++i) {
                    const int a = getVertexIndex(face[0]);
                    const int b = getVertexIndex(face[i]);
                    const int c = getVertexIndex(face[i + 1]);

                    currentMesh->triangles.push_back(G::Triangle{a, b, c});
                }
            }
        }

        pushCurrentPart(model);

        // If no normals were present, generate smooth normals per part.
        for (auto& part : model.parts) {
            bool hasUsefulNormals = false;
            for (const auto& v : part.mesh->vertices) {
                if (v.normal.magnitudeSqr() > M::EPS) {
                    hasUsefulNormals = true;
                    break;
                }
            }
            if (!hasUsefulNormals) {
                part.mesh->computeSmoothNormals();
            }
        }

        return model;
    }

} // namespace G