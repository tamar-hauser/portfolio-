#include "WheelchairBinding.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <iostream>
#include <cstdlib>
#ifdef _WIN32
# include <errno.h>
# include <windows.h>
#endif
namespace py = pybind11;

// ─────────────────────────────────────────────
// ─────────────────────────────────────────────

static SurfaceType surfaceFromString(const std::string& s)
{
    if (s == "asphalt")       return SurfaceType::ASPHALT;
    if (s == "concrete")      return SurfaceType::CONCRETE;
    if (s == "paving_stones") return SurfaceType::PAVING_STONES;
    if (s == "cobblestone")   return SurfaceType::COBBLESTONE;
    if (s == "gravel")        return SurfaceType::GRAVEL;
    if (s == "grass")         return SurfaceType::GRASS;
    if (s == "sand")          return SurfaceType::SAND;
    return SurfaceType::UNKNOWN;
}

// ─────────────────────────────────────────────
// ─────────────────────────────────────────────

static std::unordered_map<uint64_t, Node>
graphFromPyDict(const py::dict& py_nodes, const py::list& py_edges)
{
    std::unordered_map<uint64_t, Node> graph;
    graph.reserve(py::len(py_nodes));

    for (auto& [key, val] : py_nodes) {
        py::dict nd = val.cast<py::dict>();

        Node node;
        node.lat               = nd["lat"].cast<double>();
        node.lon               = nd["lon"].cast<double>();
        node.kerb_height       = nd["kerb_height"].cast<double>();
        node.is_crossing       = nd["is_crossing"].cast<bool>();
        node.has_traffic_light = nd["has_traffic_light"].cast<bool>();

        graph[key.cast<uint64_t>()] = std::move(node);
    }

    for (auto& item : py_edges) {
        py::dict ed = item.cast<py::dict>();

        uint64_t src = ed["source"].cast<uint64_t>();

        auto it = graph.find(src);
        if (it == graph.end()) {
            std::cerr << "[wheelchair_binding] אזהרה: קשת ממקור לא מוכר "
                      << src << " – מדולגת\n";
            continue;
        }

        Edge edge;
        edge.target      = ed["target"].cast<uint64_t>();
        edge.effort_cost = ed["effort_cost"].cast<double>();
        edge.length      = ed["length"].cast<double>();
        edge.width       = ed["width"].cast<double>();
        edge.incline     = ed["incline"].cast<double>();
        edge.surface     = surfaceFromString(ed["surface"].cast<std::string>());
        edge.safe_speed  = ed["safe_speed"].cast<double>();

        it->second.edges.push_back(std::move(edge));
    }

    return graph;
}

// ─────────────────────────────────────────────
// ─────────────────────────────────────────────

GraphResult buildGraphFromPython(double orig_lat, double orig_lon,
                                 double dest_lat, double dest_lon,
                                 const std::string& config_path)
{
            if (!Py_IsInitialized()) {
        #ifdef PYTHON_HOME_DIR
            std::string pyhome = std::string(PYTHON_HOME_DIR);
        #ifdef _WIN32
            // Normalize forward slashes → backslashes. CMake provides forward-slash
            // paths; mixing / and \ in Py_SetPath confuses Python's FileFinder on
            // Windows (FileFinder.scandir fails silently), so _socket.pyd is not found.
            std::replace(pyhome.begin(), pyhome.end(), '/', '\\');
            static std::wstring sPyhome(pyhome.begin(), pyhome.end());
            static std::wstring sPypath =
                sPyhome + L"\\Lib;"             +
                sPyhome + L"\\DLLs;"            +
                sPyhome + L"\\Lib\\site-packages";
            // python312.dll lives in sPyhome (not in DLLs); add both so that
            // extension modules can resolve their DLL dependencies.
            AddDllDirectory(sPyhome.c_str());
            AddDllDirectory((sPyhome + L"\\DLLs").c_str());
            Py_SetPythonHome(sPyhome.c_str());
            Py_SetPath(sPypath.c_str());
        #else
            std::string pylib = pyhome + "/Lib:" + pyhome + "/DLLs:" + pyhome + "/Lib/site-packages";
            setenv("PYTHONHOME", pyhome.c_str(), 1);
            setenv("PYTHONPATH", pylib.c_str(), 1);
        #endif
        #endif
            static py::scoped_interpreter interpreter;
            }
    py::gil_scoped_acquire gil;

    try {
        std::string maplivePath = std::string(PROJECT_SOURCE_DIR) + "/MapLive";
        std::string configFullPath = maplivePath + "/" + config_path;

        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("insert")(0, maplivePath);

        py::module_ router = py::module_::import("GraphMap");
        py::object  build  = router.attr("build_graph_for_cpp");

        py::dict result = build(
            py::make_tuple(orig_lat, orig_lon),
            py::make_tuple(dest_lat, dest_lon),
            configFullPath
        ).cast<py::dict>();

        auto graph = graphFromPyDict(
            result["nodes"].cast<py::dict>(),
            result["edges"].cast<py::list>()
        );

        uint64_t source_node = result["source_node"].cast<uint64_t>();
        uint64_t target_node = result["target_node"].cast<uint64_t>();

        std::cout << "[wheelchair_binding] גרף נטען: "
                  << graph.size() << " צמתים | "
                  << "מוצא=" << source_node
                  << " יעד="  << target_node << "\n";

        return { std::move(graph), source_node, target_node };

    } catch (const py::error_already_set& e) {
        std::cerr << "[buildGraphFromPython] Python error: " << e.what() << "\n";
        return {};
    } catch (const std::exception& e) {
        std::cerr << "[buildGraphFromPython] error: " << e.what() << "\n";
        return {};
    }
}
