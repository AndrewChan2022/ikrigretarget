//
//  mylib.cpp
//
//
//  Created by kai chen on 3/24/23.
//

#include "ikrigretargetapi.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>


#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

using namespace SoulIK;


static inline std::string to_string(bool bValue) {
    return bValue ? "true" : "false";
}

PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<SoulIKRigRetargetConfig::SoulIKRigChain>);
PYBIND11_MAKE_OPAQUE(std::vector<SoulIKRigRetargetConfig::SoulIKRigChainMapping>);

PYBIND11_MODULE(ikrigretarget, m) {

    py::bind_vector<std::vector<int>>(m, "VectorInt");
    py::bind_vector<std::vector<SoulIKRigRetargetConfig::SoulIKRigChain>>(m, "VectorSoulIKRigChain");
    py::bind_vector<std::vector<SoulIKRigRetargetConfig::SoulIKRigChainMapping>>(m, "VectorSoulIKRigChainMapping");

    py::implicitly_convertible<py::list, std::vector<int>>();
    py::implicitly_convertible<py::list, std::vector<SoulIKRigRetargetConfig::SoulIKRigChain>>();
    py::implicitly_convertible<py::list, std::vector<SoulIKRigRetargetConfig::SoulIKRigChainMapping>>();

    py::enum_<CoordType>(m, "CoordType")
        .value("RightHandZupYfront", CoordType::RightHandZupYfront)
        .value("RightHandYupZfront", CoordType::RightHandYupZfront);
        //.export_values(); // to parent space

    py::enum_<ERootType>(m, "ERootType")
        .value("RootZ", ERootType::RootZ)
        .value("RootZMinusGroundZ", ERootType::RootZMinusGroundZ)
        .value("Ignore", ERootType::Ignore);

    py::class_<SoulIK::SoulIKRigRetargetConfig::SoulIKRigChain>(m, "SoulIKRigChain")
        .def(py::init())
        .def(py::init<std::string const&, std::string const&, std::string const&>())
        .def_readwrite("chainName", &SoulIKRigRetargetConfig::SoulIKRigChain::chainName)
        .def_readwrite("startBone", &SoulIKRigRetargetConfig::SoulIKRigChain::startBone)
        .def_readwrite("endBone", &SoulIKRigRetargetConfig::SoulIKRigChain::endBone)
        .def("__repr__", [](const SoulIKRigRetargetConfig::SoulIKRigChain& a) {
                return std::string("<SoulIKRigChain:") + "\n"
                + "chainName:" + a.chainName + "\n"
                + "startBone:" + a.startBone + "\n"
                + "endBone:" + a.endBone + "\n"
                + ">";
            }
        );

    py::class_<SoulIK::SoulIKRigRetargetConfig::SoulIKRigChainMapping>(m, "SoulIKRigChainMapping")
        .def(py::init())
        .def(py::init<bool, bool, std::string const&, std::string const&>())
        .def_readwrite("EnableFK", &SoulIKRigRetargetConfig::SoulIKRigChainMapping::EnableFK)
        .def_readwrite("EnableIK", &SoulIKRigRetargetConfig::SoulIKRigChainMapping::EnableIK)
        .def_readwrite("SourceChain", &SoulIKRigRetargetConfig::SoulIKRigChainMapping::SourceChain)
        .def_readwrite("TargetChain", &SoulIKRigRetargetConfig::SoulIKRigChainMapping::TargetChain)
        .def("__repr__", [](const SoulIKRigRetargetConfig::SoulIKRigChainMapping& a) {
                return std::string("<SoulIKRigChainMapping:") + "\n"
                + "EnableFK:" + to_string(a.EnableFK) + "\n"
                + "EnableIK:" + to_string(a.EnableIK) + "\n"
                + "SourceChain:" + a.SourceChain + "\n"
                + "TargetChain:" + a.TargetChain + "\n"
                + ">";
            }
        );

    py::class_<SoulIK::SoulIKRigRetargetConfig> config(m, "SoulIKRigRetargetConfig");
    config
        .def(py::init())
        .def_readwrite("SourceCoord", &SoulIKRigRetargetConfig::SourceCoord)
        .def_readwrite("WorkCoord", &SoulIKRigRetargetConfig::WorkCoord)
        .def_readwrite("TargetCoord", &SoulIKRigRetargetConfig::TargetCoord)

        .def_readwrite("TargetRootType", &SoulIKRigRetargetConfig::TargetRootType)
        .def_readwrite("TargetRootBone", &SoulIKRigRetargetConfig::TargetRootBone)
        .def_readwrite("TargetGroundBone", &SoulIKRigRetargetConfig::TargetGroundBone)

        .def_readwrite("SourceRootType", &SoulIKRigRetargetConfig::SourceRootType)
        .def_readwrite("SourceRootBone", &SoulIKRigRetargetConfig::SourceRootBone)
        .def_readwrite("SourceGroundBone", &SoulIKRigRetargetConfig::SourceGroundBone)

        .def_readwrite("SourceChains", &SoulIKRigRetargetConfig::SourceChains)
        .def_readwrite("TargetChains", &SoulIKRigRetargetConfig::TargetChains)
        .def_readwrite("ChainMapping", &SoulIKRigRetargetConfig::ChainMapping)

        .def_readwrite("IntArray", &SoulIKRigRetargetConfig::IntArray)

        .def("__repr__", &SoulIKRigRetargetConfig::to_string);

    m.doc() = R"pbdoc(
        ikrigretarget plugin
        -----------------------

        .. currentmodule:: ikrigretarget

        .. autosummary::
           :toctree: _generate

           ikrigretarget
    )pbdoc";

    m.def("myadd", &myadd, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    m.def("retargetFBX", &retargetFBX, R"pbdoc(
        retargetFBX

        srcAnimationFile,
        srcTPoseFile,
        srcRootJointName,
        targetFile,
        targetTPoseFile,
        outfile,
        config : SoulIK::SoulIKRigRetargetConfig

    )pbdoc");



#ifdef VERSION_INFO

#define v0 MACRO_STRINGIFY(VERSION_INFO)0
#if v11 == 0
    m.attr("__version__") = "dev";
#else
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#endif
#else
    m.attr("__version__") = "dev";
#endif
}

