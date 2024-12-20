add_rules("mode.debug", "mode.release")

set_project("nav_benchmark")
set_targetdir("$(projectdir)/bin/benchmark")
set_objectdir("$(projectdir)/libs/benchmark")
set_languages("c++23", "c11")
set_toolchains("gcc")
set_toolset("cc", "gcc-14")
set_toolset("cxx", "g++-14")

add_requires("benchmark",{
    version = "1.9.0",
    configs = {
        shared = true,
        cppstd = "c++23"
    }
})

target("benchmark_io")
    set_kind("binary")
    add_files("benchmark_io.cpp")
    add_packages("benchmark")
    add_deps("nav_core","rtklib")
target_end()

target("benchmark_sv")
    set_kind("binary")
    add_files("benchmark_sv.cpp")
    add_packages("benchmark")
    add_deps("nav_core")
target_end()