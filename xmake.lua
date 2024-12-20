add_rules("mode.debug", "mode.release")
set_xmakever("2.8.7")

-- global settings
set_project("nav_cxx")
set_targetdir("$(projectdir)/bin")
set_objectdir("$(projectdir)/libs")
set_languages("c++23", "c11")
set_toolchains("gcc")
set_toolset("cc", "gcc-14")
set_toolset("cxx", "g++-14")

-- sub modules
includes("src/rtklib/xmake.lua")
includes("src/core/xmake.lua")
includes("src/third/xmake.lua")
includes("src/plot/xmake.lua")
includes("src/third/ginan/cpp/xmake.lua")
-- test modules
includes("src/test/xmake.lua")
-- benchmark modules
includes("src/benchmark/xmake.lua")

target("main")
    set_kind("binary")
    set_languages("c++23")
    add_cxxflags("-fpie")
    add_deps("nav_core")
    add_deps_ceres(target)
    add_files("src/main.cpp")
target_end()



 
