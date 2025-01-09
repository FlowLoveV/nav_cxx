add_rules("mode.debug", "mode.release")
set_xmakever("2.8.7")

-- global settings
set_project("nav_cxx")
if is_mode("debug") then
    set_targetdir("$(projectdir)/bin/debug")
else 
    set_targetdir("$(projectdir)/bin/release")
end
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



 
