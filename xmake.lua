add_rules("mode.debug", "mode.release")
set_xmakever("2.8.7")

-- global settings
set_targetdir("$(projectdir)/bin")
set_objectdir("$(projectdir)/libs")
set_languages("c++23","c11")
set_toolchains("clang")

-- required libraries
-- add_repositories("my-repo my-repo")
-- add_requires("nav_module")

-- sub modules
includes("src/rtklib/xmake.lua")
includes("src/core/xmake.lua")

target("main")
    set_kind("binary")
    set_languages("c++23")
    add_files("src/main.cpp")
    add_deps("rtklib")
    add_deps("nav_core")
    -- add_packages("nav_module")
    -- set_policy("build.c++.modules",true)



