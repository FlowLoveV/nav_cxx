add_rules("mode.debug", "mode.release")
set_xmakever("2.8.7")

-- global settings
set_targetdir("$(projectdir)/bin")
set_objectdir("$(projectdir)/libs")
set_languages("c++23","c11")
set_toolchains("clang")

-- required libraries


-- sub modules
includes("src/rtklib")
includes("src/core")

target("main")
    set_kind("binary")
    add_files("src/main.cpp")
    add_deps("rtklib")
    -- enable c++ 20 modules
    -- set_policy("build.c++.modules",true)


