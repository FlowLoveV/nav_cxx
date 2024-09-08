add_rules("mode.debug", "mode.release")
set_xmakever("2.8.7")

-- global settings
set_project("nav_cxx")
set_targetdir("$(projectdir)/bin")
set_objectdir("$(projectdir)/libs")
set_languages("c++23", "c11")
set_toolchains("gcc")

-- option("cc")
--     set_default("gcc")
-- option_end()

-- if has_config("cc") then 
--     local cc = get_config("cc")
--     if cc == "clang" then   
--         set_toolchains("clang")
--         add_cxxflags("-stdlib=libc++")
--         add_ldflags("-lc++")
--     elseif cc == "gcc" then
--         set_toolchains("gcc")
--     elseif cc == "msvc" then 
--         if is_os("linux","macos") then 
--             raise("MSVC is not supported on this operating system")
--         end
--     end
-- end

-- sub modules
includes("src/rtklib/xmake.lua")
includes("src/core/xmake.lua")
includes("src/third/xmake.lua")
includes("src/plot/xmake.lua")
includes("src/third/ginan/cpp/xmake.lua")
-- test modules
includes("src/test/xmake.lua")

target("main")
    set_kind("binary")
    set_languages("c++23")
    add_cxxflags("-fpie")
    add_deps("nav_core",{public = true})
    add_deps("reflect-cpp")
    add_files("src/main.cpp")
target_end()



