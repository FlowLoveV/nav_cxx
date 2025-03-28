add_rules("mode.debug","mode.release")

includes("../third/xmake.lua")



target("nav_core")
    set_kind("shared")

    on_load(function (target)
        if is_plat("linux", "macos") then
            target:add("cxxflags","-fvisibility=hidden")
        end
        if is_mode("debug") then 
            target:add("defines","GNSS_DEBUG")
        end 
    end)

    set_languages("c++23")
    add_cxxflags("-fpic")
    add_defines("SPDLOG_USE_STD_FORMAT",{public = true})
    add_defines("NAVP_LIBRARY")
    add_packages("spdlog","magic_enum","cpptrace",{public = true})    
    add_includedirs("include",{public = true})
    add_deps("reflect-cpp",{public = true})
    add_deps("exprtk")
    add_deps_ceres(target)
    add_files("src/ginan/*.cpp")
    add_files("src/utils/*.cpp")
    add_files("src/gnss/*.cpp")
    add_files("src/io/*.cpp")
    add_files("src/solution/*.cpp")
    add_files("src/algorithm/*.cpp")
target_end()

target("spp")
    set_kind("binary")
    set_languages("c++23")
    add_cxxflags("-fpie")
    add_deps("nav_core")
    add_files("solver/spp.cpp")
target_end()

target("rtk")
    set_kind("binary")
    set_languages("c++23")
    add_cxxflags("-fpie")
    add_deps("nav_core")
    add_files("solver/rtk.cpp")
target_end()




    
    