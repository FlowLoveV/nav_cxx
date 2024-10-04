add_rules("mode.debug")

includes("../third/xmake.lua")

target("nav_core")
    set_kind("shared")
    set_languages("c++23")
    add_cxxflags("-fpic")
    add_packages("spdlog","magic_enum","cpptrace",{public = true})    
    add_includedirs("include",{public = true})
    add_deps("reflect-cpp",{public = true})
    add_files("src/ginan/*.cpp")
    add_files("src/utils/*.cpp")
    add_files("src/gnss/*.cpp")
    add_files("src/io/*.cpp")
    add_files("src/solution/*.cpp")
    add_rpathdirs("$(projectdir)/bin")
    add_defines("BOOST_ALL_DYN_LINK")
    -- link boost
    local boost_dir = "/usr/local/boost_1_83_0/lib"
    add_links(boost_dir .. "/libboost_program_options.so",{public = true})
target_end()

target("spp")
    set_kind("binary")
    set_languages("c++23")
    add_cxxflags("-fpie")
    add_deps("nav_core")
    add_files("solver/spp.cpp")
target_end()





    
    