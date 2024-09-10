add_rules("mode.debug")

includes("../third/xmake.lua")

target("nav_core")
    set_kind("shared")
    set_languages("c++23")
    add_cxxflags("-fpic")
    add_packages("spdlog","magic_enum","cpptrace",{public = true})
    add_includedirs("include",{public = true})
    add_includedirs("$(projectdir)/src/third",{public = true})
    add_deps("rtklib","ginan_core_shared", "reflect-cpp",{public = true})
    add_files("src/*.cpp")
    add_rpathdirs("$(projectdir)/bin")
target_end()





    
    