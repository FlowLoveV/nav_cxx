add_rules("mode.debug")

includes("../third/xmake.lua")

target("nav_core")
    set_kind("shared")
    set_languages("c++23")
    add_cxxflags("-fpic")
    -- third pch
    set_pcxxheader("$(projectdir)/src/third/ginan/cpp/common/navigation.hpp")
    set_pcxxheader("$(projectdir)/src/third/ginan/cpp/common/observations.hpp")
    set_pcxxheader("$(projectdir)/src/third/ginan/cpp/common/receiver.hpp")
    set_pcxxheader("$(projectdir)/src/third/ginan/cpp/common/rinex.hpp")
    -- self pch
    set_pcxxheader("$(projectdir)/src/core/include/utils/types.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/time.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/space.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/template_utils.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/option.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/result.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/logger.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/macro.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/angle.hpp")
    set_pcxxheader("$(projectdir)/src/core/include/utils/attitude.hpp")

    add_packages("spdlog","magic_enum","cpptrace",{public = true})    
    add_includedirs("include",{public = true})
    add_includedirs("$(projectdir)/src/third",{public = true})
    add_deps("ginan_core_shared", "reflect-cpp",{public = true})
    add_files("src/*.cpp")
    add_rpathdirs("$(projectdir)/bin")
target_end()





    
    