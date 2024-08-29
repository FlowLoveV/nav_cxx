add_rules("mode.debug")

target("nav_core")
    set_kind("shared")
    set_languages("c++23")
    add_packages("spdlog","magic_enum",{public = true})
    add_includedirs("include",{public = true})
    add_deps("rtklib")
    add_files("src/*.cpp")
    add_rpathdirs("$(projectdir)/bin")
target_end()





    
    