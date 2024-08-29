add_rules("mode.debug", "mode.release")

target("plot")
    set_kind("shared")
    set_languages("c++23")
    add_includedirs("include",{public = true})
    add_files("src/*.cpp")
    add_packages("magic_enum","matplotplusplus")
target_end()
