add_rules("mode.debug", "mode.release")

target("nav_core")
    set_kind("shared")
    set_languages("c++23")
    add_includedirs("include", {public = true })
    add_files("src/*.cpp")
    
    