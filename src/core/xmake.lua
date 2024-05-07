add_rules("mode.debug", "mode.release")

-- requires fmt
add_requires("fmt",{
    version = ">=10.2.1",
    configs = {
        shared = true, 
        cppstd = "c++11" 
    }
})
-- requires spdlog
add_requires("spdlog",{
    version = ">=1.12.0",
    configs = {
        shared = true,
        cppstd = "c++20"
    }
})

target("nav_module")
    set_kind("shared")
    set_languages("c++23")
    set_policy("build.c++.modules",true)
    add_files("include/*.cppm")
    add_files("src/test.cpp")

target("nav_core")
    set_kind("shared")
    set_languages("c++23")
    -- some macro definations
    -- add_defines("ENMODULES", {public = true})
    add_defines("FORMATLIB=0",{public = true})
    -- packages
    add_packages("fmt", {public = true})
    add_packages("spdlog", {public = true})
    -- include directions
    add_includedirs("include", {public = true })
    -- source files
    add_files("src/nav.cpp")
    -- enable c++ 20 modules
    -- set_policy("build.c++.modules",true)

package("nav_module")
    set_sourcedir(path.join(os.scriptdir(), "src"))
    on_install(function(package)
        import("package.tools.xmake").install(package, {})
    end)




    
    