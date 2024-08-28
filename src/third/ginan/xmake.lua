set_project("ginan")
add_rules("mode.release")

set_targetdir("$(projectdir)/bin")
set_objectdir("$(projectdir)/libs")

-- settings 
local boost_dir = "/lib/x86_64-linux-gnu"

target("sofa_lib")
    set_kind("static") 
    add_includedirs("3rdparty/sofa/src")
    add_files("3rdparty/sofa/src/*.c")  
    add_cxflags("-fpic", {force = true})

target("ginan_core")
    set_kind("static") 
    set_optimize("fastest")
    add_defines("NDEBUG")
    set_languages("c++23")
    add_cxxflags("-O3")  
    add_cxxflags("-fpermissive")
    add_cxxflags("-Wall")
    add_cxxflags("-Wno-write-strings",{force = true})
    add_cxxflags("-Wno-deprecated-declarations")
    add_cxxflags("-Wno-format-overflow")
    add_cxxflags("-Wno-narrowing")
    add_cxxflags("-Wno-unused-but-set-variable")
    add_cxxflags("-Wno-sign-compare")
    add_cxxflags("-Wno-stringop-truncation")
    add_cxxflags("-Wno-unused-variable")
    add_cxxflags("-Wno-switch")
    add_cxxflags("-Wno-dangling-else")
    add_cxxflags("-Wno-misleading-indentation")
    add_cxxflags("-Wno-format-truncation")
    add_cxxflags("-Wno-extern-c-compat")
    add_cxxflags("-Wno-format-zero-length")
    add_cxxflags("-Wno-array-bounds")
    add_cxxflags("-Wno-restrict")
    add_cxxflags("-fpic")
    
    add_files("common/*.cpp",
              "3rdparty/jpl/*.cpp",
              "3rdparty/egm96/*.c",
              "3rdparty/slr/*.c",
              "3rdparty/iers2010/**/*.cpp",
              "pea/*.cpp",
              "iono/*.cpp",
              "inertial/*.cpp",
              "trop/*.cpp",
              "ambres/*.cpp",
              "slr/*.cpp",
              "other_ssr/*.cpp",
              "orbprop/*.cpp",
              "rtklib/*.cpp",
              "common/mongo*.cpp")
    add_includedirs("3rdparty",
                    "3rdparty/egm96",
                    "3rdparty/iers2010",
                    "3rdparty/sofa/src",
                    "3rdparty/jpl",
                    "3rdparty/mqtt_cpp/",
                    "3rdparty/slr",
                    "configurator",
                    "ambres",
                    "slr",
                    "common",
                    "iono",
                    "trop",
                    "inertial",
                    "orbprop",
                    "pea",
                    "rtklib",
                    "other_ssr",
                    "/usr/local/include/mongocxx/v_noabi",
                    "/usr/local/include/bsoncxx/v_noabi",
                    "/usr/local/include/libmongoc-1.0",
                    "/usr/local/include/libbson-1.0",
                    "/usr/local/lib",{public = true})
    recursive_add_includedirs("/usr/local/include/mongocxx",true)
    recursive_add_includedirs("/usr/local/include/bsoncxx",true)
    recursive_add_includedirs("/usr/local/include/libmongoc-1.0",true)
    recursive_add_includedirs("/usr/local/include/libbson-1.0",true)

    if has_config("openmp") then
        add_packages("openmp")
    end
    add_defines("EIGEN_USE_BLAS=1")
    add_linkdirs("/usr/local/lib",
                 "/usr/lib/x86_64-linux-gnu",
                 "/lib/x86_64-linux-gnu")
    add_deps("sofa_lib",{public = true})
    add_syslinks("m", "pthread", "dl","backtrace","crypto",{public = true})
    -- linking boost, linking static libraries
    add_syslinks(boost_dir .. "/libboost_log_setup.a",
                 boost_dir .. "/libboost_thread.a",
                 boost_dir .. "/libboost_log.a",
                 boost_dir .. "/libboost_program_options.a",
                 boost_dir .. "/libboost_serialization.a",
                 boost_dir .. "/libboost_filesystem.a",
                 boost_dir .. "/libboost_system.a",
                 boost_dir .. "/libboost_chrono.a",
                 {public = true})
    add_links("yaml-cpp", "mongocxx", "bsoncxx", "ssl", "lapack", "blas",{public = true})
target_end()

target("ginan")
    set_kind("binary")
    add_files("gui/ginan.cpp","gui/anode.cpp","gui/beast.cpp")
    add_includedirs("3rdparty/beast/")
    add_deps("ginan_core")
target_end()


