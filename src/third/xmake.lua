add_rules("mode.debug", "mode.release")

-- third
add_requires("toml++",{
    version = "3.4.0",
    configs = {
        shared = true,
        cppstd = "c++23",
    }
})
add_requires("spdlog",{
    version = "=1.14.1",
    configs = {
        shared = true,
        cppstd = "c++23",
        defines = { "SPDLOG_USE_STD_FORMAT" } -- Add macro definition to enable std::format
    }
})
add_requires("magic_enum", {
    version = "0.9.6",
    configs = {
        shared = true,
        cppstd = "c++23"
    }
})
add_requires("matplotplusplus", {
    version = "1.2.1",
    configs = {
        shared = true,
        cppstd = "c++23"
    }
})
add_requires("gflags",{
    version = "2.2.2",
    configs = {
        shared = true,
        cppstd = "c++23"
    }
})
add_requires("fast_float",{
    version = "6.1.1",
    configs = {
        shared = true,
        cppstd = "c++23"
    }
})
add_requires("proj",{
    -- version = "9.4.1",
    configs = {
        shared = true,
        cppstd = "c++23"
    }
})
add_requires("stringzilla",{
    version = "v3.9.2",
    configs = {
        shared = true,
        cppstd = "c++23",
    }
})
add_requires("cpptrace",{
    configs = {
        shared = true,
        cppstd = "c++23",
    }
})
add_requires("flatbuffers",{
    version = "v24.3.25",
    configs = {
        shared = true,
        cppstd = "c++23",
    }
})
add_requires("fmt",{
    version = "11.0.2",
    configs = {
        shared = true,
        cppstd = "c++23",
    }
})



-- Define a recursive function to traverse all directories and add them to includedirs
function recursive_add_includedirs(dir, is_public)
    for _, filepath in ipairs(os.dirs(path.join(dir, "*"))) do
        if os.isdir(filepath) then
            if is_public then
                add_includedirs(filepath, {public = true})
            else
                add_includedirs(filepath)
            end
            recursive_add_includedirs(filepath, is_public)
        end
    end
end

-- [install](http://ceres-solver.org/installation.html#getting-the-source-code)
-- [source](http://ceres-solver.org/ceres-solver-2.2.0.tar.gz)
-- platform Ubuntu 24.04
-- use `dpkg -L libsuitesparse-dev` to show where the apt library location is.
-- when you want to link ceres-solver,use add_deps_ceres(target)
function add_deps_ceres(target)
    -- apt library location 
    add_linkdirs("/usr/lib/x86_64-linux-gnu")
    -- ceres-solver library location
    add_linkdirs("/usr/local/lib")
    add_links("ceres")
    -- link deps
    -- use xmake-repo gflags instead of apt installed or self-built
    add_packages("gflags")
    add_links("glog")
    add_links("blas","lapack")
    add_links("cholmod", "amd", "colamd", "camd", "ccolamd", "suitesparseconfig")
end


-- reflect-cpp library,build from source directly
-- add_requires("reflect-cpp") don't work(2024-6-10)
target("reflect-cpp")
    set_kind("shared")
    set_languages("c++23")
    add_includedirs("reflect-cpp/include",{public = true})
    add_files("reflect-cpp/src/yyjson.c")
    add_packages("toml++",{public = true})
    add_packages("flatbuffers",{public = true})
target_end()


target("exprtk")
    set_kind("phony")
    add_includedirs("exprtk",{public = true})
    add_linkdirs("exprtk",{public = true})
target_end()

package("ginan")
    add_deps("cmake")
    local project_dir = path.join(os.scriptdir(),"ginan")
    set_sourcedir(project_dir)
    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:debug() and "Debug" or "Release"))
        table.insert(configs, "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- add_requires("ginan")

function add_ginan_header(target)
    local ginan_path = path.join(os.scriptdir(),"ginan/cpp/")
    recursive_add_includedirs(ginan_path,true)
end





