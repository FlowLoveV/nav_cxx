set_project("ginan")
add_rules("mode.debug","mode.release")
set_languages("c++23", "c11")
set_toolchains("gcc")
set_targetdir("$(projectdir)/bin")
set_objectdir("$(projectdir)/libs")

if is_plat("macosx") and is_cc("clang") then
    print("Using Clang complient compiler flags")
    add_cxxflags("-Wno-shift-overflow")
    add_cxxflags("-Wno-string-concatenation")
else
    add_cxxflags("-Wno-format-overflow")
    add_cxxflags("-Wno-stringop-truncation")
    add_cxxflags("-Wno-format-truncation")
    add_cxxflags("-Wno-restrict")
end

add_cxxflags("-std=c++2a")
add_cxxflags("-fpermissive")
add_cxxflags("-fno-omit-frame-pointer")
add_cxxflags("-Wall")
add_cxxflags("-Wno-write-strings")
add_cxxflags("-Wno-deprecated-declarations")
add_cxxflags("-Wno-narrowing")
add_cxxflags("-Wno-unused-but-set-variable")
add_cxxflags("-Wno-sign-compare")
add_cxxflags("-Wno-unused-variable")
add_cxxflags("-Wno-switch")
add_cxxflags("-Wno-dangling-else")
add_cxxflags("-Wno-misleading-indentation")
add_cxxflags("-Wno-extern-c-compat")
add_cxxflags("-Wno-format-zero-length")
add_cxxflags("-Wno-array-bounds")

if not is_mode("debug") then
    set_values("mode", "release")
end

option("build_doc")
    set_default(false)
    set_showmenu(true)
    set_description("BUILD_DOCUMENTATION")

if is_mode("debug") then
    option("enable_parallelisation")
        set_default(false)
        set_showmenu(true)
        set_description("ENABLE_PARALLELISATION")

    option("enable_optimisation")
        set_default(false)
        set_showmenu(true)
        set_description("ENABLE_OPTIMISATION")
else
    option("enable_parallelisation")
        set_default(true)
        set_showmenu(true)
        set_description("ENABLE_PARALLELISATION")

    option("enable_optimisation")
        set_default(true)
        set_showmenu(true)
        set_description("ENABLE_OPTIMISATION")
end

if has_config("enable_parallelisation") then
    add_cxxflags("-DENABLE_PARALLELISATION")
end

if has_config("enable_optimisation") then
    add_cxxflags("-DENABLE_OPTIMISATION")
end

target("sofa_lib")
    set_kind("static") 
    add_includedirs("3rdparty/sofa/src")
    add_files("3rdparty/sofa/src/*.c")  
    add_cxflags("-fpic", {force = true})
target_end()

target("ginan_core_shared")
    set_kind("shared") 
    set_optimize("fastest")
    add_cxxflags("-O3")
    add_cxxflags("-fpic")
    add_files(
          "../Architecture/Ginan.cpp",

          "common/acsConfig.cpp",
          "common/acsConfigDocs.cpp",

          "3rdparty/jpl/jpl_eph.cpp",
          "3rdparty/egm96/EGM96.c",
          "3rdparty/slr/read_crd.c",

          "3rdparty/iers2010/ch9/fcul_a.cpp",
          "3rdparty/iers2010/ch9/fcul_zd_hpa.cpp",
          "3rdparty/iers2010/hardisp/admint.cpp",
          "3rdparty/iers2010/hardisp/eval.cpp",
          "3rdparty/iers2010/hardisp/hardisp_impl.cpp",
          "3rdparty/iers2010/hardisp/recurs.cpp",
          "3rdparty/iers2010/hardisp/shells.cpp",
          "3rdparty/iers2010/hardisp/spline.cpp",
          "3rdparty/iers2010/hardisp/tdfrph.cpp",
          "3rdparty/iers2010/dehanttideinel/dehanttide_all.cpp",

          "pea/main.cpp",
          "pea/inputs.cpp",
          "pea/outputs.cpp",
          "pea/minimumConstraints.cpp",
          "pea/peaCommitStrings.cpp",
          "pea/pea_snx.cpp",
          "pea/ppp.cpp",
          "pea/ppppp.cpp",
          "pea/ppp_obs.cpp",
          "pea/ppp_ambres.cpp",
          "pea/ppp_callbacks.cpp",
          "pea/ppp_pseudoobs.cpp",
          "pea/ppp_slr.cpp",
          "pea/preprocessor.cpp",
          "pea/spp.cpp",

          "common/api.cpp",
          "common/ntripBroadcast.cpp",
          "common/acsQC.cpp",
          "common/algebra.cpp",
          "common/algebra_old.cpp",
          "common/algebraTrace.cpp",
          "common/attitude.cpp",
          "common/compare.cpp",
          "common/antenna.cpp",
          "common/biases.cpp",
          "common/biasSINEXread.cpp",
          "common/biasSINEXwrite.cpp",
          "common/common.cpp",
          "common/constants.cpp",
          "common/customDecoder.cpp",
          "common/cost.cpp",
          "common/debug.cpp",
          "common/ephemeris.cpp",
          "common/ephBroadcast.cpp",
          "common/ephKalman.cpp",
          "common/ephPrecise.cpp",
          "common/ephSSR.cpp",
          "common/erp.cpp",
          "common/fileLog.cpp",
          "common/gpx.cpp",
          "common/pos.cpp",
          "common/gTime.cpp",
          "common/interactiveTerminal.cpp",
          "common/ionModels.cpp",
          "common/linearCombo.cpp",
          "common/mongo.cpp",
          "common/mongoRead.cpp",
          "common/mongoWrite.cpp",

          "common/ntripTrace.cpp",
          "common/orbits.cpp",
          "common/receiver.cpp",
          "common/rinex.cpp",
          "common/rtsSmoothing.cpp",
          "common/rtcmDecoder.cpp",
          "common/rtcmEncoder.cpp",
          "common/rtcmTrace.cpp",
          "common/summary.cpp",
          "common/satSys.cpp",
          "common/sinex.cpp",
          "common/sinexParser.cpp",
          "common/tropSinex.cpp",
          "common/sp3.cpp",
          "common/sp3Write.cpp",
          "common/orbex.cpp",
          "common/orbexWrite.cpp",
          "common/tcpSocket.cpp",
          "common/trace.cpp",
          "common/testUtils.cpp",
          "common/rinexClkWrite.cpp",
          "common/rinexNavWrite.cpp",
          "common/rinexObsWrite.cpp",
          "common/tides.cpp",
          "common/ubxDecoder.cpp",
          "common/walkthrough.cpp",
          "common/localAtmosRegion.cpp",

          "common/streamNtrip.cpp",
          "common/streamCustom.cpp",
          "common/streamSerial.cpp",
          "common/streamUbx.cpp",
          "common/streamParser.cpp",

          "iono/geomagField.cpp",
          "iono/ionex.cpp",
          "iono/ionoMeas.cpp",
          "iono/ionoModel.cpp",
          "iono/ionoSpherical.cpp",
          "iono/ionoSphericalCaps.cpp",
          "iono/ionoBSplines.cpp",
          "iono/ionexWrite.cpp",
          "iono/ionoLocalSTEC.cpp",

          "inertial/posProp.cpp",

          "trop/tropModels.cpp",
          "trop/tropSAAS.cpp",
          "trop/tropSBAS.cpp",
          "trop/tropGPT2.cpp",
          "trop/tropVMF3.cpp",
          "trop/tropCSSR.cpp",

          "ambres/GNSSambres.cpp",

          "slr/slrCom.cpp",
          "slr/slrObs.cpp",
          "slr/slrSat.cpp",
          "slr/slrRec.cpp",

          "other_ssr/prototypeIgsSSRDecode.cpp",
          "other_ssr/prototypeCmpSSREncode.cpp",
          "other_ssr/prototypeCmpSSRDecode.cpp",
          "other_ssr/prototypeIgsSSREncode.cpp",

          "orbprop/aod.cpp",
          "orbprop/boxwing.cpp",
          "orbprop/acceleration.cpp",
          "orbprop/coordinates.cpp",
          "orbprop/iers2010.cpp",
          "orbprop/planets.cpp",
          "orbprop/tideCoeff.cpp",
          "orbprop/orbitProp.cpp",
          "orbprop/staticField.cpp",
          "orbprop/centerMassCorrections.cpp",
          "orbprop/oceanPoleTide.cpp",

          "rtklib/lambda.cpp",
          "rtklib/rtkcmn.cpp",

          "sbas/sisnet.cpp",
          "sbas/sbas.cpp"
    )
    add_includedirs("3rdparty",
                    "3rdparty/egm96",
                    "3rdparty/iers2010",
                    "3rdparty/sofa/src",
                    "3rdparty/jpl",
                    "3rdparty/slr",
                    "configurator",
                    "../Architecture",
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
                    "loading",
                    "sbas",
                    {public = true}
    )
    recursive_add_includedirs("/usr/local/include/mongocxx",true)
    recursive_add_includedirs("/usr/local/include/bsoncxx",true)
    recursive_add_includedirs("/usr/local/include/libmongoc-1.0",true)
    recursive_add_includedirs("/usr/local/include/libbson-1.0",true)

    if has_config("openmp") then
        add_packages("openmp")
    end
    if has_config("ENABLE_PARALLELISATION") then
        add_defines("ENABLE_PARALLELISATION=1")
    end
    add_defines("EIGEN_USE_BLAS=1")
    add_deps("sofa_lib",{public = true})
    add_syslinks("m", "pthread", "dl","crypto","curses", {public = true})
    -- linking boost, linking static libraries
    -- add_syslinks(
    --     "boost_log_setup",
    --     "boost_log",
    --     "boost_thread",
    --     "boost_date_time",
    --     "boost_system",
    --     "boost_program_options",
    --     "boost_serialization",
    --     "boost_timer",
    --     "boost_stacktrace_addr2line",
    --     {public = true}
    -- )
    add_defines("BOOST_ALL_DYN_LINK")
    local boost_dir = "/usr/local/boost_1_83_0/lib"
    add_links(boost_dir .. "/libboost_log_setup.so",
                 boost_dir .. "/libboost_thread.so",
                 boost_dir .. "/libboost_log.so",
                 boost_dir .. "/libboost_date_time.so",
                 boost_dir .. "/libboost_system.so",
                 boost_dir .. "/libboost_program_options.so",
                 boost_dir .. "/libboost_serialization.so",
                 boost_dir .. "/libboost_timer.so",
                 boost_dir .. "/libboost_stacktrace_addr2line.so",
                 {public = true}
    )
    add_links("yaml-cpp", "mongocxx", "bsoncxx", "ssl", "lapack", "blas",{public = true})
target_end()



target("ginan_core_static")
    set_kind("static") 
    set_optimize("fastest")
    add_cxxflags("-O3")
    add_cxxflags("-fpic")
    add_files(
          "../Architecture/Ginan.cpp",

          "common/acsConfig.cpp",
          "common/acsConfigDocs.cpp",

          "3rdparty/jpl/jpl_eph.cpp",
          "3rdparty/egm96/EGM96.c",
          "3rdparty/slr/read_crd.c",

          "3rdparty/iers2010/ch9/fcul_a.cpp",
          "3rdparty/iers2010/ch9/fcul_zd_hpa.cpp",
          "3rdparty/iers2010/hardisp/admint.cpp",
          "3rdparty/iers2010/hardisp/eval.cpp",
          "3rdparty/iers2010/hardisp/hardisp_impl.cpp",
          "3rdparty/iers2010/hardisp/recurs.cpp",
          "3rdparty/iers2010/hardisp/shells.cpp",
          "3rdparty/iers2010/hardisp/spline.cpp",
          "3rdparty/iers2010/hardisp/tdfrph.cpp",
          "3rdparty/iers2010/dehanttideinel/dehanttide_all.cpp",

          "pea/main.cpp",
          "pea/inputs.cpp",
          "pea/outputs.cpp",
          "pea/minimumConstraints.cpp",
          "pea/peaCommitStrings.cpp",
          "pea/pea_snx.cpp",
          "pea/ppp.cpp",
          "pea/ppppp.cpp",
          "pea/ppp_obs.cpp",
          "pea/ppp_ambres.cpp",
          "pea/ppp_callbacks.cpp",
          "pea/ppp_pseudoobs.cpp",
          "pea/ppp_slr.cpp",
          "pea/preprocessor.cpp",
          "pea/spp.cpp",

          "common/api.cpp",
          "common/ntripBroadcast.cpp",
          "common/acsQC.cpp",
          "common/algebra.cpp",
          "common/algebra_old.cpp",
          "common/algebraTrace.cpp",
          "common/attitude.cpp",
          "common/compare.cpp",
          "common/antenna.cpp",
          "common/biases.cpp",
          "common/biasSINEXread.cpp",
          "common/biasSINEXwrite.cpp",
          "common/common.cpp",
          "common/constants.cpp",
          "common/customDecoder.cpp",
          "common/cost.cpp",
          "common/debug.cpp",
          "common/ephemeris.cpp",
          "common/ephBroadcast.cpp",
          "common/ephKalman.cpp",
          "common/ephPrecise.cpp",
          "common/ephSSR.cpp",
          "common/erp.cpp",
          "common/fileLog.cpp",
          "common/gpx.cpp",
          "common/pos.cpp",
          "common/gTime.cpp",
          "common/interactiveTerminal.cpp",
          "common/ionModels.cpp",
          "common/linearCombo.cpp",
          "common/mongo.cpp",
          "common/mongoRead.cpp",
          "common/mongoWrite.cpp",

          "common/ntripTrace.cpp",
          "common/orbits.cpp",
          "common/receiver.cpp",
          "common/rinex.cpp",
          "common/rtsSmoothing.cpp",
          "common/rtcmDecoder.cpp",
          "common/rtcmEncoder.cpp",
          "common/rtcmTrace.cpp",
          "common/summary.cpp",
          "common/satSys.cpp",
          "common/sinex.cpp",
          "common/sinexParser.cpp",
          "common/tropSinex.cpp",
          "common/sp3.cpp",
          "common/sp3Write.cpp",
          "common/orbex.cpp",
          "common/orbexWrite.cpp",
          "common/tcpSocket.cpp",
          "common/trace.cpp",
          "common/testUtils.cpp",
          "common/rinexClkWrite.cpp",
          "common/rinexNavWrite.cpp",
          "common/rinexObsWrite.cpp",
          "common/tides.cpp",
          "common/ubxDecoder.cpp",
          "common/walkthrough.cpp",
          "common/localAtmosRegion.cpp",

          "common/streamNtrip.cpp",
          "common/streamCustom.cpp",
          "common/streamSerial.cpp",
          "common/streamUbx.cpp",
          "common/streamParser.cpp",

          "iono/geomagField.cpp",
          "iono/ionex.cpp",
          "iono/ionoMeas.cpp",
          "iono/ionoModel.cpp",
          "iono/ionoSpherical.cpp",
          "iono/ionoSphericalCaps.cpp",
          "iono/ionoBSplines.cpp",
          "iono/ionexWrite.cpp",
          "iono/ionoLocalSTEC.cpp",

          "inertial/posProp.cpp",

          "trop/tropModels.cpp",
          "trop/tropSAAS.cpp",
          "trop/tropSBAS.cpp",
          "trop/tropGPT2.cpp",
          "trop/tropVMF3.cpp",
          "trop/tropCSSR.cpp",

          "ambres/GNSSambres.cpp",

          "slr/slrCom.cpp",
          "slr/slrObs.cpp",
          "slr/slrSat.cpp",
          "slr/slrRec.cpp",

          "other_ssr/prototypeIgsSSRDecode.cpp",
          "other_ssr/prototypeCmpSSREncode.cpp",
          "other_ssr/prototypeCmpSSRDecode.cpp",
          "other_ssr/prototypeIgsSSREncode.cpp",

          "orbprop/aod.cpp",
          "orbprop/boxwing.cpp",
          "orbprop/acceleration.cpp",
          "orbprop/coordinates.cpp",
          "orbprop/iers2010.cpp",
          "orbprop/planets.cpp",
          "orbprop/tideCoeff.cpp",
          "orbprop/orbitProp.cpp",
          "orbprop/staticField.cpp",
          "orbprop/centerMassCorrections.cpp",
          "orbprop/oceanPoleTide.cpp",

          "rtklib/lambda.cpp",
          "rtklib/rtkcmn.cpp",

          "sbas/sisnet.cpp",
          "sbas/sbas.cpp"
    )
    add_includedirs("3rdparty",
                    "3rdparty/egm96",
                    "3rdparty/iers2010",
                    "3rdparty/sofa/src",
                    "3rdparty/jpl",
                    "3rdparty/slr",
                    "configurator",
                    "../Architecture",
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
                    "loading",
                    "sbas",
                    {public = true}
    )
    recursive_add_includedirs("/usr/local/include/mongocxx",true)
    recursive_add_includedirs("/usr/local/include/bsoncxx",true)
    recursive_add_includedirs("/usr/local/include/libmongoc-1.0",true)
    recursive_add_includedirs("/usr/local/include/libbson-1.0",true)

    if has_config("openmp") then
        add_packages("openmp")
    end
    if has_config("ENABLE_PARALLELISATION") then
        add_defines("ENABLE_PARALLELISATION=1")
    end
    add_defines("EIGEN_USE_BLAS=1")
    add_deps("sofa_lib",{public = true})
    add_syslinks("m", "pthread", "dl","crypto","curses", {public = true})
    -- linking boost, linking static libraries
    -- add_syslinks(
    --     "boost_log_setup",
    --     "boost_log",
    --     "boost_thread",
    --     "boost_date_time",
    --     "boost_system",
    --     "boost_program_options",
    --     "boost_serialization",
    --     "boost_timer",
    --     "boost_stacktrace_addr2line",
    --     {public = true}
    -- )
    local boost_dir = "/usr/local/boost_1_83_0/lib"
    add_links(boost_dir .. "/libboost_log_setup.a",
                 boost_dir .. "/libboost_thread.a",
                 boost_dir .. "/libboost_log.a",
                 boost_dir .. "/libboost_date_time.a",
                 boost_dir .. "/libboost_system.a",
                 boost_dir .. "/libboost_program_options.a",
                 boost_dir .. "/libboost_serialization.a",
                 boost_dir .. "/libboost_timer.a",
                 boost_dir .. "/libboost_stacktrace_addr2line.a",
                 {public = true}
    )
    add_links("yaml-cpp", "mongocxx", "bsoncxx", "ssl", "lapack", "blas",{public = true})
target_end()


