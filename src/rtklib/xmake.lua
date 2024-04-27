add_rules("mode.debug", "mode.release")

target("rtklib")
    set_languages("c99")
    set_kind("shared")
    add_files("src/**.c", "src/rcv/**.c")
    add_includedirs("include", {public = true })
    -- enable all Gnss System
    -- enable trace
    add_defines("ENACMP", "ENAGLO", "ENAGAL", "TRACE", {public = false })
    if is_os("windows") then
        add_links("wsock32", "ws2_32", "winmm", {public = true })
        add_defines("WIN_DLL", "WIN32", {public = true })
    elseif is_os("linux") then 
        add_link("pthread", "m", {public = true })
    end
    set_optimize("fastest")
target_end()



