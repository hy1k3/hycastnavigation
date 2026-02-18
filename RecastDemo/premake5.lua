--
-- http://premake.github.io/
--

local buildDir = "Build/" .. _ACTION or ""

workspace "recastnavigation"
	location (buildDir)
	startproject "RecastDemo"

	configurations {
		"Debug",
		"Release"
	}

	architecture "x64"
	symbols "On"
	exceptionhandling "Off"
	rtti "Off"

	-- Use fast math operations.  This is not required, but it speeds up some calculations
	-- at the expense of accuracy.  Because there are some functions like dtMathIsfinite
	-- that use floating point functions that become undefined behavior when compiled with
	-- fast-math, we need to conditionally short-circuit these functions.
	floatingpoint "Fast"
	defines { "RC_FAST_MATH" }

	filter "configurations:Debug"
		defines { "DEBUG" }
		optimize "Off"
		targetdir ( buildDir .. "/lib/Debug" )

	filter "configurations:Release"
		defines { "RC_DISABLE_ASSERTS" }
		optimize "Speed"
		targetdir ( buildDir .. "/lib/Release" )

	filter "system:windows"
		defines {
			"WIN32",
			"_WINDOWS",
			"_HAS_EXCEPTIONS=0"
		}

	filter "system:not windows"
		warnings "Extra"
	
	filter {"system:linux", "toolset:gcc"}
		buildoptions {
			"-Wno-error=class-memaccess",
			"-Wno-error=maybe-uninitialized"
		}

local function libproject(name, dependencies)
	project(name)
		language "C++"
		cppdialect "C++98"
		kind "StaticLib"

		warnings "Extra"
		fatalwarnings { "All" }

		local includes = {
			"../" .. name .. "/Include"
		}
		for _,dependency in ipairs(dependencies) do
			table.insert(includes, "../" .. dependency .. "/Include")
		end
		includedirs(includes)

		files {
			"../" .. name .. "/Include/*.h",
			"../" .. name .. "/Source/*.cpp"
		}
end

libproject("Recast", {})
libproject("Detour", {})
libproject("DebugUtils", {"Detour", "Recast"})

project "Contrib"
	language "C++"
	cppdialect "C++20"
	kind "StaticLib"

	includedirs {
		"../RecastDemo/Contrib/imgui",
		"../RecastDemo/Contrib/implot",
		"../RecastDemo/Contrib/imgui/backends",
	}
	files {
		"../RecastDemo/Contrib/fastlz/*.c",
		"../RecastDemo/Contrib/imgui/*.cpp",
		"../RecastDemo/Contrib/implot/*.cpp",
		"../RecastDemo/Contrib/imgui/backends/imgui_impl_sdl3.cpp",
		"../RecastDemo/Contrib/imgui/backends/imgui_impl_opengl2.cpp",
	}

	filter "system:linux"
		buildoptions { "`pkg-config --cflags sdl3`", }

	filter "system:windows"
		includedirs { "../RecastDemo/Contrib/SDL/include" }

	filter "system:macosx"
		includedirs { "Bin/SDL3.framework/Headers" }
		externalincludedirs { "Bin/SDL3.framework/Headers" }
		frameworkdirs { "Bin" }

project "RecastDemo"
	language "C++"
	cppdialect "C++20" -- we don't care about this being compatible in the same way we do with the library code.
	kind "WindowedApp"
	targetdir "Bin"
	debugdir "Bin"

	includedirs {
		"../RecastDemo/Include",
		"../DebugUtils/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	externalincludedirs {
		"../RecastDemo/Contrib/fastlz",
		"../RecastDemo/Contrib/imgui",
		"../RecastDemo/Contrib/implot",
		"../RecastDemo/Contrib/imgui/backends",
	}

	files {
		"../RecastDemo/Include/*.h",
		"../RecastDemo/Source/*.cpp",
	}

	links {
		"DebugUtils",
		"Detour",
		"Recast",
		"Contrib"
	}

	filter "system:linux"
		buildoptions {
			"`pkg-config --cflags sdl3`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`"
		}
		linkoptions {
			"`pkg-config --libs sdl3`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`"
		}

	filter "system:windows"
		includedirs { "../RecastDemo/Contrib/SDL/include" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x64" }
		debugdir "../RecastDemo/Bin/"
		defines {
			"_CRT_SECURE_NO_WARNINGS",
		}
		links {
			"glu32",
			"opengl32",
			"SDL3",
		}
		postbuildcommands {
			-- Copy the SDL3 dll to the Bin folder.
			'{COPY} "%{path.getabsolute("Contrib/SDL/lib/x64/SDL3.dll")}" "%{cfg.targetdir}"'
		}

	filter "system:macosx"
		kind "ConsoleApp"
		includedirs { "Bin/SDL3.framework/Headers" }
		externalincludedirs { "Bin/SDL3.framework/Headers" }
		frameworkdirs { "Bin" }
		links {
			"OpenGL.framework",
			"SDL3.framework",
			"Cocoa.framework",
		}

project "Tests"
	language "C++"
	cppdialect "C++20" -- Catch requires newer C++ features
	kind "ConsoleApp"
	targetdir "Bin"
	debugdir "Bin"
	warnings "Default"
	fatalwarnings { }

	-- Catch requires RTTI and exceptions
	exceptionhandling "On"
	rtti "On"

	includedirs {
		"../DebugUtils/Include",
		"../Detour/Include",
		"../Recast/Include",
		"../Recast/Source",
		"../Tests/Recast",
		"../Tests",
		"../Tests/Contrib"
	}
	files {
		"../Tests/**.h",
		"../Tests/**.hpp",
		"../Tests/**.cpp"
	}

	links {
		"DebugUtils",
		"Detour",
		"Recast",
	}

	-- enable ubsan and asan when compiling with clang
	filter "toolset:clang"
		-- Disable `-Wnan-infinity-disabled` because Catch uses functions like std::isnan() that
		-- generate warnings when compiled with -ffast-math.
		buildoptions { "-Wno-nan-infinity-disabled" }
		buildoptions { "-fsanitize=undefined", "-fsanitize=address" } -- , "-fsanitize=memory" }
		linkoptions { "-fsanitize=undefined", "-fsanitize=address" } --, "-fsanitize=memory" }

	filter "system:linux"
		buildoptions {
			"-Wno-parentheses" -- Disable parentheses warning for the Tests target, as Catch's macros generate this everywhere.
		}
		linkoptions {
			"-lpthread"
		}

	filter "system:macosx"
		kind "ConsoleApp"
		links { "Cocoa.framework" }
