add_library(Contrib STATIC
	fastlz/fastlz.c
	imgui/imgui.cpp
	imgui/imgui_demo.cpp
	imgui/imgui_draw.cpp
	imgui/imgui_tables.cpp
	imgui/imgui_widgets.cpp
	implot/implot.cpp
	implot/implot_demo.cpp
	implot/implot_items.cpp
	imgui/backends/imgui_impl_sdl3.cpp
	imgui/backends/imgui_impl_opengl2.cpp
)

set_target_properties(Contrib PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)

# Remove warnings-as-errors for third-party code
if(MSVC)
	target_compile_options(Contrib PRIVATE /W3 /WX-)
else()
	target_compile_options(Contrib PRIVATE -Wno-error -w)
endif()

# Public include directories so consumers can use these headers
target_include_directories(Contrib PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/fastlz
	${CMAKE_CURRENT_SOURCE_DIR}/imgui
	${CMAKE_CURRENT_SOURCE_DIR}/implot
	${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
)

# SDL3 includes needed for imgui_impl_sdl3.cpp
if(WIN32)
	target_include_directories(Contrib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/SDL/include)
elseif(APPLE)
	# SDL3_INCLUDE_DIR is set by the parent CMakeLists.txt
	target_include_directories(Contrib PRIVATE ${SDL3_INCLUDE_DIR})
else()
	# On Linux, link against SDL3 to get the include directories from the imported target
	target_link_libraries(Contrib PRIVATE SDL3::SDL3)
endif()
