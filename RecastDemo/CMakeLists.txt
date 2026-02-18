# Find dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)

# SDL3 setup - platform specific
if(WIN32)
	if(NOT SDL3_ROOT_DIR)
		set(SDL3_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
	endif()
	find_package(SDL3 REQUIRED)
elseif(APPLE)
	if(NOT SDL3_ROOT_DIR)
		set(SDL3_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Bin")
	endif()
	set(SDL3_FRAMEWORK_PATH "${SDL3_ROOT_DIR}/SDL3.framework")
	# Use the framework's Headers directory for <SDL.h> style includes
	set(SDL3_INCLUDE_DIR "${SDL3_FRAMEWORK_PATH}/Headers")
else()
	# Linux - use system SDL3
	find_package(SDL3 REQUIRED)
endif()

# Build the third-party Contrib library (must come after SDL3 setup since it uses SDL3 variables)
add_subdirectory(Contrib)

# RecastDemo executable
if(WIN32)
	add_executable(RecastDemo WIN32)
elseif(APPLE)
	add_executable(RecastDemo MACOSX_BUNDLE)
else()
	add_executable(RecastDemo)
endif()

target_sources(RecastDemo PRIVATE
	Source/AppState.cpp
	Source/InputGeom.cpp
	Source/main.cpp
	Source/PartitionedMesh.cpp
	Source/PerfTimer.cpp
	Source/Sample.cpp
	Source/Sample_SoloMesh.cpp
	Source/Sample_TempObstacles.cpp
	Source/Sample_TileMesh.cpp
	Source/SampleInterfaces.cpp
	Source/TestCase.cpp
	Source/Tool_ConvexVolume.cpp
	Source/Tool_NavMeshPrune.cpp
	Source/Tool_NavMeshTester.cpp
	Source/Tool_OffMeshConnection.cpp
	Source/ValueHistory.cpp
)

target_include_directories(RecastDemo PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/Include
)

set_target_properties(RecastDemo PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)

# Platform-specific compiler options
if(APPLE)
	# Suppress deprecated OpenGL warnings (gluPerspective, gluProject, etc.)
	target_compile_options(RecastDemo PRIVATE -Wno-deprecated-declarations)
endif()

# Link libraries
target_link_libraries(RecastDemo PRIVATE
	OpenGL::GL
	OpenGL::GLU
	Contrib
	RecastNavigation::DebugUtils
	RecastNavigation::Detour
	RecastNavigation::DetourTileCache
	RecastNavigation::Recast
)

# SDL3 linking - platform specific
if(APPLE)
	target_link_libraries(RecastDemo PRIVATE "-F${SDL3_ROOT_DIR} -framework SDL3")
	target_include_directories(RecastDemo PRIVATE ${SDL3_INCLUDE_DIR})
else()
	target_link_libraries(RecastDemo PRIVATE SDL3::SDL3)
endif()

# Copy runtime files to build directory
if(WIN32)
	add_custom_command(TARGET RecastDemo POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SDL3_RUNTIME_LIBRARY}" $<TARGET_FILE_DIR:RecastDemo>/
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes $<TARGET_FILE_DIR:RecastDemo>/Meshes
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases $<TARGET_FILE_DIR:RecastDemo>/TestCases
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf $<TARGET_FILE_DIR:RecastDemo>/
	)
else()
	file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
	file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
	file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

# Installation
install(TARGETS RecastDemo
	RUNTIME DESTINATION bin
	BUNDLE DESTINATION bin
)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

if(WIN32)
	install(FILES "${SDL3_RUNTIME_LIBRARY}" DESTINATION bin)
endif()
