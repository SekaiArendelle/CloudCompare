option( PLUGIN_STANDARD_QPIPECENTERLINE "Check to install qPipeCenterline plugin" OFF )

if( PLUGIN_STANDARD_QPIPECENTERLINE )

	project( QPIPECENTERLINE_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )
	
	# Add include directories
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
	)
	
	# Option to use Open3D (enabled by default if found)
	option(USE_OPEN3D_WITH_PIPECENTERLINE "Enable the use of Open3D with qPipeCenterline" ON)
	
	if (USE_OPEN3D_WITH_PIPECENTERLINE)
		find_package(Open3D QUIET)
		if(Open3D_FOUND)
			message(STATUS "Open3D found for qPipeCenterline: ${Open3D_INCLUDE_DIRS}")
			target_include_directories( ${PROJECT_NAME}
				PRIVATE
					${Open3D_INCLUDE_DIRS}
			)
			target_link_libraries( ${PROJECT_NAME} ${Open3D_LIBRARIES} )
			target_compile_definitions( ${PROJECT_NAME} PRIVATE USE_OPEN3D_WITH_PIPECENTERLINE )
		else()
			message(STATUS "Open3D not found, qPipeCenterline plugin will use fallback implementation")
		endif()
	endif()
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()