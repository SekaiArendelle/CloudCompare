option( PLUGIN_STANDARD_QPIPECENTERLINE "Check to install qPipeCenterline plugin" OFF )

if( PLUGIN_STANDARD_QPIPECENTERLINE )

	project( QPIPECENTERLINE_PLUGIN )

	# Find Open3D first
	find_package(Open3D QUIET)
	if(NOT OPEN3D_FOUND)
		message(STATUS "Open3D not found, qPipeCenterline plugin will use fallback implementation")
		set(OPEN3D_AVAILABLE FALSE)
	else()
		message(STATUS "Open3D found: ${OPEN3D_INCLUDE_DIRS}")
		set(OPEN3D_AVAILABLE TRUE)
	endif()

	AddPlugin( NAME ${PROJECT_NAME} )
	
	# Add Open3D include directories if available
	if(OPEN3D_AVAILABLE)
		target_include_directories( ${PROJECT_NAME}
			PRIVATE
				${CMAKE_CURRENT_SOURCE_DIR}/include
				${OPEN3D_INCLUDE_DIRS}
		)
		
		target_link_libraries( ${PROJECT_NAME}
			PRIVATE
				${OPEN3D_LIBRARIES}
		)
		
		# Define macro to indicate Open3D availability
		target_compile_definitions( ${PROJECT_NAME}
			PRIVATE
				OPEN3D_AVAILABLE
		)
	else()
		target_include_directories( ${PROJECT_NAME}
			PRIVATE
				${CMAKE_CURRENT_SOURCE_DIR}/include
		)
	endif()
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()