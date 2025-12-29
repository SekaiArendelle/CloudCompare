option( PLUGIN_STANDARD_QPIPECENTERLINE "Check to install qPipeCenterline plugin" OFF )

if( PLUGIN_STANDARD_QPIPECENTERLINE )

	project( QPIPECENTERLINE_PLUGIN )

	# Find Open3D first - required for this plugin
	find_package(Open3D REQUIRED)
	if(NOT Open3D_FOUND)
		message(FATAL_ERROR "Open3D is required for qPipeCenterline plugin but was not found!")
	endif()

	AddPlugin( NAME ${PROJECT_NAME} )
	
	# Add include directories
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${Open3D_INCLUDE_DIRS}
	)
	
	# Link Open3D libraries
	target_link_libraries( ${PROJECT_NAME}
		${Open3D_LIBRARIES}
	)
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()