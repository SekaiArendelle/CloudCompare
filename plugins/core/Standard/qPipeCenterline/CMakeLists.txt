option( PLUGIN_STANDARD_QPIPECENTERLINE "Check to install qPipeCenterline plugin" OFF )

if( PLUGIN_STANDARD_QPIPECENTERLINE )

	project( QPIPECENTERLINE_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
	)
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()