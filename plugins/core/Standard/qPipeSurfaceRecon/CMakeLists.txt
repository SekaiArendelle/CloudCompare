option( PLUGIN_STANDARD_QPIPE_SURFACE_RECON "Install qPipeSurfaceRecon plugin" OFF )

if( PLUGIN_STANDARD_QPIPE_SURFACE_RECON )
	project( QPIPE_SURFACE_RECON_PLUGIN )
		
	AddPlugin( NAME ${PROJECT_NAME} )

	# Add PCL dependency for pipe-specific algorithms
	find_package( PCL 1.9 REQUIRED )
	
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( images )
	
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${PCL_INCLUDE_DIRS}
	)
	
	if( CMAKE_VERSION VERSION_LESS 3.13 )
		link_directories( ${PCL_LIBRARY_DIRS} )
	else()
		target_link_directories( ${PROJECT_NAME}
			PRIVATE
				${PCL_LIBRARY_DIRS}
		)
	endif()
	
	target_link_libraries( ${PROJECT_NAME} 
		${PCL_LIBRARIES}
		QCC_DB_LIB
		QCC_IO_LIB
	)
endif()