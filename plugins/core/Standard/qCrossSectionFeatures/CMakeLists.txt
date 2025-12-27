option( PLUGIN_STANDARD_QCROSSSECTIONFEATURES "Check to install qCrossSectionFeatures plugin" OFF )

if( PLUGIN_STANDARD_QCROSSSECTIONFEATURES )

	project( QCROSSSECTIONFEATURES_PLUGIN )

	AddPlugin( NAME ${PROJECT_NAME} )

	add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/include )
	add_subdirectory( src )

	target_link_libraries( ${PROJECT_NAME}
		QCC_DB_LIB
		QCC_IO_LIB
	)

endif()