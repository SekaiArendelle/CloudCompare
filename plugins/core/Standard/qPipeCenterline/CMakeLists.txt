option( PLUGIN_STANDARD_QPIPECENTERLINE "Check to install qPipeCenterline plugin" OFF )

if( PLUGIN_STANDARD_QPIPECENTERLINE )

	project( QPIPECENTERLINE_PLUGIN )

	# TODO Maybe Open3d can be removed
	find_package(Open3D REQUIRED)
	if(NOT Open3D_FOUND)
		message(FATAL_ERROR "Open3D is required for qPipeCenterline plugin but was not found!")
	endif()

	find_package(PCL REQUIRED)
	if(NOT PCL_FOUND)
		message(FATAL_ERROR "PCL is required for qPipeCenterline plugin but was not found!")
	endif()

	AddPlugin( NAME ${PROJECT_NAME} )
	# Find Eigen3 (required by Open3D)
	find_package(Eigen3 3.4 REQUIRED NO_MODULE)

	# Eigen is already available through pixi/conda environment
	# The main CMakeLists.txt sets EIGEN_ROOT_DIR for the entire project

	# Add include directories
	target_include_directories( ${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
			# Open3D
			${Open3D_INCLUDE_DIRS}
			# Eigen
			${EIGEN_ROOT_DIR}
			# PCL
			${PCL_INCLUDE_DIRS}
	)

	# Link Open3D libraries and Eigen
	target_link_libraries( ${PROJECT_NAME}
		${Open3D_LIBRARIES}
		Eigen3::Eigen
		${PCL_LIBRARIES}
	)

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()