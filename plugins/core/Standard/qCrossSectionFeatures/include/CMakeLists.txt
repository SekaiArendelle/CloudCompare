target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qCrossSectionFeatures.h
		${CMAKE_CURRENT_LIST_DIR}/qCrossSectionFeaturesDialog.h
		${CMAKE_CURRENT_LIST_DIR}/CrossSectionExtractor.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)