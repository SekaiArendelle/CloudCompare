cmake_minimum_required(VERSION 3.10)

option( PLUGIN_STANDARD_QCROSSSECTIONFEATURES "Check to install qCrossSectionFeatures plugin" OFF )

if( PLUGIN_STANDARD_QCROSSSECTIONFEATURES )

    project( QCROSSSECTIONFEATURES_PLUGIN )

    AddPlugin( NAME ${PROJECT_NAME} )

    # 【关键】加入 segmentation, surface, kdtree, sample_consensus
    find_package(PCL REQUIRED COMPONENTS common io filters surface features segmentation kdtree sample_consensus search)

    target_sources( ${PROJECT_NAME}
        PRIVATE
            include/qCrossSectionFeatures.h
            include/qCrossSectionFeaturesDialog.h
            include/CrossSectionExtractor.h
            src/qCrossSectionFeatures.cpp
            src/qCrossSectionFeaturesDialog.cpp
            src/CrossSectionExtractor.cpp
            qCrossSectionFeatures.qrc
            info.json
    )

    target_include_directories( ${PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${PCL_INCLUDE_DIRS}
    )

    target_compile_definitions( ${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS} )

    target_link_libraries( ${PROJECT_NAME}
        QCC_DB_LIB
        QCC_IO_LIB
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
        ${PCL_LIBRARIES}
    )

endif()
