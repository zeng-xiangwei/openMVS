if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

find_package(PCL REQUIRED)
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
cxx_executable_with_flags(InterfaceFastLivo "Apps" "${cxx_default}" "${PCL_LIBRARIES};MVS" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
target_include_directories(InterfaceFastLivo PUBLIC ${PCL_INCLUDE_DIRS})
# target_link_libraries(InterfaceFastLivo ${PCL_LIBRARIES})

# Install
INSTALL(TARGETS InterfaceFastLivo
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
