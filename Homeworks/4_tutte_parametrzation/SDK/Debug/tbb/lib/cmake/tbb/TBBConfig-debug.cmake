#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "TBB::tbb" for configuration "Debug"
set_property(TARGET TBB::tbb APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(TBB::tbb PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/tbb_debug.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/tbb_debug.dll"
  )

list(APPEND _cmake_import_check_targets TBB::tbb )
list(APPEND _cmake_import_check_files_for_TBB::tbb "${_IMPORT_PREFIX}/lib/tbb_debug.lib" "${_IMPORT_PREFIX}/bin/tbb_debug.dll" )

# Import target "TBB::tbbmalloc" for configuration "Debug"
set_property(TARGET TBB::tbbmalloc APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(TBB::tbbmalloc PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/tbbmalloc_debug.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/tbbmalloc_debug.dll"
  )

list(APPEND _cmake_import_check_targets TBB::tbbmalloc )
list(APPEND _cmake_import_check_files_for_TBB::tbbmalloc "${_IMPORT_PREFIX}/lib/tbbmalloc_debug.lib" "${_IMPORT_PREFIX}/bin/tbbmalloc_debug.dll" )

# Import target "TBB::tbbmalloc_proxy" for configuration "Debug"
set_property(TARGET TBB::tbbmalloc_proxy APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(TBB::tbbmalloc_proxy PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/tbbmalloc_proxy_debug.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/tbbmalloc_proxy_debug.dll"
  )

list(APPEND _cmake_import_check_targets TBB::tbbmalloc_proxy )
list(APPEND _cmake_import_check_files_for_TBB::tbbmalloc_proxy "${_IMPORT_PREFIX}/lib/tbbmalloc_proxy_debug.lib" "${_IMPORT_PREFIX}/bin/tbbmalloc_proxy_debug.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
