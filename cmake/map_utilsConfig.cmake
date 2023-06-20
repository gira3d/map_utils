# - Try to find map_utils header files
#
# Once done this will define
#
# MAP_UTILS

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
set(map_utils_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/map_utils/include")
set(map_utils_LIBRARY "")
