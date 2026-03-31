# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

function(otl_config_file_enables option out_var)
    if(NOT EXISTS "${ARGV2}")
        set(${out_var} FALSE PARENT_SCOPE)
        return()
    endif()

    file(STRINGS "${ARGV2}" _otl_matches REGEX "^${option}=y$")
    if(_otl_matches)
        set(${out_var} TRUE PARENT_SCOPE)
    else()
        set(${out_var} FALSE PARENT_SCOPE)
    endif()
endfunction()

set(OTL_HOMEKIT_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/components/esp-homekit-sdk")
set(OTL_HOMEKIT_ENABLE FALSE)
set(_otl_config_sources "")

if(DEFINED SDKCONFIG AND EXISTS "${SDKCONFIG}")
    list(APPEND _otl_config_sources "${SDKCONFIG}")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
    list(APPEND _otl_config_sources "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
endif()

foreach(_otl_defaults IN LISTS SDKCONFIG_DEFAULTS)
    set(_otl_default_path "${_otl_defaults}")
    if(NOT IS_ABSOLUTE "${_otl_default_path}")
        set(_otl_default_path "${CMAKE_CURRENT_LIST_DIR}/${_otl_default_path}")
    endif()
    if(EXISTS "${_otl_default_path}")
        list(APPEND _otl_config_sources "${_otl_default_path}")
    endif()
endforeach()

foreach(_otl_config IN LISTS _otl_config_sources)
    otl_config_file_enables("CONFIG_OTL_HOMEKIT_ENABLE" _otl_homekit_in_file "${_otl_config}")
    if(_otl_homekit_in_file)
        set(OTL_HOMEKIT_ENABLE TRUE)
        break()
    endif()
endforeach()

if(OTL_HOMEKIT_ENABLE AND EXISTS "${OTL_HOMEKIT_SDK_DIR}/components/homekit/esp_hap_core")
    list(APPEND EXTRA_COMPONENT_DIRS
        "${OTL_HOMEKIT_SDK_DIR}/components/homekit/esp_hap_core"
        "${OTL_HOMEKIT_SDK_DIR}/components/homekit/esp_hap_apple_profiles"
        "${OTL_HOMEKIT_SDK_DIR}/components/homekit/esp_hap_platform"
        "${OTL_HOMEKIT_SDK_DIR}/components/homekit/hkdf-sha"
        "${OTL_HOMEKIT_SDK_DIR}/components/homekit/mu_srp")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(open-task-light)

if(OTL_HOMEKIT_ENABLE AND TARGET __idf_espressif__libsodium)
    target_compile_definitions(__idf_espressif__libsodium PRIVATE __STDC_WANT_LIB_EXT1__=1)
    target_compile_options(__idf_espressif__libsodium PRIVATE -std=gnu17)
endif()
