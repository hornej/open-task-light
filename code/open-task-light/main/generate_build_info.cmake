if(NOT DEFINED OUTPUT_FILE)
    message(FATAL_ERROR "OUTPUT_FILE is required")
endif()

string(TIMESTAMP OTL_BUILD_TIMESTAMP "%b %d %Y %H:%M:%S")

file(WRITE "${OUTPUT_FILE}" "#pragma once\n")
file(APPEND "${OUTPUT_FILE}" "#define OTL_BUILD_TIMESTAMP \"${OTL_BUILD_TIMESTAMP}\"\n")
