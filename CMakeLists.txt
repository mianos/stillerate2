cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stillerate2)

# ------------------ Post-Build OTA Copy ------------------

# 1. Setup the path (Default or Env Var)
if(DEFINED ENV{OTA_TARGET_PATH})
    set(OTA_TARGET_PATH $ENV{OTA_TARGET_PATH})
else()
    set(OTA_TARGET_PATH "/Volumes/files/ota")
endif()

# 2. Define the exact command
# We use 'add_custom_target' with 'ALL' so it runs on every build.
# We 'DEPENDS gen_project_binary' so it waits for the .bin to be created.
add_custom_target(copy_firmware ALL
    # Ensure the destination directory exists
    COMMAND ${CMAKE_COMMAND} -E make_directory ${OTA_TARGET_PATH}
    
    # Copy the binary
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
            ${OTA_TARGET_PATH}/${PROJECT_NAME}.bin
            
    # Wait for the standard ESP-IDF binary generation to finish
    DEPENDS gen_project_binary
    
    COMMENT "Copying ${PROJECT_NAME}.bin to ${OTA_TARGET_PATH}..."
)
