cmake_minimum_required(VERSION 3.10)

#set(MYCODE_NAME livox_python_interface)
#add_executable(${MYCODE_NAME} main.c)

set(MYCODE_NAME livox_python_interface)

include_directories(.)

# 定义用于 DLL 导出的宏
#add_definitions(-DMYLIBRARY_EXPORTS)
add_library(${MYCODE_NAME} SHARED main.c)

target_link_libraries(${MYCODE_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )

target_compile_options(livox_python_interface PRIVATE -Wall -Wextra)
