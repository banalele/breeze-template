cmake_minimum_required(VERSION 3.20)

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

# 把同目录下的 algo / ramenx 注册为 Zephyr 模块（各自的 zephyr/module.yml）
# 注意：必须在 find_package(Zephyr) 之前
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_LIST_DIR}/algo
    ${CMAKE_CURRENT_LIST_DIR}/ramenx
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(breeze_template)

# 关键：app 必须与 algo 库使用一致的 ALGO_ENABLE_ARM_MATH 定义。
# 该宏决定 Matrix<float> 走 CMSIS-DSP 特化（含 armBuffer_，sizeof=1080）
# 还是纯头文件模板（sizeof=1044）。algo 库在 algo/CMakeLists.txt 里
# 通过 add_compile_definitions 定义了它，但作用域只在 algo 模块内，
# 不会传到 app。若 app 不定义，两边对 Matrix<float> 的布局理解不一致
# （ODR 违规），Ekf::Config 拷贝时偏移错位、读到垃圾 rows/cols，
# 最终触发 BUS FAULT（BFAR 通常是个非法地址）。
if(CONFIG_ALGO_ENABLE_ARM_MATH)
    target_compile_definitions(app PRIVATE ALGO_ENABLE_ARM_MATH)
endif()

# 直接添加所有源文件到 app
target_sources(app PRIVATE

    src/main.cpp
    src/TaskLayer/all_task.cpp
    src/TaskLayer/conf_task.cpp
    src/DeviceLayer/device.cpp
    src/DeviceLayer/iwdg.c
    src/DeviceLayer/rtt.c
)
# 添加头文件路径
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/algo/include   # algo 算法库头文件（robotpilots/algorithm/*.hpp）
    src/DeviceLayer                 # 设备层头文件（如 device.hpp）
    src/TaskLayer                   # 任务层头文件（如 conf_task.hpp）
    src/ConfigLayer                 # 配置层头文件（如 rp_device_config.h）
)

target_link_libraries(app PRIVATE ramenx)
