set(mc_mujoco_lib_SRC
    mj_configuration.h
    mj_sim.cpp
    mj_utils.cpp
    mj_sim.h
    mj_sim_impl.h
    mj_utils.h
    urlab_client.h
    urlab_client.cpp
    MujocoClient.cpp
    MujocoClient.h)

set(assets_DIR "${PROJECT_SOURCE_DIR}/assets")

set(imgui_DIR "${PROJECT_SOURCE_DIR}/ext/imgui")
set(imgui_SRC
    ${imgui_DIR}/imgui.h
    ${imgui_DIR}/imgui.cpp
    ${imgui_DIR}/imgui_demo.cpp
    ${imgui_DIR}/imgui_draw.cpp
    ${imgui_DIR}/imgui_tables.cpp
    ${imgui_DIR}/imgui_widgets.cpp
    ${imgui_DIR}/backends/imgui_impl_glfw.h
    ${imgui_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_DIR}/backends/imgui_impl_opengl3.h
    ${imgui_DIR}/backends/imgui_impl_opengl3.cpp)

set(implot_DIR "${PROJECT_SOURCE_DIR}/ext/implot")
set(implot_SRC ${implot_DIR}/implot.h ${implot_DIR}/implot.cpp
               ${implot_DIR}/implot_items.cpp ${implot_DIR}/implot_demo.cpp)

set(mc_rtc-imgui_DIR "${PROJECT_SOURCE_DIR}/ext/mc_rtc-imgui")
add_subdirectory("${mc_rtc-imgui_DIR}"
                 "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc-imgui/")

configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# URLab bridge dependencies: ZMQ (RPC transport) + msgpack-c (wire encoding).
# Both are widely packaged; prefer the system package, fall back to FetchContent
# if unavailable.
find_package(ZeroMQ QUIET)
if(NOT ZeroMQ_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(ZeroMQ QUIET IMPORTED_TARGET libzmq)
  endif()
endif()
find_package(cppzmq QUIET)
find_package(msgpack-cxx QUIET)
if(NOT msgpack-cxx_FOUND)
  find_package(msgpack QUIET)
endif()

if(NOT (ZeroMQ_FOUND OR TARGET PkgConfig::ZeroMQ)
   OR NOT cppzmq_FOUND
   OR NOT (msgpack-cxx_FOUND OR msgpack_FOUND))
  message(
    STATUS
      "libzmq/cppzmq/msgpack-c not all found via find_package, fetching from source. "
      "Install libzmq3-dev, cppzmq, and msgpack-cxx system packages to skip this."
  )
  include(FetchContent)
  if(NOT (ZeroMQ_FOUND OR TARGET PkgConfig::ZeroMQ))
    FetchContent_Declare(
      libzmq
      GIT_REPOSITORY https://github.com/zeromq/libzmq.git
      GIT_TAG v4.3.5)
    set(BUILD_TESTS
        OFF
        CACHE BOOL "" FORCE)
    set(WITH_PERF_TOOL
        OFF
        CACHE BOOL "" FORCE)
    set(ENABLE_CPACK
        OFF
        CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(libzmq)
  endif()
  if(NOT cppzmq_FOUND)
    FetchContent_Declare(
      cppzmq
      GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
      GIT_TAG v4.10.0)
    set(CPPZMQ_BUILD_TESTS
        OFF
        CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(cppzmq)
  endif()
  if(NOT (msgpack-cxx_FOUND OR msgpack_FOUND))
    FetchContent_Declare(
      msgpack-c
      GIT_REPOSITORY https://github.com/msgpack/msgpack-c.git
      GIT_TAG cpp-6.1.1)
    set(MSGPACK_BUILD_EXAMPLES
        OFF
        CACHE BOOL "" FORCE)
    set(MSGPACK_BUILD_TESTS
        OFF
        CACHE BOOL "" FORCE)
    set(MSGPACK_USE_BOOST
        OFF
        CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(msgpack-c)
  endif()
endif()

add_library(
  mc_mujoco_lib STATIC
  ${mc_mujoco_lib_SRC} ${imgui_SRC} ${implot_SRC} ${mc_rtc-imgui-SRC}
  ${mc_rtc-imgui-HDR} $<TARGET_OBJECTS:pugixml>)
target_include_directories(
  mc_mujoco_lib
  PUBLIC ${MUJOCO_INCLUDE_DIR} $<BUILD_INTERFACE:${MUJOCO_SAMPLE_DIR}>
         $<INSTALL_INTERFACE:include>)
if(DEFINED MUJOCO_ROOT_INCLUDE_DIR)
  target_include_directories(mc_mujoco_lib PUBLIC ${MUJOCO_ROOT_INCLUDE_DIR})
endif()
target_include_directories(mc_mujoco_lib
                           PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(
  mc_mujoco_lib PRIVATE "${imgui_DIR}" "${implot_DIR}" "${mc_rtc-imgui_DIR}"
                        "${assets_DIR}" "${PROJECT_SOURCE_DIR}/ext/pugixml")
target_link_libraries(
  mc_mujoco_lib PUBLIC mc_rtc::mc_control mc_rtc::mc_control_client
                       ${CMAKE_DL_LIBS})
if(UNIX)
  set(NO_AS_NEEDED "-Wl,--no-as-needed")
  set(AS_NEEDED "-Wl,--as-needed")
endif()
target_link_libraries(
  mc_mujoco_lib PRIVATE ${NO_AS_NEEDED} GLEW::GLEW OpenGL::GL ${AS_NEEDED}
                        ${LIB_MUJOCO})
if(GLFW)
  target_link_libraries(mc_mujoco_lib PRIVATE ${GLFW})
else()
  target_include_directories(
    mc_mujoco_lib PRIVATE "${PROJECT_SOURCE_DIR}/ext/glfw/include/GLFW")
  target_link_libraries(mc_mujoco_lib PRIVATE $<BUILD_INTERFACE:glfw>)
  if(UNIX)
    set(mc_mujoco_lib_combined_OUT
        ${CMAKE_CURRENT_BINARY_DIR}/combined/libmc_mujoco_lib_combined.a)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/combined)
    add_custom_command(
      OUTPUT ${mc_mujoco_lib_combined_OUT}
      COMMAND ar -x $<TARGET_FILE:mc_mujoco_lib>
      COMMAND ar -x $<TARGET_FILE:glfw>
      COMMAND ar -qcs ${mc_mujoco_lib_combined_OUT} *.o
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/combined
      DEPENDS mc_mujoco_lib glfw)
    add_custom_target(mc_mujoco_lib_combined ALL
                      DEPENDS ${mc_mujoco_lib_combined_OUT})
    install(FILES ${mc_mujoco_lib_combined_OUT} DESTINATION lib)
  endif()
endif()

# URLab bridge linkage
if(TARGET libzmq)
  target_link_libraries(mc_mujoco_lib PRIVATE libzmq)
elseif(TARGET PkgConfig::ZeroMQ)
  target_link_libraries(mc_mujoco_lib PRIVATE PkgConfig::ZeroMQ)
else()
  target_link_libraries(mc_mujoco_lib PRIVATE ${ZeroMQ_LIBRARIES})
  target_include_directories(mc_mujoco_lib PRIVATE ${ZeroMQ_INCLUDE_DIRS})
endif()
if(TARGET cppzmq)
  target_link_libraries(mc_mujoco_lib PRIVATE cppzmq)
elseif(TARGET cppzmq-static)
  target_link_libraries(mc_mujoco_lib PRIVATE cppzmq-static)
endif()
if(TARGET msgpack-cxx)
  target_link_libraries(mc_mujoco_lib PRIVATE msgpack-cxx)
elseif(TARGET msgpackc-cxx)
  target_link_libraries(mc_mujoco_lib PRIVATE msgpackc-cxx)
endif()

if(NOT COMBINED_MC_MUJOCO_LIB)
  set_target_properties(mc_mujoco_lib PROPERTIES EXPORT_NAME mc_mujoco)
endif()

add_executable(mc_mujoco main.cpp)
target_link_libraries(mc_mujoco PRIVATE mc_mujoco_lib CLI11::CLI11)

install(
  TARGETS mc_mujoco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES mj_sim.h mj_configuration.h DESTINATION include/mc_mujoco)

install(
  TARGETS mc_mujoco_lib
  EXPORT mc_mujocoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  EXPORT mc_mujocoTargets
  FILE mc_mujocoTargets.cmake
  NAMESPACE mc_mujoco::
  DESTINATION lib/cmake/mc_mujoco)
