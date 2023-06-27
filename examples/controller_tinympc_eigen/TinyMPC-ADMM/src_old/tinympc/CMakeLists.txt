# if (CLANG_TIDY AND (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
#   message(STATUS "Running clang-tidy on source directory.")
#   set(CMAKE_C_CLANG_TIDY
#     clang-tidy;
#     -export-fixes=tidy-fixes.yaml;
#     -header-filter=.;
#     --config-file=${PROJECT_SOURCE_DIR}/.clang-tidy;
#     )
# endif ()

add_library(tinympc
  tinympc.h

  constants.h

  errors.h
  
  utils.h
  utils.cpp

  types.h

  model.h
  model.cpp

  auxil.h
  auxil.cpp
  
  cost_lqr.h
  cost_lqr.cpp

  lqr.h
  lqr.cpp

  constraint_linear.h
  constraint_linear.cpp

  admm.h
  admm.cpp
  )

target_include_directories(tinympc
  INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:"include">
  )

target_link_libraries(tinympc
  PRIVATE

  )

add_library(tinympc::tinympc ALIAS tinympc)