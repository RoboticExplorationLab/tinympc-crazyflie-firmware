function(add_example name)
  set(EXAMPLE_NAME ${name}_example)
  add_executable(${EXAMPLE_NAME}
    ${EXAMPLE_NAME}.c
    ${ARGN}
    )
  target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
    simpletest
    slap::slap
    tinympc::tinympc
    )
endfunction()

# add_example(bicycle_3d bicycle_3d.h bicycle_3d.c)
# add_example(bicycle_5d bicycle_5d.h bicycle_5d.c)
add_example(quadrotor quadrotor.h quadrotor.c)
add_example(quadrotor_track quadrotor.h quadrotor.c)