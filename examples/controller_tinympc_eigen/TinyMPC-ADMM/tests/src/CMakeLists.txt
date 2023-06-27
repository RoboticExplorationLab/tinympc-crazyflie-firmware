# function add_al_lqr_test(name)
#
# Adds a new executable test called <name>_test.
# Assumes thes source code is in a file called <name>_test.
function(add_my_test name)
  set(TEST_NAME ${name}_test)
  add_executable(${TEST_NAME}
    ${TEST_NAME}.c

    test_utils.h
    test_utils.c  

    ${ARGN}
    )
  target_link_libraries(${TEST_NAME}
    PRIVATE
    simpletest
    slap::slap
    tinympc::tinympc
    m  # math library
    )
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# add_subdirectory(data)

# Add tests
add_my_test(model bicycle_5d.h bicycle_5d.c)
add_my_test(setup_prob)
# add_my_test(forward_pass)
add_my_test(cost)
# add_my_test(back_pass)
add_my_test(lqr_lti)
add_my_test(lqr_lti_track)
# add_my_test(lqr_ltv_track bicycle_5d.h bicycle_5d.c)
# add_my_test(linear_cstr)
add_my_test(admm)
add_my_test(admm_lti)
# add_my_test(al_lqr_ltv bicycle_5d.h bicycle_5d.c)