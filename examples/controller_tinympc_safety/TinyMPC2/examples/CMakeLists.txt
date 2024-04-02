# add_executable(quadrotor_tracking quadrotor_tracking.cpp)
# target_link_libraries(quadrotor_tracking LINK_PUBLIC tinympc)

# add_executable(quadrotor_hovering quadrotor_hovering.cpp)
# target_link_libraries(quadrotor_hovering LINK_PUBLIC tinympc)

add_executable(rocket_landing_mpc rocket_landing_mpc.cpp)
target_link_libraries(rocket_landing_mpc LINK_PUBLIC tinympc)