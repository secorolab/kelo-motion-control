add_executable(solver_test_c
  solver_test.c
)
target_link_libraries(solver_test_c
  kelo_motion_control
)

add_executable(solver_test_cpp
  solver_test_cpp.cpp
)
target_link_libraries(solver_test_cpp
  kelo_motion_control
)

# install in project_name/examples/
install(TARGETS solver_test_c solver_test_cpp
  RUNTIME DESTINATION bin/${PROJECT_NAME}/examples
)