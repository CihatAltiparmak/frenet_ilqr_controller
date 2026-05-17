add_executable(ilqr_optimizer_demo ilqr_optimizer_demo.cpp)
target_link_libraries(ilqr_optimizer_demo ilqr_trajectory_tracker_lib)

install(TARGETS ilqr_optimizer_demo
        DESTINATION lib/ilqr_trajectory_tracker)
