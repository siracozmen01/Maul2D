add_executable(sample_car car.c)
target_link_libraries(sample_car PRIVATE maul2d)
maul2d_apply_flags(sample_car)

add_executable(sample_replay replay.c)
target_link_libraries(sample_replay PRIVATE maul2d)
maul2d_apply_flags(sample_replay)
