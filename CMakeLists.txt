
idf_component_register(
    SRCS    "icm45686.c"
            icm45686_imu/icm45686/imu/inv_imu_driver_advanced.c 
            icm45686_imu/icm45686/imu/inv_imu_driver_aux1.c 
            icm45686_imu/icm45686/imu/inv_imu_driver.c 
            icm45686_imu/icm45686/imu/inv_imu_edmp.c 
            icm45686_imu/icm45686/imu/inv_imu_selftest.c 
            icm45686_imu/icm45686/imu/inv_imu_transport.c
    INCLUDE_DIRS "." "icm45686_imu/icm45686/imu/"
)


component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)