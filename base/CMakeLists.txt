project(base)

aux_source_directory(bsp src)
aux_source_directory(common src)
# aux_source_directory(imu src)
# aux_source_directory(imu/driver src)
aux_source_directory(motor src)
aux_source_directory(motor/driver src)
aux_source_directory(remote src)

add_library(base STATIC ${src})
