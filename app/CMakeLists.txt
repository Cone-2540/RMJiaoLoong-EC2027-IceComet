project(app)
aux_source_directory(. src)
add_library(app STATIC ${src})
