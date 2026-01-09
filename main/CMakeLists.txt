idf_component_register(
                        SRCS
                            "main.c"

                        INCLUDE_DIRS 
                            ""include""
                        REQUIRES 
                            vl53l0x
                            picoros 
                        )

idf_build_set_property(COMPILE_OPTIONS -DUSER_TYPE_FILE="inc/my_ros_types.h" APPEND)
idf_build_set_property(COMPILE_OPTIONS -I${CMAKE_CURRENT_SOURCE_DIR} APPEND)