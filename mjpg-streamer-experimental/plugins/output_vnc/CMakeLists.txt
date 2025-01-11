include(FindPixman)
include(FindAML)
include(FindNeatVNC)

MJPG_STREAMER_PLUGIN_OPTION(output_vnc "VNC plugin"
                            JPEG_LIB)

if (PLUGIN_OUTPUT_VNC)
    include_directories(${PIXMAN_INCLUDE_DIRS} ${JPEG_INCLUDE_DIRS} ${NeatVNC_INCLUDE_DIRS})
    MJPG_STREAMER_PLUGIN_COMPILE(output_vnc output_vnc.c)
    target_link_libraries(output_vnc ${PIXMAN_LIBRARIES} ${AML_LIBRARIES} ${NeatVNC_LIBRARIES} ${JPEG_LIB})
endif()
