# Dependencies

    find_package(GLIB)

# Targets

    # Build only necessary funcionality
    add_library(gts
        src/bbtree.c
        src/edge.c
        src/eheap.c
        src/face.c
        src/fifo.c
        src/kdtree.c
        src/misc.c
        src/object.c
        src/point.c
        src/predicates.c
        src/refine.c
        src/segment.c
        src/surface.c
        src/triangle.c
        src/tribox3.c
        src/vertex.c
    )

# Includes

    target_include_directories(gts
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC
            ${GLIB_INCLUDE_DIRS}
    )

# Linkage

    target_link_libraries(gts
        PRIVATE
            ${GLIB_LIBRARIES}
    )

# Configure

    configure_file("src/config.h.unix" "config.h")
    configure_file("src/predicates_init.h.unix" "predicates_init.h")
