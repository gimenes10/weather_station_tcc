idf_component_register(
    SRCS
        "main.c"
        "station.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES board_config drivers sensors
)
