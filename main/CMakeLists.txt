idf_component_register(
    SRCS
        "main.c"
        "weather_task.c"
    INCLUDE_DIRS "include"
    REQUIRES board_config drivers sensors
)
