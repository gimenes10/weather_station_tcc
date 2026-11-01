idf_component_register(
    SRCS "pulse.c"
    INCLUDE_DIRS "include"
    REQUIRES ulp    # Público: o build do ULP herda os includes deste componente
    PRIV_REQUIRES board_config esp_driver_gpio esp_hw_support
)

# Programa do ULP RISC-V (gera ulp_pulse.h e embute o binário)
set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_sources "ulp/pulse_ulp.c")
set(ulp_exp_dep_srcs "pulse.c")
ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
