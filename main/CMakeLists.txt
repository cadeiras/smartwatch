file(GLOB_RECURSE EBIKE_SOURCES "ebike/*.c")
file(GLOB_RECURSE SMARTWATCH_SOURCES "smartwatch/*.c")

idf_component_register(SRCS "main.c" 
                    ${EBIKE_SOURCES}        # <--- Adiciona os ficheiros do ebike
                    ${SMARTWATCH_SOURCES}
                    INCLUDE_DIRS "." "ebike"  # <--- Adiciona a pasta ebike aos caminhos de busca
                    REQUIRES lvgl)