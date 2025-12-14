# Only compile the legacy ESP-IDF entry point.
idf_component_register(SRCS "main.c" INCLUDE_DIRS "")
