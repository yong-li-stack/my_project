file(GLOB_RECURSE LV_DEMOS_SOURCES ./*.c)

idf_component_register(
    SRC_DIRS
        "."
        "ui"
        "ui/screens"
        "ui/images"
        "ui/fonts"
        "ui/components"

    INCLUDE_DIRS
        "."
        "ui")

set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_FLAGS
    "-DLV_LVGL_H_INCLUDE_SIMPLE -DLV_USE_DEMO_MUSIC")
