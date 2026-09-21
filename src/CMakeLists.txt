# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(app_sources main.cpp tusb/usb_descriptors.c)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." "tusb"
                       REQUIRES espressif__usb_device_uac espressif__tinyusb)

# The UAC component is used in AS_PART mode so the project can expose the CDC
# control interface in the same USB configuration. TinyUSB must see our local
# tusb_config.h and descriptors instead of the component defaults.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
idf_component_get_property(uac_component espressif__usb_device_uac COMPONENT_DIR)
target_include_directories(${tusb_lib} PUBLIC
                           "${CMAKE_CURRENT_LIST_DIR}/tusb"
                           "${uac_component}/tusb_uac")
