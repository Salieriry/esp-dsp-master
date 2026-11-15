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

# Diagnostic-only source overlays. Verify the exact IDF sources before replacing
# build targets; the installed SDK stays untouched.
if(CMAKE_BINARY_DIR MATCHES "metrics")
    set(_metrics_driver_root "${CMAKE_SOURCE_DIR}/tools/metrics_driver/esp-idf-v5.5.0")
    set(_idf_gdma "$ENV{IDF_PATH}/components/esp_hw_support/dma/gdma.c")
    set(_idf_gdma_priv "$ENV{IDF_PATH}/components/esp_hw_support/dma/gdma_priv.h")
    set(_idf_i2s "$ENV{IDF_PATH}/components/esp_driver_i2s/i2s_common.c")
    set(_idf_i2s_private "$ENV{IDF_PATH}/components/esp_driver_i2s/i2s_private.h")
    foreach(_source_hash IN ITEMS
            "${_idf_gdma}|12f5c760b6e9efbbfd1d12278ba61fd878e12fec6f11052758a605f13e81de2b"
            "${_idf_gdma_priv}|1c140abeed7b14fd3dd35a57e41ba6531413dcff03653c2d061b2140344543a0"
            "${_idf_i2s}|9a3f1df3cfdf385c6ea9f05027dfcd8cb9f33f3f8a7af193bdc34ac0d0a7785c"
            "${_idf_i2s_private}|de56c1f41d10ea0e61d19c976c5b8a384d8ecadedaa9257dcff31bfab2163fde")
        string(REPLACE "|" ";" _pair "${_source_hash}")
        list(GET _pair 0 _source_path)
        list(GET _pair 1 _expected_hash)
        if(NOT EXISTS "${_source_path}")
            message(FATAL_ERROR "IDF driver source missing: ${_source_path}")
        endif()
        file(SHA256 "${_source_path}" _actual_hash)
        if(NOT _actual_hash STREQUAL _expected_hash)
            message(FATAL_ERROR "Unsupported ESP-IDF source hash for ${_source_path}: ${_actual_hash}")
        endif()
    endforeach()

    function(metrics_replace_component_source component suffix replacement)
        idf_component_get_property(_component_lib ${component} COMPONENT_LIB)
        if(NOT TARGET ${_component_lib})
            message(FATAL_ERROR "Cannot resolve IDF component target ${component}")
        endif()
        get_target_property(_sources ${_component_lib} SOURCES)
        set(_kept_sources)
        set(_found_source FALSE)
        foreach(_source IN LISTS _sources)
            if(_source MATCHES "${suffix}$")
                set(_found_source TRUE)
            else()
                list(APPEND _kept_sources "${_source}")
            endif()
        endforeach()
        if(NOT _found_source)
            message(FATAL_ERROR "Could not find ${suffix} in component ${component}")
        endif()
        set_property(TARGET ${_component_lib} PROPERTY SOURCES "${_kept_sources}")
        target_sources(${_component_lib} PRIVATE "${replacement}")
        target_include_directories(${_component_lib} PRIVATE
                                   "${CMAKE_SOURCE_DIR}/include"
                                   "${CMAKE_SOURCE_DIR}/../common")
        set(${component}_METRICS_LIB "${_component_lib}" PARENT_SCOPE)
    endfunction()

    metrics_replace_component_source(esp_hw_support "(^|/)dma/gdma\.c"
        "${_metrics_driver_root}/patched/components/esp_hw_support/dma/gdma.c")
    metrics_replace_component_source(esp_driver_i2s "(^|/)i2s_common\.c"
        "${_metrics_driver_root}/patched/components/esp_driver_i2s/i2s_common.c")
    idf_component_get_property(_esp_timer_lib esp_timer COMPONENT_LIB)
    target_link_libraries(${esp_driver_i2s_METRICS_LIB} PRIVATE ${_esp_timer_lib})
endif()
