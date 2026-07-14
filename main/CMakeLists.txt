set(SRCS
        main.c
        tcp_server.c
        msp_ltm_serial.c
        http_server.c
        db_esp32_control.c
        db_crc.c
        db_esp_now.c
        db_serial.c
        db_mavlink_msgs.c
        db_parameters.c
        db_led_indicator.c
        db_timers.c
        db_gps_beacon.c
)

if(CONFIG_BT_ENABLED)
    list(APPEND SRCS db_ble.c)
endif()

idf_component_register(SRCS ${SRCS}
        INCLUDE_DIRS ".")

# Role-baked provisioning (JONOKR): `idf.py -DDB_ROLE=AIR|GND|BEACON build`
# bakes that role's boot defaults into the image (see db_parameters.h), so the
# flasher GUI produces units that need zero web-UI configuration. The values
# are passed as the enum SYMBOLS, evaluated in C where the enums are defined.
if(DEFINED DB_ROLE)
    if(DB_ROLE STREQUAL "AIR")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                "DB_BUILD_DEFAULT_RADIO_MODE=DB_WIFI_MODE_ESPNOW_AIR")
    elseif(DB_ROLE STREQUAL "GND")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                "DB_BUILD_DEFAULT_RADIO_MODE=DB_WIFI_MODE_ESPNOW_GND")
    elseif(DB_ROLE STREQUAL "BEACON")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                "DB_BUILD_DEFAULT_RADIO_MODE=DB_WIFI_MODE_ESPNOW_AIR"
                "DB_BUILD_DEFAULT_SERIAL_PROTO=DB_SERIAL_PROTOCOL_UBX_BEACON"
                "DB_BUILD_DEFAULT_BAUD=115200")
    else()
        message(FATAL_ERROR "Unknown DB_ROLE '${DB_ROLE}' (use AIR, GND or BEACON)")
    endif()
    message(STATUS "DroneBridge JONOKR role-baked build: ${DB_ROLE}")
endif()


if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_BINARY_DIR}/frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT DEPENDS frontend)
endif()
