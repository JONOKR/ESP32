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
    # Common to every role: these are field/long-range units, so default to
    # the external antenna. Channel is explicit (not just relying on the
    # stock default) so all three roles are guaranteed to agree even if the
    # stock default ever changes upstream.
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
            "DB_BUILD_DEFAULT_CHANNEL=6"
            "DB_BUILD_DEFAULT_EN_EXT_ANT=true")
    if(DB_ROLE STREQUAL "AIR")
        # UART to the flight controller. These MUST be pinned: the generic-board
        # default is GPIO0 for BOTH tx and rx ("user has to set the correct pins",
        # db_parameters.h), so an unpinned AIR image joins the ESP-NOW mesh and
        # shows up at the GCS while carrying no FC telemetry at all - the most
        # confusing possible failure. Baud stays the 57600 default, which is what
        # ArduPilot uses on its telemetry ports.
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                "DB_BUILD_DEFAULT_RADIO_MODE=DB_WIFI_MODE_ESPNOW_AIR"
                "DB_BUILD_DEFAULT_GPIO_TX=21"
                "DB_BUILD_DEFAULT_GPIO_RX=2")
    elseif(DB_ROLE STREQUAL "GND")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                "DB_BUILD_DEFAULT_RADIO_MODE=DB_WIFI_MODE_ESPNOW_GND")
    elseif(DB_ROLE STREQUAL "BEACON")
        # Matches the verified GPS_beacon_example.cpp hardware: MicoAir M10
        # Ultra (factory 230400 baud) wired to ESP RX=GPIO20 / TX=GPIO21.
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                "DB_BUILD_DEFAULT_RADIO_MODE=DB_WIFI_MODE_ESPNOW_AIR"
                "DB_BUILD_DEFAULT_SERIAL_PROTO=DB_SERIAL_PROTOCOL_UBX_BEACON"
                "DB_BUILD_DEFAULT_BAUD=230400"
                "DB_BUILD_DEFAULT_GPIO_TX=21"
                "DB_BUILD_DEFAULT_GPIO_RX=20")
    else()
        message(FATAL_ERROR "Unknown DB_ROLE '${DB_ROLE}' (use AIR, GND or BEACON)")
    endif()
    message(STATUS "DroneBridge JONOKR role-baked build: ${DB_ROLE}")
endif()


if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_BINARY_DIR}/frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT DEPENDS frontend)
endif()
