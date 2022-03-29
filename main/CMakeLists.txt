idf_component_register(SRCS video_doorbell.c
                    INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include"
					REQUIRES camera audio_pipeline audio_hal audio_stream audio_board esp_peripherals esp-adf-libs input_key_service esp_wifi nvs_flash)

# Import the library, specifying a target name and the library path.
# The private dependencies of the library is also specified.
add_prebuilt_library(agora_sdk_lib "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-rtc.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(agora_ahpl "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libahpl.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(agora_iot_solution "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-iot-solution.a" PRIV_REQUIRES lwip)
add_prebuilt_library(cjson "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libcjson.a")
add_prebuilt_library(http_parser "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libhttp-parser.a")
add_prebuilt_library(agora_mbedtls "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-mbedtls.a")
add_prebuilt_library(media_engine "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libmedia-engine.a")
add_prebuilt_library(utility "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libutility.a")
add_prebuilt_library(webclient "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libwebclient.a")
add_prebuilt_library(license "${CMAKE_SOURCE_DIR}/lib/${idf_target}/liblicense.a")

# `main` calls a function from the library, so link it to `main`
target_link_libraries(${COMPONENT_LIB} PRIVATE agora_iot_solution media_engine agora_sdk_lib agora_ahpl
	cjson http_parser utility webclient agora_mbedtls license)