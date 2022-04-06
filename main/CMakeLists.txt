idf_component_register(SRCS video_doorbell.c
                    INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include"
					REQUIRES camera audio_pipeline audio_hal audio_stream audio_board esp_peripherals esp-adf-libs input_key_service esp_wifi nvs_flash)

# Import the library, specifying a target name and the library path.
# The private dependencies of the library is also specified.
add_prebuilt_library(rtsa "${CMAKE_SOURCE_DIR}/lib/${idf_target}/librtsa.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(ahpl "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libahpl.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(agora_iot_solution "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-iot-solution.a" PRIV_REQUIRES lwip)
add_prebuilt_library(cjson "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-cjson.a")
add_prebuilt_library(http_parser "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libhttp-parser.a")
add_prebuilt_library(agora_mbedtls "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-mbedtls.a")
add_prebuilt_library(media_engine "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libmedia-engine.a")
add_prebuilt_library(iot_utility "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libiot-utility.a")
add_prebuilt_library(agora_webclient "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libagora-webclient.a")
add_prebuilt_library(iot_license "${CMAKE_SOURCE_DIR}/lib/${idf_target}/libiot-license.a")

# `main` calls a function from the library, so link it to `main`
target_link_libraries(${COMPONENT_LIB} PRIVATE agora_iot_solution media_engine rtsa ahpl
	cjson http_parser iot_utility agora_webclient agora_mbedtls iot_license)