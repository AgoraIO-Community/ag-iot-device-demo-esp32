set(COMPONENT_ADD_INCLUDEDIRS include)

register_component()

target_link_libraries(${COMPONENT_TARGET} INTERFACE "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}")

add_prebuilt_library(rtsa "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/librtsa.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(ahpl "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libahpl.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(agora_iot_solution "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-iot-solution.a" PRIV_REQUIRES lwip)
add_prebuilt_library(cjson "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-cjson.a")
add_prebuilt_library(agora_mbedtls "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-mbedtls.a")
add_prebuilt_library(media_engine "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libmedia-engine.a")
add_prebuilt_library(iot_utility "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libiot-utility.a")
add_prebuilt_library(agora_webclient "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-webclient.a")
add_prebuilt_library(license_active "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/liblicense-active.a")
add_prebuilt_library(iot-audio-codec "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libiot-audio-codec.a")
add_prebuilt_library(agora-aws-shadow "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-aws-shadow.a")


target_link_libraries(${COMPONENT_TARGET} INTERFACE agora_iot_solution media_engine rtsa ahpl
	cjson iot_utility agora_webclient agora_mbedtls license_active iot-audio-codec agora-aws-shadow)
