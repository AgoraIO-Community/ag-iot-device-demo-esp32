set(COMPONENT_ADD_INCLUDEDIRS include)

#set(COMPONENT_REQUIRES mqtt json)

register_component()

target_link_libraries(${COMPONENT_TARGET} INTERFACE "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}")

add_prebuilt_library(rtsa "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/librtsa.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(ahpl "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libahpl.a" PRIV_REQUIRES lwip pthread)
add_prebuilt_library(agora_iot_solution "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-iot-solution.a" PRIV_REQUIRES lwip)
add_prebuilt_library(agora_callkit "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-callkit.a")
add_prebuilt_library(agora_aws_shadow "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-aws-shadow.a")
add_prebuilt_library(cjson "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-cjson.a")
add_prebuilt_library(http_parser "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libhttp-parser.a")
add_prebuilt_library(agora_mbedtls "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-mbedtls.a")
add_prebuilt_library(media_engine "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libmedia-engine.a")
add_prebuilt_library(iot_utility "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libiot-utility.a")
add_prebuilt_library(agora_webclient "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libagora-webclient.a")
add_prebuilt_library(iot_license "${CMAKE_CURRENT_SOURCE_DIR}/lib/${idf_target}/libiot-license.a")

target_link_libraries(${COMPONENT_TARGET} INTERFACE agora_iot_solution agora_callkit agora_aws_shadow media_engine rtsa ahpl
	cjson http_parser iot_utility agora_webclient agora_mbedtls iot_license)
