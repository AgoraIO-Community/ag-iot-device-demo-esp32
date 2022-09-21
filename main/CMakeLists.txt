idf_component_register(SRCS video_doorbell.c device_state.c doorbell_dp_common.c blufi_init.c blufi_security.c blufi_impl.c
					REQUIRES camera quirc audio_pipeline audio_hal audio_stream audio_board esp_peripherals esp-adf-libs input_key_service esp_wifi nvs_flash agora_iot_sdk)
