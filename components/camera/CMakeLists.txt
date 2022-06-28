

set(camera_src
	"cam_hal.c"
	"esp_camera.c"
	"sccb.c"
	"sensor.c"
	"sensors/ov2640.c"
	"sensors/ov3660.c"
	"sensors/ov5640.c"
	"sensors/ov7725.c"
	"sensors/ov7670.c"
	"sensors/nt99141.c"
	"sensors/gc0308.c"
	"sensors/gc2145.c"
	"sensors/gc032a.c"
	"sensors/bf3005.c"
	"sensors/bf20a6.c"
    	"sensors/sc030iot.c"
	"conversions/yuv.c"
	"conversions/to_jpg.cpp"
	"conversions/to_bmp.c"
	"conversions/jpge.cpp"
	"conversions/esp_jpg_decode.c"
  )

if(IDF_TARGET STREQUAL "esp32s3")
	list(APPEND camera_src
		target/esp32s3/ll_cam.c
		)
endif()

idf_component_register(SRCS ${camera_src}
                    INCLUDE_DIRS "." "include" "conversions/include"
										PRIV_INCLUDE_DIRS "private_include" "target/private_include"
										"sensors/private_include" "conversions/private_include"
										PRIV_REQUIRES nvs_flash)
