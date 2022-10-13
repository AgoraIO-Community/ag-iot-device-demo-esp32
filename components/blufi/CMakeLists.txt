

set(blufi_src
	"blufi_impl.c"
	"blufi_init.c"
	"blufi_security.c"
  )

idf_component_register(SRCS ${blufi_src}
                    INCLUDE_DIRS "."
										PRIV_REQUIRES nvs_flash)
