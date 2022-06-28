

set(quirc_src
	"lib/decode.c"
	"lib/identify.c"
	"lib/quirc.c"
	"lib/version_db.c"
	"lib/convert.c"
  )


idf_component_register(SRCS ${quirc_src}
                    INCLUDE_DIRS "include"
										PRIV_INCLUDE_DIRS "private_include")
