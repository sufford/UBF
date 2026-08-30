set(INC
	../include
	../../blenfont
	../../blenkernel
	../../blenlib
	../../blentranslation
	../../gpu
	../../makesdna
	../../makesrna
	../../windowmanager
	../../../../intern/guardedalloc
	../../../../intern/glew-mx
	../../../gameengine/KXScript
)

set(INC_SYS
	${GLEW_INCLUDE_PATH}
)

set(SRC
	space_text.c
	text_autocomplete.c
	text_draw.c
	text_format.c
	text_format_kx.c
	text_format_lua.c
	text_format_osl.c
	text_format_pov.c
	text_format_pov_ini.c
	text_format_py.c
	text_header.c
	text_ops.c
	text_undo.c

	text_format.h
	text_intern.h
)

add_definitions(${GL_DEFINITIONS})

if(WITH_PYTHON)
	list(APPEND INC
		../../python
	)
	add_definitions(-DWITH_PYTHON)
endif()

if(WITH_INTERNATIONAL)
	add_definitions(-DWITH_INTERNATIONAL)
endif()

blender_add_lib(bf_editor_space_text "${SRC}" "${INC}" "${INC_SYS}" bf_kxscript)