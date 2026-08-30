set(INC
    .
    ../../blender/blenkernel
    ../../blender/blenlib
    ../../blender/makesdna
    ../../blender/makesrna
	../../blender/windowmanager
)

set(INC_SYS
)

set(SRC
    KXScriptLexer.cpp
    KXScriptParser.cpp
    KXScriptInterpreter.cpp
    KXScriptManager.cpp
    
    KXScriptLexer.h
    KXScriptParser.h
    KXScriptInterpreter.h
    KXScriptManager.h
    KXScriptAST.h
    KXScriptValue.h
)

blender_add_lib(bf_kxscript "${SRC}" "${INC}" "${INC_SYS}" bf_blenkernel bf_blenlib bf_dna bf_windowmanager)