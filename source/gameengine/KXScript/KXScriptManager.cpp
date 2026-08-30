#include "KXScriptManager.h"
#include "KXScriptLexer.h"
#include "KXScriptParser.h"
#include <iostream>
#include <sstream>

void KXScriptManager::runScript(const std::string& code) {
    KXLexer lexer(code);
    auto tokens = lexer.tokenize();
    
    KXParser parser(tokens);
    auto program = parser.parseProgram();
    
    interpreter.execute(program);
}

extern "C" const char* KXScript_RunString(const char* code) {
    static std::string last_output;
    last_output.clear();
    
    std::stringstream ss;
    auto old_buf = std::cout.rdbuf(ss.rdbuf());
    
    try {
        KXScriptManager manager;
        manager.runScript(std::string(code));
    }
    catch (const std::exception& e) {
        std::cout.rdbuf(old_buf);
        last_output = std::string("KXScript Error: ") + e.what();
        return last_output.c_str();
    }
    
    std::cout.rdbuf(old_buf);
    last_output = ss.str();
    return last_output.c_str();
}