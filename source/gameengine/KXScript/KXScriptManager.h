#pragma once

#include "KXScriptInterpreter.h"
#include <string>
#include "DNA_object_types.h"

class KXScriptManager {
private:
    KXInterpreter interpreter;
    std::string output;
	
	Object* findBlenderObject(const std::string& name);

    
public:
    void runScript(const std::string& code);
    std::string getOutput() { return output; }
};