#pragma once

#include "KXScriptAST.h"
#include "KXScriptValue.h"
#include <map>
#include <string>
#include <vector>
#include <memory>

struct Object;

class KXInterpreter {
private:
    std::map<std::string, KXValue> variables;
    std::map<std::string, KXFunctionExpr*> functions;
    
    bool isTruthy(const KXValue& value);
    Object* findBlenderObject(const std::string& name);  // ADD THIS
    
public:
    KXValue evaluate(KXExpr* expr);
    void execute(const std::vector<std::unique_ptr<KXExpr>>& program);
};