#pragma once

#include "KXScriptLexer.h"
#include "KXScriptAST.h"
#include <vector>
#include <memory>

class KXParser {
private:
    std::vector<KXToken> tokens;
    size_t current = 0;
    
    KXToken peek();
    KXToken advance();
    bool check(KXTokenType t);
    bool match(KXTokenType t);
    
    // ADD THESE if missing:
    std::unique_ptr<KXExpr> parseIfStatement();
    std::unique_ptr<KXExpr> parseWhileStatement();
    std::unique_ptr<KXExpr> parseFunctionDefinition();
    std::unique_ptr<KXExpr> parseReturnStatement();
    std::unique_ptr<KXExpr> parseFunctionCall(const std::string& name);
	std::unique_ptr<KXExpr> parseUnary();
	std::unique_ptr<KXExpr> parseLogical();
	std::unique_ptr<KXExpr> parseForStatement();
	std::unique_ptr<KXExpr> parseIndex();
    
public:
    KXParser(const std::vector<KXToken>& toks);
    std::vector<std::unique_ptr<KXExpr>> parseProgram();
    std::unique_ptr<KXExpr> parseStatement();
    std::unique_ptr<KXExpr> parseExpression();
    std::unique_ptr<KXExpr> parseEquality();
    std::unique_ptr<KXExpr> parseComparison();
    std::unique_ptr<KXExpr> parseTerm();
    std::unique_ptr<KXExpr> parseFactor();
    std::unique_ptr<KXExpr> parsePrimary();
};