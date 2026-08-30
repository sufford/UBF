#pragma once

#include <string>
#include <vector>

enum class KXTokenType {
    NUMBER, STRING, IDENTIFIER,
    VAR, IF, ELSE, WHILE, FUNC, RETURN, FOR, IN, TRUE, FALSE,
    PLUS, MINUS, STAR, SLASH, EQUAL, EQUAL_EQUAL,
    LESS, GREATER, LESS_EQUAL, GREATER_EQUAL, NOT_EQUAL, 
	AND_AND, OR_OR, NOT,
	LPAREN, RPAREN, LBRACE, RBRACE, LBRACKET, RBRACKET,
    COMMA, DOT, SEMICOLON, DOLLAR, 
    PLUS_EQUALS, MINUS_EQUALS, MUL_EQUALS, DIV_EQUALS,
    PLUS_PLUS, MINUS_MINUS,
    EOF_TOKEN
};

struct KXToken {
    KXTokenType type;
    std::string value;
    int line;
};

class KXLexer {
private:
    std::string source;
    size_t pos = 0;
    int line = 1;
    
    char peek();
    char advance();
    
public:
    KXLexer(const std::string& src);
    std::vector<KXToken> tokenize();
};