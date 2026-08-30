#include "KXScriptLexer.h"
#include <cctype>

KXLexer::KXLexer(const std::string& src) : source(src) {}

char KXLexer::peek() {
    return pos < source.length() ? source[pos] : '\0';
}

char KXLexer::advance() {
    char c = source[pos++];
    if (c == '\n') line++;
    return c;
}

std::vector<KXToken> KXLexer::tokenize() {
    std::vector<KXToken> tokens;
    
    while (pos < source.length()) {
        char c = peek();
        
        // Skip whitespace
        if (isspace(c)) {
            advance();
            continue;
        }
        
        // Skip # comments
        if (c == '#') {
            while (pos < source.length() && peek() != '\n') advance();
            continue;
        }
        
        // Skip // comments
        if (c == '/' && pos + 1 < source.length() && source[pos + 1] == '/') {
            advance();
            advance();
            while (pos < source.length() && peek() != '\n') advance();
            continue;
        }
        
        // Numbers
        if (isdigit(c)) {
            std::string num;
            while (isdigit(peek()) || peek() == '.') num += advance();
            tokens.push_back({KXTokenType::NUMBER, num, line});
            continue;
        }
        
        // Strings
        if (c == '"') {
            advance();
            std::string str;
            while (peek() != '"' && pos < source.length()) str += advance();
            advance();
            tokens.push_back({KXTokenType::STRING, str, line});
            continue;
        }
        
        // Identifiers and keywords
        if (isalpha(c) || c == '_') {
            std::string ident;
            while (isalnum(peek()) || peek() == '_') ident += advance();
            
            KXTokenType type = KXTokenType::IDENTIFIER;
            if (ident == "var") type = KXTokenType::VAR;
            else if (ident == "if") type = KXTokenType::IF;
            else if (ident == "else") type = KXTokenType::ELSE;
            else if (ident == "while") type = KXTokenType::WHILE;
			else if (ident == "func") type = KXTokenType::FUNC;
			else if (ident == "return") type = KXTokenType::RETURN;
            else if (ident == "true") type = KXTokenType::TRUE;
            else if (ident == "false") type = KXTokenType::FALSE;
			else if (ident == "for") type = KXTokenType::FOR;
			else if (ident == "in") type = KXTokenType::IN;
			
            
            tokens.push_back({type, ident, line});
            continue;
        }
        
        // Operators
        switch(c) {
			case '+':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::PLUS_EQUALS, "+=", line});
				} else if (peek() == '+') {
					advance();
					tokens.push_back({KXTokenType::PLUS_PLUS, "++", line});
				} else {
					tokens.push_back({KXTokenType::PLUS, "+", line});
				}
				break;
			case '-':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::MINUS_EQUALS, "-=", line});
				} else if (peek() == '-') {
					advance();
					tokens.push_back({KXTokenType::MINUS_MINUS, "--", line});
				} else {
					tokens.push_back({KXTokenType::MINUS, "-", line});
				}
				break;
			case '*':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::MUL_EQUALS, "*=", line});
				} else {
					tokens.push_back({KXTokenType::STAR, "*", line});
				}
				break;
			case '/':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::DIV_EQUALS, "/=", line});
				} else {
					tokens.push_back({KXTokenType::SLASH, "/", line});
				}
				break;
            case '=':
                advance();
                if (peek() == '=') {
                    advance();
                    tokens.push_back({KXTokenType::EQUAL_EQUAL, "==", line});
                } else {
                    tokens.push_back({KXTokenType::EQUAL, "=", line});
                }
                break;
			case '<':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::LESS_EQUAL, "<=", line});
				} else {
					tokens.push_back({KXTokenType::LESS, "<", line});
				}
				break;
			case '>':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::GREATER_EQUAL, ">=", line});
				} else {
					tokens.push_back({KXTokenType::GREATER, ">", line});
				}
				break;
            case '(': advance(); tokens.push_back({KXTokenType::LPAREN, "(", line}); break;
            case ')': advance(); tokens.push_back({KXTokenType::RPAREN, ")", line}); break;
            case '{': advance(); tokens.push_back({KXTokenType::LBRACE, "{", line}); break;
            case '}': advance(); tokens.push_back({KXTokenType::RBRACE, "}", line}); break;
            case ',': advance(); tokens.push_back({KXTokenType::COMMA, ",", line}); break;
            case '.': advance(); tokens.push_back({KXTokenType::DOT, ".", line}); break;
            case ';': advance(); tokens.push_back({KXTokenType::SEMICOLON, ";", line}); break;
            case '&':
				advance();
				if (peek() == '&') {
					advance();
					tokens.push_back({KXTokenType::AND_AND, "&&", line});
				}
				break;
			case '!':
				advance();
				if (peek() == '=') {
					advance();
					tokens.push_back({KXTokenType::NOT_EQUAL, "!=", line});
				} else {
					tokens.push_back({KXTokenType::NOT, "!", line});
				}
				break;
			case '|':
				advance();
				if (peek() == '|') {
					advance();
					tokens.push_back({KXTokenType::OR_OR, "||", line});
				}
				break;
			case '[': advance(); tokens.push_back({KXTokenType::LBRACKET, "[", line}); break;
			case ']': advance(); tokens.push_back({KXTokenType::RBRACKET, "]", line}); break;
			case '$': advance(); tokens.push_back({KXTokenType::DOLLAR, "$", line}); break;
            default:
                advance();
        }
    }
    
    tokens.push_back({KXTokenType::EOF_TOKEN, "", line});
    return tokens;
}