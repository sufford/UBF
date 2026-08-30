#include "KXScriptParser.h"
#include <stdexcept>
#include <algorithm>

KXParser::KXParser(const std::vector<KXToken>& toks) : tokens(toks) {}

KXToken KXParser::peek() { return tokens[current]; }
KXToken KXParser::advance() { return tokens[current++]; }
bool KXParser::check(KXTokenType t) { return peek().type == t; }
bool KXParser::match(KXTokenType t) {
    if (check(t)) { advance(); return true; }
    return false;
}

std::vector<std::unique_ptr<KXExpr>> KXParser::parseProgram() {
    std::vector<std::unique_ptr<KXExpr>> statements;
    while (!check(KXTokenType::EOF_TOKEN)) {
        statements.push_back(parseStatement());
    }
    return statements;
}

std::unique_ptr<KXExpr> KXParser::parseStatement() {
    // Function definition
    if (match(KXTokenType::FUNC)) {
        return parseFunctionDefinition();
    }
    
    // Return statement
    if (match(KXTokenType::RETURN)) {
        return parseReturnStatement();
    }
	
	//if else statements but inside parseStatement
	if (match(KXTokenType::IF)) {
        return parseIfStatement();
    }
    
    // While statement
    if (match(KXTokenType::WHILE)) {
        return parseWhileStatement();
    }
	
	//for
	if (match(KXTokenType::FOR)) {
		return parseForStatement();
	}
	
	
	//compiler or sum shit i forgot what it does
	if (match(KXTokenType::VAR)) {
        KXToken name = advance();
        if (!match(KXTokenType::EQUAL)) {
            throw std::runtime_error("Expected '=' in variable declaration");
        }
        auto value = parseExpression();
        match(KXTokenType::SEMICOLON);
        return std::make_unique<KXAssignExpr>(name.value, std::move(value));
    }
    
    if (check(KXTokenType::IDENTIFIER) && peek().value == "print") {
        advance();
        if (!match(KXTokenType::LPAREN)) {
            throw std::runtime_error("Expected '(' after print");
        }
        auto expr = parseExpression();
        if (!match(KXTokenType::RPAREN)) {
            throw std::runtime_error("Expected ')' after print expression");
        }
        match(KXTokenType::SEMICOLON);
        return std::make_unique<KXPrintExpr>(std::move(expr));
    }
    
    auto expr = parseExpression();
    match(KXTokenType::SEMICOLON);
    return expr;
}

//for function
std::unique_ptr<KXExpr> KXParser::parseForStatement() {
    // Parse variable name
    if (!check(KXTokenType::IDENTIFIER)) {
        throw std::runtime_error("Expected variable name after 'for'");
    }
    std::string varName = advance().value;
    
    // Expect 'in'
    if (!match(KXTokenType::IN)) {
        throw std::runtime_error("Expected 'in' after variable name");
    }
    
    // Expect 'range'
    if (!check(KXTokenType::IDENTIFIER) || peek().value != "range") {
        throw std::runtime_error("Expected 'range' after 'in'");
    }
    advance(); // consume 'range'
    
    if (!match(KXTokenType::LPAREN)) {
        throw std::runtime_error("Expected '(' after range");
    }
    
    // Default values
    std::unique_ptr<KXExpr> start = std::make_unique<KXNumberExpr>(0);
    std::unique_ptr<KXExpr> end = nullptr;
    
    // Parse first argument
    if (!check(KXTokenType::RPAREN)) {
        auto first = parseExpression();
        
        if (match(KXTokenType::COMMA)) {
            // Two arguments: start, end
            start = std::move(first);
            end = parseExpression();
        } else {
            // One argument: end
            end = std::move(first);
        }
    }
    
    if (!match(KXTokenType::RPAREN)) {
        throw std::runtime_error("Expected ')' after range arguments");
    }
    
    if (!end) {
        throw std::runtime_error("range() requires at least one argument");
    }
    
    // Parse body
    if (!match(KXTokenType::LBRACE)) {
        throw std::runtime_error("Expected '{' after for loop");
    }
    
    std::vector<std::unique_ptr<KXExpr>> body;
    while (!check(KXTokenType::RBRACE) && !check(KXTokenType::EOF_TOKEN)) {
        body.push_back(parseStatement());
    }
    
    if (!match(KXTokenType::RBRACE)) {
        throw std::runtime_error("Expected '}' after for body");
    }
    
    return std::make_unique<KXForExpr>(varName, std::move(start), std::move(end), std::move(body));
}

std::unique_ptr<KXExpr> KXParser::parseFunctionDefinition() {
    // Get function name
    if (!check(KXTokenType::IDENTIFIER)) {
        throw std::runtime_error("Expected function name after 'func'");
    }
    std::string funcName = advance().value;
    
    // Parse parameters
    if (!match(KXTokenType::LPAREN)) {
        throw std::runtime_error("Expected '(' after function name");
    }
    
    std::vector<std::string> parameters;
    if (!check(KXTokenType::RPAREN)) {
        // Parse first parameter
        if (!check(KXTokenType::IDENTIFIER)) {
            throw std::runtime_error("Expected parameter name");
        }
        parameters.push_back(advance().value);
        
        // Parse additional parameters
        while (match(KXTokenType::COMMA)) {
            if (!check(KXTokenType::IDENTIFIER)) {
                throw std::runtime_error("Expected parameter name after ','");
            }
            parameters.push_back(advance().value);
        }
    }
    
    if (!match(KXTokenType::RPAREN)) {
        throw std::runtime_error("Expected ')' after parameters");
    }
    
    // Parse function body
    if (!match(KXTokenType::LBRACE)) {
        throw std::runtime_error("Expected '{' before function body");
    }
    
    std::vector<std::unique_ptr<KXExpr>> body;
    while (!check(KXTokenType::RBRACE) && !check(KXTokenType::EOF_TOKEN)) {
        body.push_back(parseStatement());
    }
    
    if (!match(KXTokenType::RBRACE)) {
        throw std::runtime_error("Expected '}' after function body");
    }
    
    return std::make_unique<KXFunctionExpr>(funcName, std::move(parameters), std::move(body));
}

std::unique_ptr<KXExpr> KXParser::parseReturnStatement() {
    // Optional return value
    if (check(KXTokenType::RBRACE) || check(KXTokenType::EOF_TOKEN) || 
        check(KXTokenType::SEMICOLON)) {
        match(KXTokenType::SEMICOLON);
        return std::make_unique<KXReturnExpr>(nullptr);
    }
    
    auto value = parseExpression();
    match(KXTokenType::SEMICOLON);
    return std::make_unique<KXReturnExpr>(std::move(value));
}

std::unique_ptr<KXExpr> KXParser::parseFunctionCall(const std::string& name) {
    // Already consumed function name and '('
    std::vector<std::unique_ptr<KXExpr>> arguments;
    
    if (!check(KXTokenType::RPAREN)) {
        // Parse first argument
        arguments.push_back(parseExpression());
        
        // Parse additional arguments
        while (match(KXTokenType::COMMA)) {
            arguments.push_back(parseExpression());
        }
    }
    
    if (!match(KXTokenType::RPAREN)) {
        throw std::runtime_error("Expected ')' after arguments");
    }
    
    return std::make_unique<KXFunctionCallExpr>(name, std::move(arguments));
}

std::unique_ptr<KXExpr> KXParser::parseExpression() {
	// Property assignment ($Cube.position = value)
	if (check(KXTokenType::DOLLAR)) {
		advance(); // consume $
		if (!check(KXTokenType::IDENTIFIER)) {
			throw std::runtime_error("Expected object name after $");
		}
		std::string objName = advance().value;
		auto objRef = std::make_unique<KXObjectRefExpr>(objName);

		if (match(KXTokenType::DOT)) {
			if (!check(KXTokenType::IDENTIFIER)) {
				throw std::runtime_error("Expected property name after '.'");
			}
			std::string propName = advance().value;

			if (match(KXTokenType::EQUAL)) {
				auto value = parseExpression();
				auto assign = std::make_unique<KXAssignExpr>(objName, std::move(value));
				assign->target = std::make_unique<KXPropertyExpr>(std::move(objRef), propName);
				return assign;
			}

			// NOT an assignment - return property expression
			return std::make_unique<KXPropertyExpr>(std::move(objRef), propName);
		}

		// Just $Cube - return object ref
		return objRef;
	}
    
    if (check(KXTokenType::IDENTIFIER)) {
        KXToken name = peek();
        advance();
        
        if (match(KXTokenType::EQUAL)) {
            auto value = parseExpression();
            return std::make_unique<KXAssignExpr>(name.value, std::move(value));
        }
        if (match(KXTokenType::PLUS_EQUALS)) {
            auto value = parseExpression();
            auto assign = std::make_unique<KXAssignExpr>(name.value, std::move(value));
            assign->compound = true;
            assign->compoundOp = '+';
            return assign;
        }
		if (match(KXTokenType::MINUS_EQUALS)) {
			auto value = parseExpression();
			auto assign = std::make_unique<KXAssignExpr>(name.value, std::move(value));
			assign->compound = true;
			assign->compoundOp = '-';
			return assign;
		}
		if (match(KXTokenType::MUL_EQUALS)) {
			auto value = parseExpression();
			auto assign = std::make_unique<KXAssignExpr>(name.value, std::move(value));
			assign->compound = true;
			assign->compoundOp = '*';
			return assign;
		}
		if (match(KXTokenType::DIV_EQUALS)) {
			auto value = parseExpression();
			auto assign = std::make_unique<KXAssignExpr>(name.value, std::move(value));
			assign->compound = true;
			assign->compoundOp = '/';
			return assign;
		}
		if (match(KXTokenType::PLUS_PLUS)) {
			auto assign = std::make_unique<KXAssignExpr>(name.value, 
				std::make_unique<KXNumberExpr>(1));
			assign->compound = true;
			assign->compoundOp = '+';
			return assign;
		}
		if (match(KXTokenType::MINUS_MINUS)) {
			auto assign = std::make_unique<KXAssignExpr>(name.value, 
				std::make_unique<KXNumberExpr>(1));
			assign->compound = true;
			assign->compoundOp = '-';
			return assign;
		}
		
        current--;
    }
    
    return parseLogical();
}

std::unique_ptr<KXExpr> KXParser::parseUnary() {
    if (match(KXTokenType::MINUS)) {
        auto expr = parseUnary();
        return std::make_unique<KXBinaryExpr>('-', 
            std::make_unique<KXNumberExpr>(0), 
            std::move(expr));
    }
    if (match(KXTokenType::NOT)) {
        auto expr = parseUnary();
        return std::make_unique<KXBinaryExpr>('!', 
            std::move(expr), 
            std::make_unique<KXNumberExpr>(0));
    }
    return parseIndex();  // Changed from parsePrimary()
}

std::unique_ptr<KXExpr> KXParser::parseEquality() {
    auto expr = parseComparison();
    while (check(KXTokenType::EQUAL_EQUAL) || 
           check(KXTokenType::LESS) || 
           check(KXTokenType::GREATER) ||
           check(KXTokenType::LESS_EQUAL) ||
           check(KXTokenType::GREATER_EQUAL) ||
           check(KXTokenType::NOT_EQUAL)) {
        KXToken op = advance();
        auto right = parseComparison();
        
        // Map token to operator char
        char opChar;
        switch(op.type) {
            case KXTokenType::EQUAL_EQUAL: opChar = '='; break;
            case KXTokenType::LESS: opChar = '<'; break;
            case KXTokenType::GREATER: opChar = '>'; break;
            case KXTokenType::LESS_EQUAL: opChar = 'L'; break;  // L for <=
            case KXTokenType::GREATER_EQUAL: opChar = 'G'; break;  // G for >=
            case KXTokenType::NOT_EQUAL: opChar = 'N'; break;  // N for !=
            default: opChar = '?';
        }
        
        expr = std::make_unique<KXBinaryExpr>(opChar, std::move(expr), std::move(right));
    }
    return expr;
}

std::unique_ptr<KXExpr> KXParser::parseLogical() {
    auto expr = parseEquality();
    while (check(KXTokenType::AND_AND) || check(KXTokenType::OR_OR)) {
        KXToken op = advance();
        auto right = parseEquality();
        
        char opChar = (op.type == KXTokenType::AND_AND) ? '&' : '|';
        expr = std::make_unique<KXBinaryExpr>(opChar, std::move(expr), std::move(right));
    }
    return expr;
}

std::unique_ptr<KXExpr> KXParser::parseComparison() {
    return parseTerm();
}

std::unique_ptr<KXExpr> KXParser::parseTerm() {
    auto expr = parseFactor();
    while (check(KXTokenType::PLUS) || check(KXTokenType::MINUS)) {
        KXToken op = advance();
        auto right = parseFactor();
        expr = std::make_unique<KXBinaryExpr>(op.value[0], std::move(expr), std::move(right));
    }
    return expr;
}

std::unique_ptr<KXExpr> KXParser::parseFactor() {
    auto expr = parseUnary();
    while (check(KXTokenType::STAR) || check(KXTokenType::SLASH)) {
        KXToken op = advance();
        auto right = parsePrimary();
        expr = std::make_unique<KXBinaryExpr>(op.value[0], std::move(expr), std::move(right));
    }
    return expr;
}

std::unique_ptr<KXExpr> KXParser::parsePrimary() {
    // Object reference ($Cube)
	if (match(KXTokenType::DOLLAR)) {
		if (!check(KXTokenType::IDENTIFIER)) {
			throw std::runtime_error("Expected object name after $");
		}
		std::string objName = advance().value;
		return std::make_unique<KXObjectRefExpr>(objName);
	}
	
	if (check(KXTokenType::NUMBER)) {
        return std::make_unique<KXNumberExpr>(std::stod(advance().value));
    }
    if (check(KXTokenType::STRING)) {
        return std::make_unique<KXStringExpr>(advance().value);
    }
    if (check(KXTokenType::TRUE)) {
        advance();
        return std::make_unique<KXBoolExpr>(true);
    }
    if (check(KXTokenType::FALSE)) {
        advance();
        return std::make_unique<KXBoolExpr>(false);
    }
    if (check(KXTokenType::IDENTIFIER)) {
        std::string name = advance().value;
        
        // Check if it's a function call
        if (match(KXTokenType::LPAREN)) {
            return parseFunctionCall(name);
        }
        
        // Otherwise it's a variable
        return std::make_unique<KXVariableExpr>(name);
    }
    if (match(KXTokenType::LPAREN)) {
        auto expr = parseExpression();
        if (!match(KXTokenType::RPAREN)) {
            throw std::runtime_error("Expected ')' after expression");
        }
        return expr;
    }
	
	//brackets
	if (match(KXTokenType::LBRACKET)) {
    std::vector<std::unique_ptr<KXExpr>> elements;
    
    if (!check(KXTokenType::RBRACKET)) {
        elements.push_back(parseExpression());
        
        while (match(KXTokenType::COMMA)) {
            elements.push_back(parseExpression());
        }
    }
    
    if (!match(KXTokenType::RBRACKET)) {
        throw std::runtime_error("Expected ']' after array");
    }
    
    return std::make_unique<KXArrayExpr>(std::move(elements));
}
    
    throw std::runtime_error("Unexpected token: " + peek().value + " at line " + std::to_string(peek().line));
}

//index parser
std::unique_ptr<KXExpr> KXParser::parseIndex() {
    auto expr = parsePrimary();
    
    while (true) {
        if (match(KXTokenType::LBRACKET)) {
            auto index = parseExpression();
            if (!match(KXTokenType::RBRACKET)) {
                throw std::runtime_error("Expected ']' after index");
            }
            expr = std::make_unique<KXIndexExpr>(std::move(expr), std::move(index));
        }
        else if (match(KXTokenType::DOT)) {
            if (!check(KXTokenType::IDENTIFIER)) {
                throw std::runtime_error("Expected property name after '.'");
            }
            std::string propName = advance().value;
            expr = std::make_unique<KXPropertyExpr>(std::move(expr), propName);
        }
        else {
            break;
        }
    }
    
    return expr;
}

//if else statements
std::unique_ptr<KXExpr> KXParser::parseIfStatement() {
    // Already consumed 'if' keyword
    auto condition = parseExpression();
    
    // Parse then block
    if (!match(KXTokenType::LBRACE)) {
        throw std::runtime_error("Expected '{' after if condition");
    }
    
    std::vector<std::unique_ptr<KXExpr>> thenBody;
    while (!check(KXTokenType::RBRACE) && !check(KXTokenType::EOF_TOKEN)) {
        thenBody.push_back(parseStatement());
    }
    
    if (!match(KXTokenType::RBRACE)) {
        throw std::runtime_error("Expected '}' after if body");
    }
    
	// Check for else or else if
	std::vector<std::unique_ptr<KXExpr>> elseBody;
	if (check(KXTokenType::ELSE)) {
		advance(); // consume 'else'
    
		// Check for else if
		if (match(KXTokenType::IF)) {
			// Parse as nested if statement
			auto nestedIf = parseIfStatement();
			elseBody.push_back(std::move(nestedIf));
		} else {
			// Regular else block
			if (!match(KXTokenType::LBRACE)) {
				throw std::runtime_error("Expected '{' after else");
			}
			
			while (!check(KXTokenType::RBRACE) && !check(KXTokenType::EOF_TOKEN)) {
				elseBody.push_back(parseStatement());
			}
			
			if (!match(KXTokenType::RBRACE)) {
				throw std::runtime_error("Expected '}' after else body");
			}
		}
	}
    
    return std::make_unique<KXIfExpr>(
        std::move(condition), 
        std::move(thenBody), 
        std::move(elseBody)
    );
}

std::unique_ptr<KXExpr> KXParser::parseWhileStatement() {
    // Already consumed 'while' keyword
    auto condition = parseExpression();
    
    if (!match(KXTokenType::LBRACE)) {
        throw std::runtime_error("Expected '{' after while condition");
    }
    
    std::vector<std::unique_ptr<KXExpr>> body;
    while (!check(KXTokenType::RBRACE) && !check(KXTokenType::EOF_TOKEN)) {
        body.push_back(parseStatement());
    }
    
    if (!match(KXTokenType::RBRACE)) {
        throw std::runtime_error("Expected '}' after while body");
    }
    
    return std::make_unique<KXWhileExpr>(std::move(condition), std::move(body));
}