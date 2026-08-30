#include "KXScriptInterpreter.h"
#include <stdexcept>

#include "KXScriptInterpreter.h"
#include "BKE_global.h"
#include "BKE_main.h"
#include "DNA_object_types.h"
#include "BLI_string.h"
#include <stdexcept>
#include "BKE_depsgraph.h"  // For DAG_id_tag_update

//remove this later
#include "WM_api.h"
#include "WM_types.h"

#include <cstring>  // Add this at top

Object* KXInterpreter::findBlenderObject(const std::string& name) {
	Main *bmain = G.main;
	if (!bmain) return nullptr;

	for (Object *ob = (Object*)bmain->object.first; ob; ob = (Object*)ob->id.next) {
		if (ob->id.name[2] && strcmp(ob->id.name + 2, name.c_str()) == 0) {
			return ob;
		}
	}
	return nullptr;
}

// Helper function implementation
bool KXInterpreter::isTruthy(const KXValue& value) {
    switch(value.type) {
        case KXValue::BOOL: 
            return value.boolean;
        case KXValue::NUMBER: 
            return value.number != 0;
        case KXValue::STRING: 
            return !value.str.empty();
        case KXValue::NIL:
            return false;
        default: 
            return false;
    }
}

KXValue KXInterpreter::evaluate(KXExpr* expr) {
	// Object reference
	if (auto* objRef = dynamic_cast<KXObjectRefExpr*>(expr)) {
		Object* ob = findBlenderObject(objRef->name);
		if (ob) {
			return KXValue::object_val(ob);
		}
		throw std::runtime_error("Object not found: " + objRef->name);
	}
	
	// Property access
	if (auto* propExpr = dynamic_cast<KXPropertyExpr*>(expr)) {
		KXValue obj = evaluate(propExpr->object.get());
		
		if (obj.type == KXValue::OBJECT && obj.objectPtr) {
			Object* ob = (Object*)obj.objectPtr;
			
			if (propExpr->property == "position") {
				return KXValue::array_val({
					KXValue::num(ob->loc[0]),
					KXValue::num(ob->loc[1]),
					KXValue::num(ob->loc[2])
				});
			}
			if (propExpr->property == "rotation") {
				return KXValue::array_val({
					KXValue::num(ob->rot[0]),
					KXValue::num(ob->rot[1]),
					KXValue::num(ob->rot[2])
				});
			}
			if (propExpr->property == "scale") {
				return KXValue::array_val({
					KXValue::num(ob->size[0]),
					KXValue::num(ob->size[1]),
					KXValue::num(ob->size[2])
				});
			}
			if (propExpr->property == "visible") {
				return KXValue::bool_val(!(ob->restrictflag & OB_RESTRICT_VIEW));
			}
		}
		
		return KXValue::nil();
	}
	
	
	
	//if else statements but inside evaluate or sum shit
	if (auto* ifExpr = dynamic_cast<KXIfExpr*>(expr)) {
		KXValue cond = evaluate(ifExpr->condition.get());
    
		// Use isTruthy instead of checking BOOL directly
		if (isTruthy(cond)) {
			for (auto& stmt : ifExpr->thenBody) {
				evaluate(stmt.get());
			}
		} else if (ifExpr->elseBody.size() > 0) {
			for (auto& stmt : ifExpr->elseBody) {
				evaluate(stmt.get());
			}
		}
    return KXValue::nil();
	}
	
	//for
	if (auto* forExpr = dynamic_cast<KXForExpr*>(expr)) {
		KXValue startVal = evaluate(forExpr->start.get());
		KXValue endVal = evaluate(forExpr->end.get());
		
		int start = (int)startVal.number;
		int end = (int)endVal.number;
		
		for (int i = start; i < end; i++) {
			variables[forExpr->variable] = KXValue::num(i);
			
			for (auto& stmt : forExpr->body) {
				evaluate(stmt.get());
			}
		}
		
		return KXValue::nil();
	}
	
	//i forgot what this thing does

	if (auto* whileExpr = dynamic_cast<KXWhileExpr*>(expr)) {
		int iterations = 0;
		while (isTruthy(evaluate(whileExpr->condition.get()))) {
			for (auto& stmt : whileExpr->body) {
				evaluate(stmt.get());
			}
			
			if (++iterations > 10000) {
				std::cerr << "KXScript: Infinite loop detected" << std::endl;
				break;
			}
		}
		return KXValue::nil();
	}
	
	// Function definition
	if (auto* funcDef = dynamic_cast<KXFunctionExpr*>(expr)) {
		functions[funcDef->name] = funcDef;
		return KXValue::nil();
	}
	
	// Return statement
	if (auto* retExpr = dynamic_cast<KXReturnExpr*>(expr)) {
		if (retExpr->value) {
			return evaluate(retExpr->value.get());
		}
		return KXValue::nil();
	}

	// Built-in array functions
	if (auto* callExpr = dynamic_cast<KXFunctionCallExpr*>(expr)) {
		// len()
		if (callExpr->name == "len") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("len() expects 1 argument");
			}
			KXValue val = evaluate(callExpr->arguments[0].get());
			if (val.type == KXValue::ARRAY) {
				return KXValue::num((double)val.array.size());
			}
			if (val.type == KXValue::STRING) {
				return KXValue::num((double)val.str.length());
			}
			throw std::runtime_error("len() expects array or string");
		}

		// push()
		if (callExpr->name == "push") {
			if (callExpr->arguments.size() != 2) {
				throw std::runtime_error("push() expects 2 arguments");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			KXValue val = evaluate(callExpr->arguments[1].get());
			if (arr.type != KXValue::ARRAY) {
				throw std::runtime_error("push() expects array as first argument");
			}
			arr.array.push_back(val);
			return arr;
		}

		// pop()
		if (callExpr->name == "pop") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("pop() expects 1 argument");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			if (arr.type != KXValue::ARRAY) {
				throw std::runtime_error("pop() expects array");
			}
			if (arr.array.empty()) {
				throw std::runtime_error("pop() on empty array");
			}
			KXValue last = arr.array.back();
			arr.array.pop_back();
			return last;
		}

		// first()
		if (callExpr->name == "first") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("first() expects 1 argument");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			if (arr.type != KXValue::ARRAY || arr.array.empty()) {
				throw std::runtime_error("first() expects non-empty array");
			}
			return arr.array[0];
		}

		// last()
		if (callExpr->name == "last") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("last() expects 1 argument");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			if (arr.type != KXValue::ARRAY || arr.array.empty()) {
				throw std::runtime_error("last() expects non-empty array");
			}
			return arr.array.back();
		}

		// contains() - works for both arrays and strings
		if (callExpr->name == "contains") {
			if (callExpr->arguments.size() != 2) {
				throw std::runtime_error("contains() expects 2 arguments");
			}
			KXValue first = evaluate(callExpr->arguments[0].get());
			KXValue second = evaluate(callExpr->arguments[1].get());

			// String contains
			if (first.type == KXValue::STRING && second.type == KXValue::STRING) {
				return KXValue::bool_val(first.str.find(second.str) != std::string::npos);
			}

			// Array contains
			if (first.type == KXValue::ARRAY) {
				for (const auto& elem : first.array) {
					if (elem.type == second.type && elem.type == KXValue::NUMBER && elem.number == second.number) {
						return KXValue::bool_val(true);
					}
					if (elem.type == second.type && elem.type == KXValue::STRING && elem.str == second.str) {
						return KXValue::bool_val(true);
					}
					if (elem.type == second.type && elem.type == KXValue::BOOL && elem.boolean == second.boolean) {
						return KXValue::bool_val(true);
					}
				}
				return KXValue::bool_val(false);
			}

			throw std::runtime_error("contains() expects array or string as first argument");
		}

		// sum()
		if (callExpr->name == "sum") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("sum() expects 1 argument");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			if (arr.type != KXValue::ARRAY) {
				throw std::runtime_error("sum() expects array");
			}
			double total = 0;
			for (const auto& elem : arr.array) {
				if (elem.type == KXValue::NUMBER) {
					total += elem.number;
				}
			}
			return KXValue::num(total);
		}

		// max()
		if (callExpr->name == "max") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("max() expects 1 argument");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			if (arr.type != KXValue::ARRAY || arr.array.empty()) {
				throw std::runtime_error("max() expects non-empty array");
			}
			KXValue maxVal = arr.array[0];
			for (const auto& elem : arr.array) {
				if (elem.type == KXValue::NUMBER && elem.number > maxVal.number) {
					maxVal = elem;
				}
			}
			return maxVal;
		}

		// min()
		if (callExpr->name == "min") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("min() expects 1 argument");
			}
			KXValue arr = evaluate(callExpr->arguments[0].get());
			if (arr.type != KXValue::ARRAY || arr.array.empty()) {
				throw std::runtime_error("min() expects non-empty array");
			}
			KXValue minVal = arr.array[0];
			for (const auto& elem : arr.array) {
				if (elem.type == KXValue::NUMBER && elem.number < minVal.number) {
					minVal = elem;
				}
			}
			return minVal;
		}

		// upper()
		if (callExpr->name == "upper") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("upper() expects 1 argument");
			}
			KXValue val = evaluate(callExpr->arguments[0].get());
			if (val.type != KXValue::STRING) {
				throw std::runtime_error("upper() expects string");
			}
			std::string result = val.str;
			for (auto& c : result) {
				c = toupper(c);
			}
			return KXValue::text(result);
		}

		// lower()
		if (callExpr->name == "lower") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("lower() expects 1 argument");
			}
			KXValue val = evaluate(callExpr->arguments[0].get());
			if (val.type != KXValue::STRING) {
				throw std::runtime_error("lower() expects string");
			}
			std::string result = val.str;
			for (auto& c : result) {
				c = tolower(c);
			}
			return KXValue::text(result);
		}

		// trim()
		if (callExpr->name == "trim") {
			if (callExpr->arguments.size() != 1) {
				throw std::runtime_error("trim() expects 1 argument");
			}
			KXValue val = evaluate(callExpr->arguments[0].get());
			if (val.type != KXValue::STRING) {
				throw std::runtime_error("trim() expects string");
			}
			std::string result = val.str;
			// Remove leading spaces
			size_t start = result.find_first_not_of(" \t\n\r");
			if (start == std::string::npos) {
				return KXValue::text("");
			}
			// Remove trailing spaces
			size_t end = result.find_last_not_of(" \t\n\r");
			result = result.substr(start, end - start + 1);
			return KXValue::text(result);
		}

		// contains() for strings
		if (callExpr->name == "contains") {
			if (callExpr->arguments.size() != 2) {
				throw std::runtime_error("contains() expects 2 arguments");
			}
			KXValue first = evaluate(callExpr->arguments[0].get());
			KXValue second = evaluate(callExpr->arguments[1].get());

			// String contains
			if (first.type == KXValue::STRING && second.type == KXValue::STRING) {
				return KXValue::bool_val(first.str.find(second.str) != std::string::npos);
			}

			// Array contains (existing code)
			if (first.type == KXValue::ARRAY) {
				for (const auto& elem : first.array) {
					if (elem.type == second.type && elem.type == KXValue::NUMBER && elem.number == second.number) {
						return KXValue::bool_val(true);
					}
					if (elem.type == second.type && elem.type == KXValue::STRING && elem.str == second.str) {
						return KXValue::bool_val(true);
					}
				}
				return KXValue::bool_val(false);
			}

			throw std::runtime_error("contains() expects array or string");
		}

		// starts_with()
		if (callExpr->name == "starts_with") {
			if (callExpr->arguments.size() != 2) {
				throw std::runtime_error("starts_with() expects 2 arguments");
			}
			KXValue str = evaluate(callExpr->arguments[0].get());
			KXValue prefix = evaluate(callExpr->arguments[1].get());
			if (str.type != KXValue::STRING || prefix.type != KXValue::STRING) {
				throw std::runtime_error("starts_with() expects strings");
			}
			if (prefix.str.length() > str.str.length()) {
				return KXValue::bool_val(false);
			}
			return KXValue::bool_val(str.str.substr(0, prefix.str.length()) == prefix.str);
		}

		// ends_with()
		if (callExpr->name == "ends_with") {
			if (callExpr->arguments.size() != 2) {
				throw std::runtime_error("ends_with() expects 2 arguments");
			}
			KXValue str = evaluate(callExpr->arguments[0].get());
			KXValue suffix = evaluate(callExpr->arguments[1].get());
			if (str.type != KXValue::STRING || suffix.type != KXValue::STRING) {
				throw std::runtime_error("ends_with() expects strings");
			}
			if (suffix.str.length() > str.str.length()) {
				return KXValue::bool_val(false);
			}
			return KXValue::bool_val(str.str.substr(str.str.length() - suffix.str.length()) == suffix.str);
		}

		// replace()
		if (callExpr->name == "replace") {
			if (callExpr->arguments.size() != 3) {
				throw std::runtime_error("replace() expects 3 arguments");
			}
			KXValue str = evaluate(callExpr->arguments[0].get());
			KXValue from = evaluate(callExpr->arguments[1].get());
			KXValue to = evaluate(callExpr->arguments[2].get());
			if (str.type != KXValue::STRING || from.type != KXValue::STRING || to.type != KXValue::STRING) {
				throw std::runtime_error("replace() expects strings");
			}
			std::string result = str.str;
			size_t pos = 0;
			while ((pos = result.find(from.str, pos)) != std::string::npos) {
				result.replace(pos, from.str.length(), to.str);
				pos += to.str.length();
			}
			return KXValue::text(result);
		}
	}

	// Function call
	if (auto* callExpr = dynamic_cast<KXFunctionCallExpr*>(expr)) {
		auto it = functions.find(callExpr->name);
		if (it == functions.end()) {
			throw std::runtime_error("Undefined function: " + callExpr->name);
		}
		
		KXFunctionExpr* func = it->second;
		
		if (callExpr->arguments.size() != func->parameters.size()) {
			throw std::runtime_error("Wrong number of arguments for function: " + callExpr->name);
		}
    
		// Save old variables
		auto oldVariables = variables;
    
		// Set parameters
		for (size_t i = 0; i < func->parameters.size(); i++) {
			variables[func->parameters[i]] = evaluate(callExpr->arguments[i].get());
		}
    
		// Execute body
		KXValue returnValue = KXValue::nil();
		for (auto& stmt : func->body) {
			if (dynamic_cast<KXReturnExpr*>(stmt.get())) {
				returnValue = evaluate(stmt.get());
				break;
			}
			evaluate(stmt.get());
		}
    
		// Restore variables
		variables = oldVariables;
		
		return returnValue;
	}
	
	//array
	// Array literal
	if (auto* arrExpr = dynamic_cast<KXArrayExpr*>(expr)) {
		std::vector<KXValue> elements;
		for (auto& elem : arrExpr->elements) {
			elements.push_back(evaluate(elem.get()));
		}
		return KXValue::array_val(elements);
	}

	// Index access (reading only)
	if (auto* idxExpr = dynamic_cast<KXIndexExpr*>(expr)) {
		KXValue obj = evaluate(idxExpr->object.get());
		KXValue index = evaluate(idxExpr->index.get());
		
		if (obj.type == KXValue::ARRAY) {
			int idx = (int)index.number;
			if (idx >= 0 && idx < (int)obj.array.size()) {
				return obj.array[idx];
			}
			throw std::runtime_error("Array index out of bounds");
		}
		
		throw std::runtime_error("Cannot index non-array value");
	}
	
	
	//the rest
	
	
	
	if (auto* num = dynamic_cast<KXNumberExpr*>(expr)) {
        return KXValue::num(num->value);
    }
    
    if (auto* str = dynamic_cast<KXStringExpr*>(expr)) {
        return KXValue::text(str->value);
    }
    
    if (auto* b = dynamic_cast<KXBoolExpr*>(expr)) {
        return KXValue::bool_val(b->value);
    }
    
    if (auto* var = dynamic_cast<KXVariableExpr*>(expr)) {
        auto it = variables.find(var->name);
        if (it != variables.end()) {
            return it->second;
        }
        throw std::runtime_error("Undefined variable: " + var->name);
    }
    
    if (auto* bin = dynamic_cast<KXBinaryExpr*>(expr)) {
        KXValue left = evaluate(bin->left.get());
        KXValue right = evaluate(bin->right.get());
        
		switch(bin->op) {
			case '+':
				if (left.type == KXValue::STRING || right.type == KXValue::STRING) {
					return KXValue::text(left.str + right.str);
				}
				return KXValue::num(left.number + right.number);
			case '-': 
				return KXValue::num(left.number - right.number);
			case '*': 
				return KXValue::num(left.number * right.number);
			case '/': 
				return KXValue::num(left.number / right.number);
			case '<': 
				return KXValue::bool_val(left.number < right.number);
			case '>': 
				return KXValue::bool_val(left.number > right.number);
			case '=': 
				return KXValue::bool_val(left.number == right.number);
			case 'L':  // <=
				return KXValue::bool_val(left.number <= right.number);
			case 'G':  // >=
				return KXValue::bool_val(left.number >= right.number);
			case 'N':  // !=
				return KXValue::bool_val(left.number != right.number);
			case '&':  // AND
				return KXValue::bool_val(isTruthy(left) && isTruthy(right));
			case '|':  // OR
				return KXValue::bool_val(isTruthy(left) || isTruthy(right));
			case '!':  // NOT
				return KXValue::bool_val(!isTruthy(left));
		}
    }
    
	if (auto* assign = dynamic_cast<KXAssignExpr*>(expr)) {
        KXValue value = evaluate(assign->value.get());
        
        // Property assignment ($Cube.position = value)
        if (assign->target) {
            if (auto* propExpr = dynamic_cast<KXPropertyExpr*>(assign->target.get())) {
                KXValue obj = evaluate(propExpr->object.get());
                
                if (obj.type == KXValue::OBJECT && obj.objectPtr) {
                    Object* ob = (Object*)obj.objectPtr;
                    
                    if (propExpr->property == "position" && value.type == KXValue::ARRAY && value.array.size() >= 3) {
                        ob->loc[0] = value.array[0].number;
                        ob->loc[1] = value.array[1].number;
                        ob->loc[2] = value.array[2].number;
						DAG_id_tag_update(&ob->id, OB_RECALC_OB);
                        return value;
                    }
                    if (propExpr->property == "rotation" && value.type == KXValue::ARRAY && value.array.size() >= 3) {
                        ob->rot[0] = value.array[0].number;
                        ob->rot[1] = value.array[1].number;
                        ob->rot[2] = value.array[2].number;
						DAG_id_tag_update(&ob->id, OB_RECALC_OB);
                        return value;
                    }
                    if (propExpr->property == "scale" && value.type == KXValue::ARRAY && value.array.size() >= 3) {
                        ob->size[0] = value.array[0].number;
                        ob->size[1] = value.array[1].number;
                        ob->size[2] = value.array[2].number;
						DAG_id_tag_update(&ob->id, OB_RECALC_OB);
                        return value;
                    }
                    if (propExpr->property == "visible" && value.type == KXValue::BOOL) {
                        if (value.boolean) {
                            ob->restrictflag &= ~OB_RESTRICT_VIEW;
                        } else {
                            ob->restrictflag |= OB_RESTRICT_VIEW;
                        }
						DAG_id_tag_update(&ob->id, OB_RECALC_OB);
						WM_main_add_notifier(NC_OBJECT | ND_TRANSFORM, NULL);
						return value;
                    }
                }
            }
        }
        
        // Regular variable assignment (with compound support)
        if (assign->compound) {
            auto it = variables.find(assign->name);
            if (it != variables.end()) {
                KXValue current = it->second;
                switch(assign->compoundOp) {
					case '+': 
						value = KXValue::num(current.number + value.number); 
						break;
					case '-': 
						value = KXValue::num(current.number - value.number); 
						break;
					case '*': 
						value = KXValue::num(current.number * value.number); 
						break;
					case '/': 
						value = KXValue::num(current.number / value.number); 
						break;
				}
            }
        }
        
        variables[assign->name] = value;
        return value;
    }
    
    if (auto* print = dynamic_cast<KXPrintExpr*>(expr)) {
        KXValue value = evaluate(print->expr.get());
        value.print();
        std::cout << std::endl;
        return value;
    }
    
    throw std::runtime_error("Unknown expression type");
}

void KXInterpreter::execute(const std::vector<std::unique_ptr<KXExpr>>& program) {
    for (const auto& expr : program) {
        evaluate(expr.get());
    }
}