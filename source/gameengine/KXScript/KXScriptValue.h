#pragma once

#include <string>
#include <iostream>
#include <vector>

// Forward declare Blender Object
struct Object;

class KXValue {
public:
    enum Type { NIL, NUMBER, STRING, BOOL, ARRAY, OBJECT };
    
    Type type = NIL;
    double number = 0;
    std::string str;
    bool boolean = false;
    std::vector<KXValue> array;
    Object* objectPtr = nullptr;  // For Blender objects
    
    static KXValue nil() { return KXValue(); }
    static KXValue num(double n) { KXValue v; v.type = NUMBER; v.number = n; return v; }
    static KXValue text(const std::string& s) { KXValue v; v.type = STRING; v.str = s; return v; }
    static KXValue bool_val(bool b) { KXValue v; v.type = BOOL; v.boolean = b; return v; }
    static KXValue array_val(const std::vector<KXValue>& arr) {
        KXValue v;
        v.type = ARRAY;
        v.array = arr;
        return v;
    }
    static KXValue object_val(Object* obj) {
        KXValue v;
        v.type = OBJECT;
        v.objectPtr = obj;
        return v;
    }
    
    void print() const {
        switch(type) {
            case NIL: std::cout << "nil"; break;
            case NUMBER: std::cout << number; break;
            case STRING: std::cout << str; break;
            case BOOL: std::cout << (boolean ? "true" : "false"); break;
            case ARRAY:
                std::cout << "[";
                for (size_t i = 0; i < array.size(); i++) {
                    if (i > 0) std::cout << ", ";
                    array[i].print();
                }
                std::cout << "]";
                break;
            case OBJECT:
                std::cout << "<object>";
                break;
        }
    }
};