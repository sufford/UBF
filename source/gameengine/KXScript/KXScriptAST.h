#pragma once

#include <memory>
#include <string>
#include <vector>

struct KXExpr {
    virtual ~KXExpr() = default;
};

struct KXNumberExpr : KXExpr {
    double value;
    KXNumberExpr(double v) : value(v) {}
};

struct KXStringExpr : KXExpr {
    std::string value;
    KXStringExpr(const std::string& s) : value(s) {}
};

struct KXBoolExpr : KXExpr {
    bool value;
    KXBoolExpr(bool b) : value(b) {}
};

struct KXVariableExpr : KXExpr {
    std::string name;
    KXVariableExpr(const std::string& n) : name(n) {}
};

struct KXBinaryExpr : KXExpr {
    char op;
    std::unique_ptr<KXExpr> left, right;
    KXBinaryExpr(char o, std::unique_ptr<KXExpr> l, std::unique_ptr<KXExpr> r)
        : op(o), left(std::move(l)), right(std::move(r)) {}
};

struct KXAssignExpr : KXExpr {
    std::string name;
    std::unique_ptr<KXExpr> value;
	std::unique_ptr<KXExpr> target;
    bool compound = false;
    char compoundOp;
    KXAssignExpr(const std::string& n, std::unique_ptr<KXExpr> v)
        : name(n), value(std::move(v)) {}
};

struct KXPrintExpr : KXExpr {
    std::unique_ptr<KXExpr> expr;
    KXPrintExpr(std::unique_ptr<KXExpr> e) : expr(std::move(e)) {}
};

//if else statements
struct KXIfExpr : KXExpr {
    std::unique_ptr<KXExpr> condition;
    std::vector<std::unique_ptr<KXExpr>> thenBody;
    std::vector<std::unique_ptr<KXExpr>> elseBody;
    
    KXIfExpr(std::unique_ptr<KXExpr> cond,
             std::vector<std::unique_ptr<KXExpr>> thenB,
             std::vector<std::unique_ptr<KXExpr>> elseB = {})
        : condition(std::move(cond)), 
          thenBody(std::move(thenB)), 
          elseBody(std::move(elseB)) {}
};

struct KXWhileExpr : KXExpr {
    std::unique_ptr<KXExpr> condition;
    std::vector<std::unique_ptr<KXExpr>> body;
    
    KXWhileExpr(std::unique_ptr<KXExpr> cond,
                std::vector<std::unique_ptr<KXExpr>> b)
        : condition(std::move(cond)), body(std::move(b)) {}
};

//functions
struct KXFunctionExpr : KXExpr {
    std::string name;
    std::vector<std::string> parameters;
    std::vector<std::unique_ptr<KXExpr>> body;
    
    KXFunctionExpr(const std::string& n, 
                   std::vector<std::string> params,
                   std::vector<std::unique_ptr<KXExpr>> b)
        : name(n), parameters(std::move(params)), body(std::move(b)) {}
};

struct KXReturnExpr : KXExpr {
    std::unique_ptr<KXExpr> value;
    
    KXReturnExpr(std::unique_ptr<KXExpr> v) : value(std::move(v)) {}
};

struct KXFunctionCallExpr : KXExpr {
    std::string name;
    std::vector<std::unique_ptr<KXExpr>> arguments;
    
    KXFunctionCallExpr(const std::string& n, 
                       std::vector<std::unique_ptr<KXExpr>> args)
        : name(n), arguments(std::move(args)) {}
};

//for
struct KXForExpr : KXExpr {
    std::string variable;
    std::unique_ptr<KXExpr> start;
    std::unique_ptr<KXExpr> end;
    std::vector<std::unique_ptr<KXExpr>> body;
    
    KXForExpr(const std::string& var,
              std::unique_ptr<KXExpr> s,
              std::unique_ptr<KXExpr> e,
              std::vector<std::unique_ptr<KXExpr>> b)
        : variable(var), start(std::move(s)), end(std::move(e)), body(std::move(b)) {}
};

//array lists
struct KXArrayExpr : KXExpr {
    std::vector<std::unique_ptr<KXExpr>> elements;
    KXArrayExpr(std::vector<std::unique_ptr<KXExpr>> elems) 
        : elements(std::move(elems)) {}
};

struct KXIndexExpr : KXExpr {
    std::unique_ptr<KXExpr> object;
    std::unique_ptr<KXExpr> index;
    KXIndexExpr(std::unique_ptr<KXExpr> obj, std::unique_ptr<KXExpr> idx)
        : object(std::move(obj)), index(std::move(idx)) {}
};

//object identitation
struct KXObjectRefExpr : KXExpr {
    std::string name;
    KXObjectRefExpr(const std::string& n) : name(n) {}
};

struct KXPropertyExpr : KXExpr {
    std::unique_ptr<KXExpr> object;
    std::string property;
    KXPropertyExpr(std::unique_ptr<KXExpr> obj, const std::string& prop)
        : object(std::move(obj)), property(prop) {}
};