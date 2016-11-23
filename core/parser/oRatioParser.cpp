
// Generated from oRatio.g4 by ANTLR 4.5.3


#include "oRatioListener.h"
#include "oRatioVisitor.h"

#include "oRatioParser.h"


using namespace antlrcpp;
using namespace oratio;
using namespace antlr4;

oRatioParser::oRatioParser(TokenStream *input) : Parser(input) {
    _interpreter = new atn::ParserATNSimulator(this, _atn, _decisionToDFA, _sharedContextCache);
}

oRatioParser::~oRatioParser() {
    delete _interpreter;
}

std::string oRatioParser::getGrammarFileName() const {
    return "oRatio.g4";
}

const std::vector<std::string>& oRatioParser::getRuleNames() const {
    return _ruleNames;
}

dfa::Vocabulary& oRatioParser::getVocabulary() const {
    return _vocabulary;
}


//----------------- Compilation_unitContext ------------------------------------------------------------------

oRatioParser::Compilation_unitContext::Compilation_unitContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::Compilation_unitContext::EOF() {
    return getToken(oRatioParser::EOF, 0);
}

std::vector<oRatioParser::Type_declarationContext *> oRatioParser::Compilation_unitContext::type_declaration() {
    return getRuleContexts<oRatioParser::Type_declarationContext>();
}

oRatioParser::Type_declarationContext* oRatioParser::Compilation_unitContext::type_declaration(size_t i) {
    return getRuleContext<oRatioParser::Type_declarationContext>(i);
}

std::vector<oRatioParser::Method_declarationContext *> oRatioParser::Compilation_unitContext::method_declaration() {
    return getRuleContexts<oRatioParser::Method_declarationContext>();
}

oRatioParser::Method_declarationContext* oRatioParser::Compilation_unitContext::method_declaration(size_t i) {
    return getRuleContext<oRatioParser::Method_declarationContext>(i);
}

std::vector<oRatioParser::Predicate_declarationContext *> oRatioParser::Compilation_unitContext::predicate_declaration() {
    return getRuleContexts<oRatioParser::Predicate_declarationContext>();
}

oRatioParser::Predicate_declarationContext* oRatioParser::Compilation_unitContext::predicate_declaration(size_t i) {
    return getRuleContext<oRatioParser::Predicate_declarationContext>(i);
}

std::vector<oRatioParser::StatementContext *> oRatioParser::Compilation_unitContext::statement() {
    return getRuleContexts<oRatioParser::StatementContext>();
}

oRatioParser::StatementContext* oRatioParser::Compilation_unitContext::statement(size_t i) {
    return getRuleContext<oRatioParser::StatementContext>(i);
}

size_t oRatioParser::Compilation_unitContext::getRuleIndex() const {
    return oRatioParser::RuleCompilation_unit;
}

void oRatioParser::Compilation_unitContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterCompilation_unit(this);
}

void oRatioParser::Compilation_unitContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitCompilation_unit(this);
}

antlrcpp::Any oRatioParser::Compilation_unitContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitCompilation_unit(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Compilation_unitContext* oRatioParser::compilation_unit() {
    Compilation_unitContext *_localctx = _tracker.createInstance<Compilation_unitContext>(_ctx, getState());
    enterRule(_localctx, 0, oRatioParser::RuleCompilation_unit);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(72);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while ((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::TYPE_DEF)
                | (1ULL << oRatioParser::REAL)
                | (1ULL << oRatioParser::BOOL)
                | (1ULL << oRatioParser::STRING)
                | (1ULL << oRatioParser::ENUM)
                | (1ULL << oRatioParser::CLASS)
                | (1ULL << oRatioParser::GOAL)
                | (1ULL << oRatioParser::FACT)
                | (1ULL << oRatioParser::PREDICATE)
                | (1ULL << oRatioParser::NEW)
                | (1ULL << oRatioParser::THIS)
                | (1ULL << oRatioParser::VOID)
                | (1ULL << oRatioParser::TRUE)
                | (1ULL << oRatioParser::FALSE)
                | (1ULL << oRatioParser::RETURN)
                | (1ULL << oRatioParser::LPAREN)
                | (1ULL << oRatioParser::LBRACKET)
                | (1ULL << oRatioParser::LBRACE)
                | (1ULL << oRatioParser::PLUS)
                | (1ULL << oRatioParser::MINUS)
                | (1ULL << oRatioParser::BANG)
                | (1ULL << oRatioParser::ID)
                | (1ULL << oRatioParser::NumericLiteral)
                | (1ULL << oRatioParser::StringLiteral))) != 0)) {
            setState(70);
            _errHandler->sync(this);
            switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 0, _ctx)) {
                case 1:
                {
                    setState(66);
                    type_declaration();
                    break;
                }

                case 2:
                {
                    setState(67);
                    method_declaration();
                    break;
                }

                case 3:
                {
                    setState(68);
                    predicate_declaration();
                    break;
                }

                case 4:
                {
                    setState(69);
                    statement();
                    break;
                }

            }
            setState(74);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }
        setState(75);
        match(oRatioParser::EOF);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Type_declarationContext ------------------------------------------------------------------

oRatioParser::Type_declarationContext::Type_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::Typedef_declarationContext* oRatioParser::Type_declarationContext::typedef_declaration() {
    return getRuleContext<oRatioParser::Typedef_declarationContext>(0);
}

oRatioParser::Enum_declarationContext* oRatioParser::Type_declarationContext::enum_declaration() {
    return getRuleContext<oRatioParser::Enum_declarationContext>(0);
}

oRatioParser::Class_declarationContext* oRatioParser::Type_declarationContext::class_declaration() {
    return getRuleContext<oRatioParser::Class_declarationContext>(0);
}

size_t oRatioParser::Type_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleType_declaration;
}

void oRatioParser::Type_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterType_declaration(this);
}

void oRatioParser::Type_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitType_declaration(this);
}

antlrcpp::Any oRatioParser::Type_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitType_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Type_declarationContext* oRatioParser::type_declaration() {
    Type_declarationContext *_localctx = _tracker.createInstance<Type_declarationContext>(_ctx, getState());
    enterRule(_localctx, 2, oRatioParser::RuleType_declaration);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(80);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::TYPE_DEF:
            {
                enterOuterAlt(_localctx, 1);
                setState(77);
                typedef_declaration();
                break;
            }

            case oRatioParser::ENUM:
            {
                enterOuterAlt(_localctx, 2);
                setState(78);
                enum_declaration();
                break;
            }

            case oRatioParser::CLASS:
            {
                enterOuterAlt(_localctx, 3);
                setState(79);
                class_declaration();
                break;
            }

            default:
                throw NoViableAltException(this);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Typedef_declarationContext ------------------------------------------------------------------

oRatioParser::Typedef_declarationContext::Typedef_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::Primitive_typeContext* oRatioParser::Typedef_declarationContext::primitive_type() {
    return getRuleContext<oRatioParser::Primitive_typeContext>(0);
}

oRatioParser::ExprContext* oRatioParser::Typedef_declarationContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

tree::TerminalNode* oRatioParser::Typedef_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

size_t oRatioParser::Typedef_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleTypedef_declaration;
}

void oRatioParser::Typedef_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterTypedef_declaration(this);
}

void oRatioParser::Typedef_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitTypedef_declaration(this);
}

antlrcpp::Any oRatioParser::Typedef_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitTypedef_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Typedef_declarationContext* oRatioParser::typedef_declaration() {
    Typedef_declarationContext *_localctx = _tracker.createInstance<Typedef_declarationContext>(_ctx, getState());
    enterRule(_localctx, 4, oRatioParser::RuleTypedef_declaration);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(82);
        match(oRatioParser::TYPE_DEF);
        setState(83);
        primitive_type();
        setState(84);
        expr(0);
        setState(85);
        dynamic_cast<Typedef_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(86);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Enum_declarationContext ------------------------------------------------------------------

oRatioParser::Enum_declarationContext::Enum_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::Enum_constantsContext *> oRatioParser::Enum_declarationContext::enum_constants() {
    return getRuleContexts<oRatioParser::Enum_constantsContext>();
}

oRatioParser::Enum_constantsContext* oRatioParser::Enum_declarationContext::enum_constants(size_t i) {
    return getRuleContext<oRatioParser::Enum_constantsContext>(i);
}

tree::TerminalNode* oRatioParser::Enum_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

size_t oRatioParser::Enum_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleEnum_declaration;
}

void oRatioParser::Enum_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterEnum_declaration(this);
}

void oRatioParser::Enum_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitEnum_declaration(this);
}

antlrcpp::Any oRatioParser::Enum_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitEnum_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Enum_declarationContext* oRatioParser::enum_declaration() {
    Enum_declarationContext *_localctx = _tracker.createInstance<Enum_declarationContext>(_ctx, getState());
    enterRule(_localctx, 6, oRatioParser::RuleEnum_declaration);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(88);
        match(oRatioParser::ENUM);
        setState(89);
        dynamic_cast<Enum_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(90);
        enum_constants();
        setState(95);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::BAR) {
            setState(91);
            match(oRatioParser::BAR);
            setState(92);
            enum_constants();
            setState(97);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }
        setState(98);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Enum_constantsContext ------------------------------------------------------------------

oRatioParser::Enum_constantsContext::Enum_constantsContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<tree::TerminalNode *> oRatioParser::Enum_constantsContext::StringLiteral() {
    return getTokens(oRatioParser::StringLiteral);
}

tree::TerminalNode* oRatioParser::Enum_constantsContext::StringLiteral(size_t i) {
    return getToken(oRatioParser::StringLiteral, i);
}

oRatioParser::TypeContext* oRatioParser::Enum_constantsContext::type() {
    return getRuleContext<oRatioParser::TypeContext>(0);
}

size_t oRatioParser::Enum_constantsContext::getRuleIndex() const {
    return oRatioParser::RuleEnum_constants;
}

void oRatioParser::Enum_constantsContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterEnum_constants(this);
}

void oRatioParser::Enum_constantsContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitEnum_constants(this);
}

antlrcpp::Any oRatioParser::Enum_constantsContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitEnum_constants(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Enum_constantsContext* oRatioParser::enum_constants() {
    Enum_constantsContext *_localctx = _tracker.createInstance<Enum_constantsContext>(_ctx, getState());
    enterRule(_localctx, 8, oRatioParser::RuleEnum_constants);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(111);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::LBRACE:
            {
                enterOuterAlt(_localctx, 1);
                setState(100);
                match(oRatioParser::LBRACE);
                setState(101);
                match(oRatioParser::StringLiteral);
                setState(106);
                _errHandler->sync(this);
                _la = _input->LA(1);
                while (_la == oRatioParser::COMMA) {
                    setState(102);
                    match(oRatioParser::COMMA);
                    setState(103);
                    match(oRatioParser::StringLiteral);
                    setState(108);
                    _errHandler->sync(this);
                    _la = _input->LA(1);
                }
                setState(109);
                match(oRatioParser::RBRACE);
                break;
            }

            case oRatioParser::REAL:
            case oRatioParser::BOOL:
            case oRatioParser::STRING:
            case oRatioParser::ID:
            {
                enterOuterAlt(_localctx, 2);
                setState(110);
                type();
                break;
            }

            default:
                throw NoViableAltException(this);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Class_declarationContext ------------------------------------------------------------------

oRatioParser::Class_declarationContext::Class_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::Class_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Type_listContext* oRatioParser::Class_declarationContext::type_list() {
    return getRuleContext<oRatioParser::Type_listContext>(0);
}

std::vector<oRatioParser::MemberContext *> oRatioParser::Class_declarationContext::member() {
    return getRuleContexts<oRatioParser::MemberContext>();
}

oRatioParser::MemberContext* oRatioParser::Class_declarationContext::member(size_t i) {
    return getRuleContext<oRatioParser::MemberContext>(i);
}

size_t oRatioParser::Class_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleClass_declaration;
}

void oRatioParser::Class_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterClass_declaration(this);
}

void oRatioParser::Class_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitClass_declaration(this);
}

antlrcpp::Any oRatioParser::Class_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitClass_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Class_declarationContext* oRatioParser::class_declaration() {
    Class_declarationContext *_localctx = _tracker.createInstance<Class_declarationContext>(_ctx, getState());
    enterRule(_localctx, 10, oRatioParser::RuleClass_declaration);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(113);
        match(oRatioParser::CLASS);
        setState(114);
        dynamic_cast<Class_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(117);

        _la = _input->LA(1);
        if (_la == oRatioParser::COLON) {
            setState(115);
            match(oRatioParser::COLON);
            setState(116);
            type_list();
        }
        setState(119);
        match(oRatioParser::LBRACE);
        setState(123);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while ((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::TYPE_DEF)
                | (1ULL << oRatioParser::REAL)
                | (1ULL << oRatioParser::BOOL)
                | (1ULL << oRatioParser::STRING)
                | (1ULL << oRatioParser::ENUM)
                | (1ULL << oRatioParser::CLASS)
                | (1ULL << oRatioParser::PREDICATE)
                | (1ULL << oRatioParser::VOID)
                | (1ULL << oRatioParser::ID))) != 0)) {
            setState(120);
            member();
            setState(125);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }
        setState(126);
        match(oRatioParser::RBRACE);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- MemberContext ------------------------------------------------------------------

oRatioParser::MemberContext::MemberContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::Field_declarationContext* oRatioParser::MemberContext::field_declaration() {
    return getRuleContext<oRatioParser::Field_declarationContext>(0);
}

oRatioParser::Method_declarationContext* oRatioParser::MemberContext::method_declaration() {
    return getRuleContext<oRatioParser::Method_declarationContext>(0);
}

oRatioParser::Constructor_declarationContext* oRatioParser::MemberContext::constructor_declaration() {
    return getRuleContext<oRatioParser::Constructor_declarationContext>(0);
}

oRatioParser::Predicate_declarationContext* oRatioParser::MemberContext::predicate_declaration() {
    return getRuleContext<oRatioParser::Predicate_declarationContext>(0);
}

oRatioParser::Type_declarationContext* oRatioParser::MemberContext::type_declaration() {
    return getRuleContext<oRatioParser::Type_declarationContext>(0);
}

size_t oRatioParser::MemberContext::getRuleIndex() const {
    return oRatioParser::RuleMember;
}

void oRatioParser::MemberContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterMember(this);
}

void oRatioParser::MemberContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitMember(this);
}

antlrcpp::Any oRatioParser::MemberContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitMember(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::MemberContext* oRatioParser::member() {
    MemberContext *_localctx = _tracker.createInstance<MemberContext>(_ctx, getState());
    enterRule(_localctx, 12, oRatioParser::RuleMember);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(133);
        _errHandler->sync(this);
        switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 8, _ctx)) {
            case 1:
            {
                enterOuterAlt(_localctx, 1);
                setState(128);
                field_declaration();
                break;
            }

            case 2:
            {
                enterOuterAlt(_localctx, 2);
                setState(129);
                method_declaration();
                break;
            }

            case 3:
            {
                enterOuterAlt(_localctx, 3);
                setState(130);
                constructor_declaration();
                break;
            }

            case 4:
            {
                enterOuterAlt(_localctx, 4);
                setState(131);
                predicate_declaration();
                break;
            }

            case 5:
            {
                enterOuterAlt(_localctx, 5);
                setState(132);
                type_declaration();
                break;
            }

        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Field_declarationContext ------------------------------------------------------------------

oRatioParser::Field_declarationContext::Field_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::TypeContext* oRatioParser::Field_declarationContext::type() {
    return getRuleContext<oRatioParser::TypeContext>(0);
}

std::vector<oRatioParser::Variable_decContext *> oRatioParser::Field_declarationContext::variable_dec() {
    return getRuleContexts<oRatioParser::Variable_decContext>();
}

oRatioParser::Variable_decContext* oRatioParser::Field_declarationContext::variable_dec(size_t i) {
    return getRuleContext<oRatioParser::Variable_decContext>(i);
}

size_t oRatioParser::Field_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleField_declaration;
}

void oRatioParser::Field_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterField_declaration(this);
}

void oRatioParser::Field_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitField_declaration(this);
}

antlrcpp::Any oRatioParser::Field_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitField_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Field_declarationContext* oRatioParser::field_declaration() {
    Field_declarationContext *_localctx = _tracker.createInstance<Field_declarationContext>(_ctx, getState());
    enterRule(_localctx, 14, oRatioParser::RuleField_declaration);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(135);
        type();
        setState(136);
        variable_dec();
        setState(141);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::COMMA) {
            setState(137);
            match(oRatioParser::COMMA);
            setState(138);
            variable_dec();
            setState(143);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }
        setState(144);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Variable_decContext ------------------------------------------------------------------

oRatioParser::Variable_decContext::Variable_decContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::Variable_decContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::ExprContext* oRatioParser::Variable_decContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

size_t oRatioParser::Variable_decContext::getRuleIndex() const {
    return oRatioParser::RuleVariable_dec;
}

void oRatioParser::Variable_decContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterVariable_dec(this);
}

void oRatioParser::Variable_decContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitVariable_dec(this);
}

antlrcpp::Any oRatioParser::Variable_decContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitVariable_dec(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Variable_decContext* oRatioParser::variable_dec() {
    Variable_decContext *_localctx = _tracker.createInstance<Variable_decContext>(_ctx, getState());
    enterRule(_localctx, 16, oRatioParser::RuleVariable_dec);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(146);
        dynamic_cast<Variable_decContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(149);

        _la = _input->LA(1);
        if (_la == oRatioParser::EQUAL) {
            setState(147);
            match(oRatioParser::EQUAL);
            setState(148);
            expr(0);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Method_declarationContext ------------------------------------------------------------------

oRatioParser::Method_declarationContext::Method_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

size_t oRatioParser::Method_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleMethod_declaration;
}

void oRatioParser::Method_declarationContext::copyFrom(Method_declarationContext *ctx) {
    ParserRuleContext::copyFrom(ctx);
}

//----------------- Void_method_declarationContext ------------------------------------------------------------------

oRatioParser::BlockContext* oRatioParser::Void_method_declarationContext::block() {
    return getRuleContext<oRatioParser::BlockContext>(0);
}

tree::TerminalNode* oRatioParser::Void_method_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Typed_listContext* oRatioParser::Void_method_declarationContext::typed_list() {
    return getRuleContext<oRatioParser::Typed_listContext>(0);
}

oRatioParser::Void_method_declarationContext::Void_method_declarationContext(Method_declarationContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Void_method_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterVoid_method_declaration(this);
}

void oRatioParser::Void_method_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitVoid_method_declaration(this);
}

antlrcpp::Any oRatioParser::Void_method_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitVoid_method_declaration(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Type_method_declarationContext ------------------------------------------------------------------

oRatioParser::TypeContext* oRatioParser::Type_method_declarationContext::type() {
    return getRuleContext<oRatioParser::TypeContext>(0);
}

oRatioParser::BlockContext* oRatioParser::Type_method_declarationContext::block() {
    return getRuleContext<oRatioParser::BlockContext>(0);
}

tree::TerminalNode* oRatioParser::Type_method_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Typed_listContext* oRatioParser::Type_method_declarationContext::typed_list() {
    return getRuleContext<oRatioParser::Typed_listContext>(0);
}

oRatioParser::Type_method_declarationContext::Type_method_declarationContext(Method_declarationContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Type_method_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterType_method_declaration(this);
}

void oRatioParser::Type_method_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitType_method_declaration(this);
}

antlrcpp::Any oRatioParser::Type_method_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitType_method_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Method_declarationContext* oRatioParser::method_declaration() {
    Method_declarationContext *_localctx = _tracker.createInstance<Method_declarationContext>(_ctx, getState());
    enterRule(_localctx, 18, oRatioParser::RuleMethod_declaration);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(173);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::VOID:
            {
                _localctx = dynamic_cast<Method_declarationContext *> (_tracker.createInstance<oRatioParser::Void_method_declarationContext>(_localctx));
                enterOuterAlt(_localctx, 1);
                setState(151);
                match(oRatioParser::VOID);
                setState(152);
                dynamic_cast<Void_method_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
                setState(153);
                match(oRatioParser::LPAREN);
                setState(155);

                _la = _input->LA(1);
                if ((((_la & ~0x3fULL) == 0) &&
                        ((1ULL << _la) & ((1ULL << oRatioParser::REAL)
                        | (1ULL << oRatioParser::BOOL)
                        | (1ULL << oRatioParser::STRING)
                        | (1ULL << oRatioParser::ID))) != 0)) {
                    setState(154);
                    typed_list();
                }
                setState(157);
                match(oRatioParser::RPAREN);
                setState(158);
                match(oRatioParser::LBRACE);
                setState(159);
                block();
                setState(160);
                match(oRatioParser::RBRACE);
                break;
            }

            case oRatioParser::REAL:
            case oRatioParser::BOOL:
            case oRatioParser::STRING:
            case oRatioParser::ID:
            {
                _localctx = dynamic_cast<Method_declarationContext *> (_tracker.createInstance<oRatioParser::Type_method_declarationContext>(_localctx));
                enterOuterAlt(_localctx, 2);
                setState(162);
                type();
                setState(163);
                dynamic_cast<Type_method_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
                setState(164);
                match(oRatioParser::LPAREN);
                setState(166);

                _la = _input->LA(1);
                if ((((_la & ~0x3fULL) == 0) &&
                        ((1ULL << _la) & ((1ULL << oRatioParser::REAL)
                        | (1ULL << oRatioParser::BOOL)
                        | (1ULL << oRatioParser::STRING)
                        | (1ULL << oRatioParser::ID))) != 0)) {
                    setState(165);
                    typed_list();
                }
                setState(168);
                match(oRatioParser::RPAREN);
                setState(169);
                match(oRatioParser::LBRACE);
                setState(170);
                block();
                setState(171);
                match(oRatioParser::RBRACE);
                break;
            }

            default:
                throw NoViableAltException(this);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Constructor_declarationContext ------------------------------------------------------------------

oRatioParser::Constructor_declarationContext::Constructor_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::BlockContext* oRatioParser::Constructor_declarationContext::block() {
    return getRuleContext<oRatioParser::BlockContext>(0);
}

tree::TerminalNode* oRatioParser::Constructor_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Typed_listContext* oRatioParser::Constructor_declarationContext::typed_list() {
    return getRuleContext<oRatioParser::Typed_listContext>(0);
}

std::vector<oRatioParser::Initializer_elementContext *> oRatioParser::Constructor_declarationContext::initializer_element() {
    return getRuleContexts<oRatioParser::Initializer_elementContext>();
}

oRatioParser::Initializer_elementContext* oRatioParser::Constructor_declarationContext::initializer_element(size_t i) {
    return getRuleContext<oRatioParser::Initializer_elementContext>(i);
}

size_t oRatioParser::Constructor_declarationContext::getRuleIndex() const {
    return oRatioParser::RuleConstructor_declaration;
}

void oRatioParser::Constructor_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterConstructor_declaration(this);
}

void oRatioParser::Constructor_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitConstructor_declaration(this);
}

antlrcpp::Any oRatioParser::Constructor_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitConstructor_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Constructor_declarationContext* oRatioParser::constructor_declaration() {
    Constructor_declarationContext *_localctx = _tracker.createInstance<Constructor_declarationContext>(_ctx, getState());
    enterRule(_localctx, 20, oRatioParser::RuleConstructor_declaration);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(175);
        dynamic_cast<Constructor_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(176);
        match(oRatioParser::LPAREN);
        setState(178);

        _la = _input->LA(1);
        if ((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::REAL)
                | (1ULL << oRatioParser::BOOL)
                | (1ULL << oRatioParser::STRING)
                | (1ULL << oRatioParser::ID))) != 0)) {
            setState(177);
            typed_list();
        }
        setState(180);
        match(oRatioParser::RPAREN);
        setState(190);

        _la = _input->LA(1);
        if (_la == oRatioParser::COLON) {
            setState(181);
            match(oRatioParser::COLON);
            setState(182);
            initializer_element();
            setState(187);
            _errHandler->sync(this);
            _la = _input->LA(1);
            while (_la == oRatioParser::COMMA) {
                setState(183);
                match(oRatioParser::COMMA);
                setState(184);
                initializer_element();
                setState(189);
                _errHandler->sync(this);
                _la = _input->LA(1);
            }
        }
        setState(192);
        match(oRatioParser::LBRACE);
        setState(193);
        block();
        setState(194);
        match(oRatioParser::RBRACE);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Initializer_elementContext ------------------------------------------------------------------

oRatioParser::Initializer_elementContext::Initializer_elementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::Initializer_elementContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Expr_listContext* oRatioParser::Initializer_elementContext::expr_list() {
    return getRuleContext<oRatioParser::Expr_listContext>(0);
}

size_t oRatioParser::Initializer_elementContext::getRuleIndex() const {
    return oRatioParser::RuleInitializer_element;
}

void oRatioParser::Initializer_elementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterInitializer_element(this);
}

void oRatioParser::Initializer_elementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitInitializer_element(this);
}

antlrcpp::Any oRatioParser::Initializer_elementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitInitializer_element(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Initializer_elementContext* oRatioParser::initializer_element() {
    Initializer_elementContext *_localctx = _tracker.createInstance<Initializer_elementContext>(_ctx, getState());
    enterRule(_localctx, 22, oRatioParser::RuleInitializer_element);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(196);
        dynamic_cast<Initializer_elementContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(197);
        match(oRatioParser::LPAREN);
        setState(199);

        _la = _input->LA(1);
        if ((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::NEW)
                | (1ULL << oRatioParser::THIS)
                | (1ULL << oRatioParser::TRUE)
                | (1ULL << oRatioParser::FALSE)
                | (1ULL << oRatioParser::LPAREN)
                | (1ULL << oRatioParser::LBRACKET)
                | (1ULL << oRatioParser::PLUS)
                | (1ULL << oRatioParser::MINUS)
                | (1ULL << oRatioParser::BANG)
                | (1ULL << oRatioParser::ID)
                | (1ULL << oRatioParser::NumericLiteral)
                | (1ULL << oRatioParser::StringLiteral))) != 0)) {
            setState(198);
            expr_list();
        }
        setState(201);
        match(oRatioParser::RPAREN);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Predicate_declarationContext ------------------------------------------------------------------

oRatioParser::Predicate_declarationContext::Predicate_declarationContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::BlockContext* oRatioParser::Predicate_declarationContext::block() {
    return getRuleContext<oRatioParser::BlockContext>(0);
}

tree::TerminalNode* oRatioParser::Predicate_declarationContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Typed_listContext* oRatioParser::Predicate_declarationContext::typed_list() {
    return getRuleContext<oRatioParser::Typed_listContext>(0);
}

oRatioParser::Type_listContext* oRatioParser::Predicate_declarationContext::type_list() {
    return getRuleContext<oRatioParser::Type_listContext>(0);
}

size_t oRatioParser::Predicate_declarationContext::getRuleIndex() const {
    return oRatioParser::RulePredicate_declaration;
}

void oRatioParser::Predicate_declarationContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterPredicate_declaration(this);
}

void oRatioParser::Predicate_declarationContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitPredicate_declaration(this);
}

antlrcpp::Any oRatioParser::Predicate_declarationContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitPredicate_declaration(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Predicate_declarationContext* oRatioParser::predicate_declaration() {
    Predicate_declarationContext *_localctx = _tracker.createInstance<Predicate_declarationContext>(_ctx, getState());
    enterRule(_localctx, 24, oRatioParser::RulePredicate_declaration);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(203);
        match(oRatioParser::PREDICATE);
        setState(204);
        dynamic_cast<Predicate_declarationContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(205);
        match(oRatioParser::LPAREN);
        setState(207);

        _la = _input->LA(1);
        if ((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::REAL)
                | (1ULL << oRatioParser::BOOL)
                | (1ULL << oRatioParser::STRING)
                | (1ULL << oRatioParser::ID))) != 0)) {
            setState(206);
            typed_list();
        }
        setState(209);
        match(oRatioParser::RPAREN);
        setState(212);

        _la = _input->LA(1);
        if (_la == oRatioParser::COLON) {
            setState(210);
            match(oRatioParser::COLON);
            setState(211);
            type_list();
        }
        setState(214);
        match(oRatioParser::LBRACE);
        setState(215);
        block();
        setState(216);
        match(oRatioParser::RBRACE);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- StatementContext ------------------------------------------------------------------

oRatioParser::StatementContext::StatementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::Assignment_statementContext* oRatioParser::StatementContext::assignment_statement() {
    return getRuleContext<oRatioParser::Assignment_statementContext>(0);
}

oRatioParser::Local_variable_statementContext* oRatioParser::StatementContext::local_variable_statement() {
    return getRuleContext<oRatioParser::Local_variable_statementContext>(0);
}

oRatioParser::Expression_statementContext* oRatioParser::StatementContext::expression_statement() {
    return getRuleContext<oRatioParser::Expression_statementContext>(0);
}

oRatioParser::Disjunction_statementContext* oRatioParser::StatementContext::disjunction_statement() {
    return getRuleContext<oRatioParser::Disjunction_statementContext>(0);
}

oRatioParser::Formula_statementContext* oRatioParser::StatementContext::formula_statement() {
    return getRuleContext<oRatioParser::Formula_statementContext>(0);
}

oRatioParser::Return_statementContext* oRatioParser::StatementContext::return_statement() {
    return getRuleContext<oRatioParser::Return_statementContext>(0);
}

oRatioParser::BlockContext* oRatioParser::StatementContext::block() {
    return getRuleContext<oRatioParser::BlockContext>(0);
}

size_t oRatioParser::StatementContext::getRuleIndex() const {
    return oRatioParser::RuleStatement;
}

void oRatioParser::StatementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterStatement(this);
}

void oRatioParser::StatementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitStatement(this);
}

antlrcpp::Any oRatioParser::StatementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitStatement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::StatementContext* oRatioParser::statement() {
    StatementContext *_localctx = _tracker.createInstance<StatementContext>(_ctx, getState());
    enterRule(_localctx, 26, oRatioParser::RuleStatement);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(228);
        _errHandler->sync(this);
        switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 20, _ctx)) {
            case 1:
            {
                enterOuterAlt(_localctx, 1);
                setState(218);
                assignment_statement();
                break;
            }

            case 2:
            {
                enterOuterAlt(_localctx, 2);
                setState(219);
                local_variable_statement();
                break;
            }

            case 3:
            {
                enterOuterAlt(_localctx, 3);
                setState(220);
                expression_statement();
                break;
            }

            case 4:
            {
                enterOuterAlt(_localctx, 4);
                setState(221);
                disjunction_statement();
                break;
            }

            case 5:
            {
                enterOuterAlt(_localctx, 5);
                setState(222);
                formula_statement();
                break;
            }

            case 6:
            {
                enterOuterAlt(_localctx, 6);
                setState(223);
                return_statement();
                break;
            }

            case 7:
            {
                enterOuterAlt(_localctx, 7);
                setState(224);
                match(oRatioParser::LBRACE);
                setState(225);
                block();
                setState(226);
                match(oRatioParser::RBRACE);
                break;
            }

        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- BlockContext ------------------------------------------------------------------

oRatioParser::BlockContext::BlockContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::StatementContext *> oRatioParser::BlockContext::statement() {
    return getRuleContexts<oRatioParser::StatementContext>();
}

oRatioParser::StatementContext* oRatioParser::BlockContext::statement(size_t i) {
    return getRuleContext<oRatioParser::StatementContext>(i);
}

size_t oRatioParser::BlockContext::getRuleIndex() const {
    return oRatioParser::RuleBlock;
}

void oRatioParser::BlockContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterBlock(this);
}

void oRatioParser::BlockContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitBlock(this);
}

antlrcpp::Any oRatioParser::BlockContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitBlock(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::BlockContext* oRatioParser::block() {
    BlockContext *_localctx = _tracker.createInstance<BlockContext>(_ctx, getState());
    enterRule(_localctx, 28, oRatioParser::RuleBlock);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(233);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while ((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::REAL)
                | (1ULL << oRatioParser::BOOL)
                | (1ULL << oRatioParser::STRING)
                | (1ULL << oRatioParser::GOAL)
                | (1ULL << oRatioParser::FACT)
                | (1ULL << oRatioParser::NEW)
                | (1ULL << oRatioParser::THIS)
                | (1ULL << oRatioParser::TRUE)
                | (1ULL << oRatioParser::FALSE)
                | (1ULL << oRatioParser::RETURN)
                | (1ULL << oRatioParser::LPAREN)
                | (1ULL << oRatioParser::LBRACKET)
                | (1ULL << oRatioParser::LBRACE)
                | (1ULL << oRatioParser::PLUS)
                | (1ULL << oRatioParser::MINUS)
                | (1ULL << oRatioParser::BANG)
                | (1ULL << oRatioParser::ID)
                | (1ULL << oRatioParser::NumericLiteral)
                | (1ULL << oRatioParser::StringLiteral))) != 0)) {
            setState(230);
            statement();
            setState(235);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Assignment_statementContext ------------------------------------------------------------------

oRatioParser::Assignment_statementContext::Assignment_statementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::Assignment_statementContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::ExprContext* oRatioParser::Assignment_statementContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

oRatioParser::Qualified_idContext* oRatioParser::Assignment_statementContext::qualified_id() {
    return getRuleContext<oRatioParser::Qualified_idContext>(0);
}

size_t oRatioParser::Assignment_statementContext::getRuleIndex() const {
    return oRatioParser::RuleAssignment_statement;
}

void oRatioParser::Assignment_statementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterAssignment_statement(this);
}

void oRatioParser::Assignment_statementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitAssignment_statement(this);
}

antlrcpp::Any oRatioParser::Assignment_statementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitAssignment_statement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Assignment_statementContext* oRatioParser::assignment_statement() {
    Assignment_statementContext *_localctx = _tracker.createInstance<Assignment_statementContext>(_ctx, getState());
    enterRule(_localctx, 30, oRatioParser::RuleAssignment_statement);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(239);
        _errHandler->sync(this);

        switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 22, _ctx)) {
            case 1:
            {
                setState(236);
                dynamic_cast<Assignment_statementContext *> (_localctx)->object = qualified_id();
                setState(237);
                match(oRatioParser::DOT);
                break;
            }

        }
        setState(241);
        dynamic_cast<Assignment_statementContext *> (_localctx)->field = match(oRatioParser::ID);
        setState(242);
        match(oRatioParser::EQUAL);
        setState(243);
        dynamic_cast<Assignment_statementContext *> (_localctx)->value = expr(0);
        setState(244);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Local_variable_statementContext ------------------------------------------------------------------

oRatioParser::Local_variable_statementContext::Local_variable_statementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::TypeContext* oRatioParser::Local_variable_statementContext::type() {
    return getRuleContext<oRatioParser::TypeContext>(0);
}

std::vector<oRatioParser::Variable_decContext *> oRatioParser::Local_variable_statementContext::variable_dec() {
    return getRuleContexts<oRatioParser::Variable_decContext>();
}

oRatioParser::Variable_decContext* oRatioParser::Local_variable_statementContext::variable_dec(size_t i) {
    return getRuleContext<oRatioParser::Variable_decContext>(i);
}

size_t oRatioParser::Local_variable_statementContext::getRuleIndex() const {
    return oRatioParser::RuleLocal_variable_statement;
}

void oRatioParser::Local_variable_statementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterLocal_variable_statement(this);
}

void oRatioParser::Local_variable_statementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitLocal_variable_statement(this);
}

antlrcpp::Any oRatioParser::Local_variable_statementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitLocal_variable_statement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Local_variable_statementContext* oRatioParser::local_variable_statement() {
    Local_variable_statementContext *_localctx = _tracker.createInstance<Local_variable_statementContext>(_ctx, getState());
    enterRule(_localctx, 32, oRatioParser::RuleLocal_variable_statement);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(246);
        type();
        setState(247);
        variable_dec();
        setState(252);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::COMMA) {
            setState(248);
            match(oRatioParser::COMMA);
            setState(249);
            variable_dec();
            setState(254);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }
        setState(255);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Expression_statementContext ------------------------------------------------------------------

oRatioParser::Expression_statementContext::Expression_statementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::ExprContext* oRatioParser::Expression_statementContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

size_t oRatioParser::Expression_statementContext::getRuleIndex() const {
    return oRatioParser::RuleExpression_statement;
}

void oRatioParser::Expression_statementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterExpression_statement(this);
}

void oRatioParser::Expression_statementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitExpression_statement(this);
}

antlrcpp::Any oRatioParser::Expression_statementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitExpression_statement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Expression_statementContext* oRatioParser::expression_statement() {
    Expression_statementContext *_localctx = _tracker.createInstance<Expression_statementContext>(_ctx, getState());
    enterRule(_localctx, 34, oRatioParser::RuleExpression_statement);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(257);
        expr(0);
        setState(258);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Disjunction_statementContext ------------------------------------------------------------------

oRatioParser::Disjunction_statementContext::Disjunction_statementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::ConjunctionContext *> oRatioParser::Disjunction_statementContext::conjunction() {
    return getRuleContexts<oRatioParser::ConjunctionContext>();
}

oRatioParser::ConjunctionContext* oRatioParser::Disjunction_statementContext::conjunction(size_t i) {
    return getRuleContext<oRatioParser::ConjunctionContext>(i);
}

size_t oRatioParser::Disjunction_statementContext::getRuleIndex() const {
    return oRatioParser::RuleDisjunction_statement;
}

void oRatioParser::Disjunction_statementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterDisjunction_statement(this);
}

void oRatioParser::Disjunction_statementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitDisjunction_statement(this);
}

antlrcpp::Any oRatioParser::Disjunction_statementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitDisjunction_statement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Disjunction_statementContext* oRatioParser::disjunction_statement() {
    Disjunction_statementContext *_localctx = _tracker.createInstance<Disjunction_statementContext>(_ctx, getState());
    enterRule(_localctx, 36, oRatioParser::RuleDisjunction_statement);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(260);
        conjunction();
        setState(263);
        _errHandler->sync(this);
        _la = _input->LA(1);
        do {
            setState(261);
            match(oRatioParser::OR);
            setState(262);
            conjunction();
            setState(265);
            _errHandler->sync(this);
            _la = _input->LA(1);
        } while (_la == oRatioParser::OR);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- ConjunctionContext ------------------------------------------------------------------

oRatioParser::ConjunctionContext::ConjunctionContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::BlockContext* oRatioParser::ConjunctionContext::block() {
    return getRuleContext<oRatioParser::BlockContext>(0);
}

oRatioParser::ExprContext* oRatioParser::ConjunctionContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

size_t oRatioParser::ConjunctionContext::getRuleIndex() const {
    return oRatioParser::RuleConjunction;
}

void oRatioParser::ConjunctionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterConjunction(this);
}

void oRatioParser::ConjunctionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitConjunction(this);
}

antlrcpp::Any oRatioParser::ConjunctionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitConjunction(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::ConjunctionContext* oRatioParser::conjunction() {
    ConjunctionContext *_localctx = _tracker.createInstance<ConjunctionContext>(_ctx, getState());
    enterRule(_localctx, 38, oRatioParser::RuleConjunction);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(267);
        match(oRatioParser::LBRACE);
        setState(268);
        block();
        setState(269);
        match(oRatioParser::RBRACE);
        setState(274);
        _errHandler->sync(this);

        switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 25, _ctx)) {
            case 1:
            {
                setState(270);
                match(oRatioParser::LBRACKET);
                setState(271);
                dynamic_cast<ConjunctionContext *> (_localctx)->cost = expr(0);
                setState(272);
                match(oRatioParser::RBRACKET);
                break;
            }

        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Formula_statementContext ------------------------------------------------------------------

oRatioParser::Formula_statementContext::Formula_statementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<tree::TerminalNode *> oRatioParser::Formula_statementContext::ID() {
    return getTokens(oRatioParser::ID);
}

tree::TerminalNode* oRatioParser::Formula_statementContext::ID(size_t i) {
    return getToken(oRatioParser::ID, i);
}

oRatioParser::Assignment_listContext* oRatioParser::Formula_statementContext::assignment_list() {
    return getRuleContext<oRatioParser::Assignment_listContext>(0);
}

oRatioParser::Qualified_idContext* oRatioParser::Formula_statementContext::qualified_id() {
    return getRuleContext<oRatioParser::Qualified_idContext>(0);
}

size_t oRatioParser::Formula_statementContext::getRuleIndex() const {
    return oRatioParser::RuleFormula_statement;
}

void oRatioParser::Formula_statementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterFormula_statement(this);
}

void oRatioParser::Formula_statementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitFormula_statement(this);
}

antlrcpp::Any oRatioParser::Formula_statementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitFormula_statement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Formula_statementContext* oRatioParser::formula_statement() {
    Formula_statementContext *_localctx = _tracker.createInstance<Formula_statementContext>(_ctx, getState());
    enterRule(_localctx, 40, oRatioParser::RuleFormula_statement);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(278);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::GOAL:
            {
                setState(276);
                dynamic_cast<Formula_statementContext *> (_localctx)->goal = match(oRatioParser::GOAL);
                break;
            }

            case oRatioParser::FACT:
            {
                setState(277);
                dynamic_cast<Formula_statementContext *> (_localctx)->fact = match(oRatioParser::FACT);
                break;
            }

            default:
                throw NoViableAltException(this);
        }
        setState(280);
        dynamic_cast<Formula_statementContext *> (_localctx)->name = match(oRatioParser::ID);
        setState(281);
        match(oRatioParser::EQUAL);
        setState(282);
        match(oRatioParser::NEW);
        setState(286);
        _errHandler->sync(this);

        switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 27, _ctx)) {
            case 1:
            {
                setState(283);
                dynamic_cast<Formula_statementContext *> (_localctx)->object = qualified_id();
                setState(284);
                match(oRatioParser::DOT);
                break;
            }

        }
        setState(288);
        dynamic_cast<Formula_statementContext *> (_localctx)->predicate = match(oRatioParser::ID);
        setState(289);
        match(oRatioParser::LPAREN);
        setState(291);

        _la = _input->LA(1);
        if (_la == oRatioParser::ID) {
            setState(290);
            assignment_list();
        }
        setState(293);
        match(oRatioParser::RPAREN);
        setState(294);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Return_statementContext ------------------------------------------------------------------

oRatioParser::Return_statementContext::Return_statementContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::ExprContext* oRatioParser::Return_statementContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

size_t oRatioParser::Return_statementContext::getRuleIndex() const {
    return oRatioParser::RuleReturn_statement;
}

void oRatioParser::Return_statementContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterReturn_statement(this);
}

void oRatioParser::Return_statementContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitReturn_statement(this);
}

antlrcpp::Any oRatioParser::Return_statementContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitReturn_statement(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Return_statementContext* oRatioParser::return_statement() {
    Return_statementContext *_localctx = _tracker.createInstance<Return_statementContext>(_ctx, getState());
    enterRule(_localctx, 42, oRatioParser::RuleReturn_statement);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(296);
        match(oRatioParser::RETURN);
        setState(297);
        expr(0);
        setState(298);
        match(oRatioParser::SEMICOLON);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Assignment_listContext ------------------------------------------------------------------

oRatioParser::Assignment_listContext::Assignment_listContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::AssignmentContext *> oRatioParser::Assignment_listContext::assignment() {
    return getRuleContexts<oRatioParser::AssignmentContext>();
}

oRatioParser::AssignmentContext* oRatioParser::Assignment_listContext::assignment(size_t i) {
    return getRuleContext<oRatioParser::AssignmentContext>(i);
}

size_t oRatioParser::Assignment_listContext::getRuleIndex() const {
    return oRatioParser::RuleAssignment_list;
}

void oRatioParser::Assignment_listContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterAssignment_list(this);
}

void oRatioParser::Assignment_listContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitAssignment_list(this);
}

antlrcpp::Any oRatioParser::Assignment_listContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitAssignment_list(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Assignment_listContext* oRatioParser::assignment_list() {
    Assignment_listContext *_localctx = _tracker.createInstance<Assignment_listContext>(_ctx, getState());
    enterRule(_localctx, 44, oRatioParser::RuleAssignment_list);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(300);
        assignment();
        setState(305);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::COMMA) {
            setState(301);
            match(oRatioParser::COMMA);
            setState(302);
            assignment();
            setState(307);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- AssignmentContext ------------------------------------------------------------------

oRatioParser::AssignmentContext::AssignmentContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::AssignmentContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::ExprContext* oRatioParser::AssignmentContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

size_t oRatioParser::AssignmentContext::getRuleIndex() const {
    return oRatioParser::RuleAssignment;
}

void oRatioParser::AssignmentContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterAssignment(this);
}

void oRatioParser::AssignmentContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitAssignment(this);
}

antlrcpp::Any oRatioParser::AssignmentContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitAssignment(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::AssignmentContext* oRatioParser::assignment() {
    AssignmentContext *_localctx = _tracker.createInstance<AssignmentContext>(_ctx, getState());
    enterRule(_localctx, 46, oRatioParser::RuleAssignment);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(308);
        dynamic_cast<AssignmentContext *> (_localctx)->field = match(oRatioParser::ID);
        setState(309);
        match(oRatioParser::COLON);
        setState(310);
        dynamic_cast<AssignmentContext *> (_localctx)->value = expr(0);

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- ExprContext ------------------------------------------------------------------

oRatioParser::ExprContext::ExprContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

size_t oRatioParser::ExprContext::getRuleIndex() const {
    return oRatioParser::RuleExpr;
}

void oRatioParser::ExprContext::copyFrom(ExprContext *ctx) {
    ParserRuleContext::copyFrom(ctx);
}

//----------------- Cast_expressionContext ------------------------------------------------------------------

oRatioParser::TypeContext* oRatioParser::Cast_expressionContext::type() {
    return getRuleContext<oRatioParser::TypeContext>(0);
}

oRatioParser::ExprContext* oRatioParser::Cast_expressionContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

oRatioParser::Cast_expressionContext::Cast_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Cast_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterCast_expression(this);
}

void oRatioParser::Cast_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitCast_expression(this);
}

antlrcpp::Any oRatioParser::Cast_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitCast_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Qualified_id_expressionContext ------------------------------------------------------------------

oRatioParser::Qualified_idContext* oRatioParser::Qualified_id_expressionContext::qualified_id() {
    return getRuleContext<oRatioParser::Qualified_idContext>(0);
}

oRatioParser::Qualified_id_expressionContext::Qualified_id_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Qualified_id_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterQualified_id_expression(this);
}

void oRatioParser::Qualified_id_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitQualified_id_expression(this);
}

antlrcpp::Any oRatioParser::Qualified_id_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitQualified_id_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Division_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Division_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Division_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Division_expressionContext::Division_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Division_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterDivision_expression(this);
}

void oRatioParser::Division_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitDivision_expression(this);
}

antlrcpp::Any oRatioParser::Division_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitDivision_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Subtraction_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Subtraction_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Subtraction_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Subtraction_expressionContext::Subtraction_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Subtraction_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterSubtraction_expression(this);
}

void oRatioParser::Subtraction_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitSubtraction_expression(this);
}

antlrcpp::Any oRatioParser::Subtraction_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitSubtraction_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Extc_one_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Extc_one_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Extc_one_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Extc_one_expressionContext::Extc_one_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Extc_one_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterExtc_one_expression(this);
}

void oRatioParser::Extc_one_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitExtc_one_expression(this);
}

antlrcpp::Any oRatioParser::Extc_one_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitExtc_one_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Plus_expressionContext ------------------------------------------------------------------

oRatioParser::ExprContext* oRatioParser::Plus_expressionContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

oRatioParser::Plus_expressionContext::Plus_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Plus_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterPlus_expression(this);
}

void oRatioParser::Plus_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitPlus_expression(this);
}

antlrcpp::Any oRatioParser::Plus_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitPlus_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Function_expressionContext ------------------------------------------------------------------

tree::TerminalNode* oRatioParser::Function_expressionContext::ID() {
    return getToken(oRatioParser::ID, 0);
}

oRatioParser::Expr_listContext* oRatioParser::Function_expressionContext::expr_list() {
    return getRuleContext<oRatioParser::Expr_listContext>(0);
}

oRatioParser::Qualified_idContext* oRatioParser::Function_expressionContext::qualified_id() {
    return getRuleContext<oRatioParser::Qualified_idContext>(0);
}

oRatioParser::Function_expressionContext::Function_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Function_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterFunction_expression(this);
}

void oRatioParser::Function_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitFunction_expression(this);
}

antlrcpp::Any oRatioParser::Function_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitFunction_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Addition_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Addition_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Addition_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Addition_expressionContext::Addition_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Addition_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterAddition_expression(this);
}

void oRatioParser::Addition_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitAddition_expression(this);
}

antlrcpp::Any oRatioParser::Addition_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitAddition_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Parentheses_expressionContext ------------------------------------------------------------------

oRatioParser::ExprContext* oRatioParser::Parentheses_expressionContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

oRatioParser::Parentheses_expressionContext::Parentheses_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Parentheses_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterParentheses_expression(this);
}

void oRatioParser::Parentheses_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitParentheses_expression(this);
}

antlrcpp::Any oRatioParser::Parentheses_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitParentheses_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Minus_expressionContext ------------------------------------------------------------------

oRatioParser::ExprContext* oRatioParser::Minus_expressionContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

oRatioParser::Minus_expressionContext::Minus_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Minus_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterMinus_expression(this);
}

void oRatioParser::Minus_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitMinus_expression(this);
}

antlrcpp::Any oRatioParser::Minus_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitMinus_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Implication_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Implication_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Implication_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Implication_expressionContext::Implication_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Implication_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterImplication_expression(this);
}

void oRatioParser::Implication_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitImplication_expression(this);
}

antlrcpp::Any oRatioParser::Implication_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitImplication_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Lt_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Lt_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Lt_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Lt_expressionContext::Lt_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Lt_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterLt_expression(this);
}

void oRatioParser::Lt_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitLt_expression(this);
}

antlrcpp::Any oRatioParser::Lt_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitLt_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Not_expressionContext ------------------------------------------------------------------

oRatioParser::ExprContext* oRatioParser::Not_expressionContext::expr() {
    return getRuleContext<oRatioParser::ExprContext>(0);
}

oRatioParser::Not_expressionContext::Not_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Not_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterNot_expression(this);
}

void oRatioParser::Not_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitNot_expression(this);
}

antlrcpp::Any oRatioParser::Not_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitNot_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Conjunction_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Conjunction_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Conjunction_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Conjunction_expressionContext::Conjunction_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Conjunction_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterConjunction_expression(this);
}

void oRatioParser::Conjunction_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitConjunction_expression(this);
}

antlrcpp::Any oRatioParser::Conjunction_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitConjunction_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Geq_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Geq_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Geq_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Geq_expressionContext::Geq_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Geq_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterGeq_expression(this);
}

void oRatioParser::Geq_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitGeq_expression(this);
}

antlrcpp::Any oRatioParser::Geq_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitGeq_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Range_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Range_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Range_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Range_expressionContext::Range_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Range_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterRange_expression(this);
}

void oRatioParser::Range_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitRange_expression(this);
}

antlrcpp::Any oRatioParser::Range_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitRange_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Multiplication_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Multiplication_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Multiplication_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Multiplication_expressionContext::Multiplication_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Multiplication_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterMultiplication_expression(this);
}

void oRatioParser::Multiplication_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitMultiplication_expression(this);
}

antlrcpp::Any oRatioParser::Multiplication_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitMultiplication_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Leq_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Leq_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Leq_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Leq_expressionContext::Leq_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Leq_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterLeq_expression(this);
}

void oRatioParser::Leq_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitLeq_expression(this);
}

antlrcpp::Any oRatioParser::Leq_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitLeq_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Gt_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Gt_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Gt_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Gt_expressionContext::Gt_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Gt_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterGt_expression(this);
}

void oRatioParser::Gt_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitGt_expression(this);
}

antlrcpp::Any oRatioParser::Gt_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitGt_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Constructor_expressionContext ------------------------------------------------------------------

oRatioParser::TypeContext* oRatioParser::Constructor_expressionContext::type() {
    return getRuleContext<oRatioParser::TypeContext>(0);
}

oRatioParser::Expr_listContext* oRatioParser::Constructor_expressionContext::expr_list() {
    return getRuleContext<oRatioParser::Expr_listContext>(0);
}

oRatioParser::Constructor_expressionContext::Constructor_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Constructor_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterConstructor_expression(this);
}

void oRatioParser::Constructor_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitConstructor_expression(this);
}

antlrcpp::Any oRatioParser::Constructor_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitConstructor_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Disjunction_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Disjunction_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Disjunction_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Disjunction_expressionContext::Disjunction_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Disjunction_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterDisjunction_expression(this);
}

void oRatioParser::Disjunction_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitDisjunction_expression(this);
}

antlrcpp::Any oRatioParser::Disjunction_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitDisjunction_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Literal_expressionContext ------------------------------------------------------------------

oRatioParser::LiteralContext* oRatioParser::Literal_expressionContext::literal() {
    return getRuleContext<oRatioParser::LiteralContext>(0);
}

oRatioParser::Literal_expressionContext::Literal_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Literal_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterLiteral_expression(this);
}

void oRatioParser::Literal_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitLiteral_expression(this);
}

antlrcpp::Any oRatioParser::Literal_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitLiteral_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Eq_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Eq_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Eq_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Eq_expressionContext::Eq_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Eq_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterEq_expression(this);
}

void oRatioParser::Eq_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitEq_expression(this);
}

antlrcpp::Any oRatioParser::Eq_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitEq_expression(this);
    else
        return visitor->visitChildren(this);
}
//----------------- Neq_expressionContext ------------------------------------------------------------------

std::vector<oRatioParser::ExprContext *> oRatioParser::Neq_expressionContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Neq_expressionContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

oRatioParser::Neq_expressionContext::Neq_expressionContext(ExprContext *ctx) {
    copyFrom(ctx);
}

void oRatioParser::Neq_expressionContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterNeq_expression(this);
}

void oRatioParser::Neq_expressionContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitNeq_expression(this);
}

antlrcpp::Any oRatioParser::Neq_expressionContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitNeq_expression(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::ExprContext* oRatioParser::expr() {
    return expr(0);
}

oRatioParser::ExprContext* oRatioParser::expr(int precedence) {
    ParserRuleContext *parentContext = _ctx;
    size_t parentState = getState();
    oRatioParser::ExprContext *_localctx = _tracker.createInstance<ExprContext>(_ctx, parentState);
    oRatioParser::ExprContext *previousContext = _localctx;
    size_t startState = 48;
    enterRecursionRule(_localctx, 48, oRatioParser::RuleExpr, precedence);

    size_t _la = 0;

    auto onExit = finally([ = ]{
        unrollRecursionContexts(parentContext);
    });
    try {
        size_t alt;
        enterOuterAlt(_localctx, 1);
        setState(355);
        _errHandler->sync(this);
        switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 33, _ctx)) {
            case 1:
            {
                _localctx = _tracker.createInstance<Literal_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;

                setState(313);
                literal();
                break;
            }

            case 2:
            {
                _localctx = _tracker.createInstance<Parentheses_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(314);
                match(oRatioParser::LPAREN);
                setState(315);
                expr(0);
                setState(316);
                match(oRatioParser::RPAREN);
                break;
            }

            case 3:
            {
                _localctx = _tracker.createInstance<Plus_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(318);
                match(oRatioParser::PLUS);
                setState(319);
                expr(18);
                break;
            }

            case 4:
            {
                _localctx = _tracker.createInstance<Minus_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(320);
                match(oRatioParser::MINUS);
                setState(321);
                expr(17);
                break;
            }

            case 5:
            {
                _localctx = _tracker.createInstance<Not_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(322);
                match(oRatioParser::BANG);
                setState(323);
                expr(16);
                break;
            }

            case 6:
            {
                _localctx = _tracker.createInstance<Qualified_id_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(324);
                qualified_id();
                break;
            }

            case 7:
            {
                _localctx = _tracker.createInstance<Function_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(328);
                _errHandler->sync(this);

                switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 30, _ctx)) {
                    case 1:
                    {
                        setState(325);
                        dynamic_cast<Function_expressionContext *> (_localctx)->object = qualified_id();
                        setState(326);
                        match(oRatioParser::DOT);
                        break;
                    }

                }
                setState(330);
                dynamic_cast<Function_expressionContext *> (_localctx)->function_name = match(oRatioParser::ID);
                setState(331);
                match(oRatioParser::LPAREN);
                setState(333);

                _la = _input->LA(1);
                if ((((_la & ~0x3fULL) == 0) &&
                        ((1ULL << _la) & ((1ULL << oRatioParser::NEW)
                        | (1ULL << oRatioParser::THIS)
                        | (1ULL << oRatioParser::TRUE)
                        | (1ULL << oRatioParser::FALSE)
                        | (1ULL << oRatioParser::LPAREN)
                        | (1ULL << oRatioParser::LBRACKET)
                        | (1ULL << oRatioParser::PLUS)
                        | (1ULL << oRatioParser::MINUS)
                        | (1ULL << oRatioParser::BANG)
                        | (1ULL << oRatioParser::ID)
                        | (1ULL << oRatioParser::NumericLiteral)
                        | (1ULL << oRatioParser::StringLiteral))) != 0)) {
                    setState(332);
                    expr_list();
                }
                setState(335);
                match(oRatioParser::RPAREN);
                break;
            }

            case 8:
            {
                _localctx = _tracker.createInstance<Cast_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(336);
                match(oRatioParser::LPAREN);
                setState(337);
                type();
                setState(338);
                match(oRatioParser::RPAREN);
                setState(339);
                expr(13);
                break;
            }

            case 9:
            {
                _localctx = _tracker.createInstance<Range_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(341);
                match(oRatioParser::LBRACKET);
                setState(342);
                dynamic_cast<Range_expressionContext *> (_localctx)->min = expr(0);
                setState(343);
                match(oRatioParser::COMMA);
                setState(344);
                dynamic_cast<Range_expressionContext *> (_localctx)->max = expr(0);
                setState(345);
                match(oRatioParser::RBRACKET);
                break;
            }

            case 10:
            {
                _localctx = _tracker.createInstance<Constructor_expressionContext>(_localctx);
                _ctx = _localctx;
                previousContext = _localctx;
                setState(347);
                match(oRatioParser::NEW);
                setState(348);
                type();
                setState(349);
                match(oRatioParser::LPAREN);
                setState(351);

                _la = _input->LA(1);
                if ((((_la & ~0x3fULL) == 0) &&
                        ((1ULL << _la) & ((1ULL << oRatioParser::NEW)
                        | (1ULL << oRatioParser::THIS)
                        | (1ULL << oRatioParser::TRUE)
                        | (1ULL << oRatioParser::FALSE)
                        | (1ULL << oRatioParser::LPAREN)
                        | (1ULL << oRatioParser::LBRACKET)
                        | (1ULL << oRatioParser::PLUS)
                        | (1ULL << oRatioParser::MINUS)
                        | (1ULL << oRatioParser::BANG)
                        | (1ULL << oRatioParser::ID)
                        | (1ULL << oRatioParser::NumericLiteral)
                        | (1ULL << oRatioParser::StringLiteral))) != 0)) {
                    setState(350);
                    expr_list();
                }
                setState(353);
                match(oRatioParser::RPAREN);
                break;
            }

        }
        _ctx->stop = _input->LT(-1);
        setState(425);
        _errHandler->sync(this);
        alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 41, _ctx);
        while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER) {
            if (alt == 1) {
                if (!_parseListeners.empty())
                    triggerExitRuleEvent();
                previousContext = _localctx;
                setState(423);
                _errHandler->sync(this);
                switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 40, _ctx)) {
                    case 1:
                    {
                        auto newContext = _tracker.createInstance<Division_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(357);

                        if (!(precpred(_ctx, 21))) throw FailedPredicateException(this, "precpred(_ctx, 21)");
                        setState(358);
                        match(oRatioParser::SLASH);
                        setState(359);
                        expr(22);
                        break;
                    }

                    case 2:
                    {
                        auto newContext = _tracker.createInstance<Eq_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(360);

                        if (!(precpred(_ctx, 10))) throw FailedPredicateException(this, "precpred(_ctx, 10)");
                        setState(361);
                        match(oRatioParser::EQEQ);
                        setState(362);
                        expr(11);
                        break;
                    }

                    case 3:
                    {
                        auto newContext = _tracker.createInstance<Geq_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(363);

                        if (!(precpred(_ctx, 9))) throw FailedPredicateException(this, "precpred(_ctx, 9)");
                        setState(364);
                        match(oRatioParser::GTEQ);
                        setState(365);
                        expr(10);
                        break;
                    }

                    case 4:
                    {
                        auto newContext = _tracker.createInstance<Leq_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(366);

                        if (!(precpred(_ctx, 8))) throw FailedPredicateException(this, "precpred(_ctx, 8)");
                        setState(367);
                        match(oRatioParser::LTEQ);
                        setState(368);
                        expr(9);
                        break;
                    }

                    case 5:
                    {
                        auto newContext = _tracker.createInstance<Gt_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(369);

                        if (!(precpred(_ctx, 7))) throw FailedPredicateException(this, "precpred(_ctx, 7)");
                        setState(370);
                        match(oRatioParser::GT);
                        setState(371);
                        expr(8);
                        break;
                    }

                    case 6:
                    {
                        auto newContext = _tracker.createInstance<Lt_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(372);

                        if (!(precpred(_ctx, 6))) throw FailedPredicateException(this, "precpred(_ctx, 6)");
                        setState(373);
                        match(oRatioParser::LT);
                        setState(374);
                        expr(7);
                        break;
                    }

                    case 7:
                    {
                        auto newContext = _tracker.createInstance<Neq_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(375);

                        if (!(precpred(_ctx, 5))) throw FailedPredicateException(this, "precpred(_ctx, 5)");
                        setState(376);
                        match(oRatioParser::BANGEQ);
                        setState(377);
                        expr(6);
                        break;
                    }

                    case 8:
                    {
                        auto newContext = _tracker.createInstance<Implication_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(378);

                        if (!(precpred(_ctx, 4))) throw FailedPredicateException(this, "precpred(_ctx, 4)");
                        setState(379);
                        match(oRatioParser::IMPLICATION);
                        setState(380);
                        expr(5);
                        break;
                    }

                    case 9:
                    {
                        auto newContext = _tracker.createInstance<Multiplication_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(381);

                        if (!(precpred(_ctx, 22))) throw FailedPredicateException(this, "precpred(_ctx, 22)");
                        setState(384);
                        _errHandler->sync(this);
                        alt = 1;
                        do {
                            switch (alt) {
                                case 1:
                                {
                                    setState(382);
                                    match(oRatioParser::STAR);
                                    setState(383);
                                    expr(0);
                                    break;
                                }

                                default:
                                    throw NoViableAltException(this);
                            }
                            setState(386);
                            _errHandler->sync(this);
                            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 34, _ctx);
                        } while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER);
                        break;
                    }

                    case 10:
                    {
                        auto newContext = _tracker.createInstance<Addition_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(388);

                        if (!(precpred(_ctx, 20))) throw FailedPredicateException(this, "precpred(_ctx, 20)");
                        setState(391);
                        _errHandler->sync(this);
                        alt = 1;
                        do {
                            switch (alt) {
                                case 1:
                                {
                                    setState(389);
                                    match(oRatioParser::PLUS);
                                    setState(390);
                                    expr(0);
                                    break;
                                }

                                default:
                                    throw NoViableAltException(this);
                            }
                            setState(393);
                            _errHandler->sync(this);
                            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 35, _ctx);
                        } while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER);
                        break;
                    }

                    case 11:
                    {
                        auto newContext = _tracker.createInstance<Subtraction_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(395);

                        if (!(precpred(_ctx, 19))) throw FailedPredicateException(this, "precpred(_ctx, 19)");
                        setState(398);
                        _errHandler->sync(this);
                        alt = 1;
                        do {
                            switch (alt) {
                                case 1:
                                {
                                    setState(396);
                                    match(oRatioParser::MINUS);
                                    setState(397);
                                    expr(0);
                                    break;
                                }

                                default:
                                    throw NoViableAltException(this);
                            }
                            setState(400);
                            _errHandler->sync(this);
                            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 36, _ctx);
                        } while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER);
                        break;
                    }

                    case 12:
                    {
                        auto newContext = _tracker.createInstance<Disjunction_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(402);

                        if (!(precpred(_ctx, 3))) throw FailedPredicateException(this, "precpred(_ctx, 3)");
                        setState(405);
                        _errHandler->sync(this);
                        alt = 1;
                        do {
                            switch (alt) {
                                case 1:
                                {
                                    setState(403);
                                    match(oRatioParser::BAR);
                                    setState(404);
                                    expr(0);
                                    break;
                                }

                                default:
                                    throw NoViableAltException(this);
                            }
                            setState(407);
                            _errHandler->sync(this);
                            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 37, _ctx);
                        } while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER);
                        break;
                    }

                    case 13:
                    {
                        auto newContext = _tracker.createInstance<Conjunction_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(409);

                        if (!(precpred(_ctx, 2))) throw FailedPredicateException(this, "precpred(_ctx, 2)");
                        setState(412);
                        _errHandler->sync(this);
                        alt = 1;
                        do {
                            switch (alt) {
                                case 1:
                                {
                                    setState(410);
                                    match(oRatioParser::AMP);
                                    setState(411);
                                    expr(0);
                                    break;
                                }

                                default:
                                    throw NoViableAltException(this);
                            }
                            setState(414);
                            _errHandler->sync(this);
                            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 38, _ctx);
                        } while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER);
                        break;
                    }

                    case 14:
                    {
                        auto newContext = _tracker.createInstance<Extc_one_expressionContext>(_tracker.createInstance<ExprContext>(parentContext, parentState));
                        _localctx = newContext;
                        pushNewRecursionContext(newContext, startState, RuleExpr);
                        setState(416);

                        if (!(precpred(_ctx, 1))) throw FailedPredicateException(this, "precpred(_ctx, 1)");
                        setState(419);
                        _errHandler->sync(this);
                        alt = 1;
                        do {
                            switch (alt) {
                                case 1:
                                {
                                    setState(417);
                                    match(oRatioParser::CARET);
                                    setState(418);
                                    expr(0);
                                    break;
                                }

                                default:
                                    throw NoViableAltException(this);
                            }
                            setState(421);
                            _errHandler->sync(this);
                            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 39, _ctx);
                        } while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER);
                        break;
                    }

                }
            }
            setState(427);
            _errHandler->sync(this);
            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 41, _ctx);
        }
    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }
    return _localctx;
}

//----------------- Expr_listContext ------------------------------------------------------------------

oRatioParser::Expr_listContext::Expr_listContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::ExprContext *> oRatioParser::Expr_listContext::expr() {
    return getRuleContexts<oRatioParser::ExprContext>();
}

oRatioParser::ExprContext* oRatioParser::Expr_listContext::expr(size_t i) {
    return getRuleContext<oRatioParser::ExprContext>(i);
}

size_t oRatioParser::Expr_listContext::getRuleIndex() const {
    return oRatioParser::RuleExpr_list;
}

void oRatioParser::Expr_listContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterExpr_list(this);
}

void oRatioParser::Expr_listContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitExpr_list(this);
}

antlrcpp::Any oRatioParser::Expr_listContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitExpr_list(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Expr_listContext* oRatioParser::expr_list() {
    Expr_listContext *_localctx = _tracker.createInstance<Expr_listContext>(_ctx, getState());
    enterRule(_localctx, 50, oRatioParser::RuleExpr_list);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(428);
        expr(0);
        setState(433);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::COMMA) {
            setState(429);
            match(oRatioParser::COMMA);
            setState(430);
            expr(0);
            setState(435);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- LiteralContext ------------------------------------------------------------------

oRatioParser::LiteralContext::LiteralContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

tree::TerminalNode* oRatioParser::LiteralContext::NumericLiteral() {
    return getToken(oRatioParser::NumericLiteral, 0);
}

tree::TerminalNode* oRatioParser::LiteralContext::StringLiteral() {
    return getToken(oRatioParser::StringLiteral, 0);
}

size_t oRatioParser::LiteralContext::getRuleIndex() const {
    return oRatioParser::RuleLiteral;
}

void oRatioParser::LiteralContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterLiteral(this);
}

void oRatioParser::LiteralContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitLiteral(this);
}

antlrcpp::Any oRatioParser::LiteralContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitLiteral(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::LiteralContext* oRatioParser::literal() {
    LiteralContext *_localctx = _tracker.createInstance<LiteralContext>(_ctx, getState());
    enterRule(_localctx, 52, oRatioParser::RuleLiteral);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(440);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::NumericLiteral:
            {
                enterOuterAlt(_localctx, 1);
                setState(436);
                dynamic_cast<LiteralContext *> (_localctx)->numeric = match(oRatioParser::NumericLiteral);
                break;
            }

            case oRatioParser::StringLiteral:
            {
                enterOuterAlt(_localctx, 2);
                setState(437);
                dynamic_cast<LiteralContext *> (_localctx)->string = match(oRatioParser::StringLiteral);
                break;
            }

            case oRatioParser::TRUE:
            {
                enterOuterAlt(_localctx, 3);
                setState(438);
                dynamic_cast<LiteralContext *> (_localctx)->t = match(oRatioParser::TRUE);
                break;
            }

            case oRatioParser::FALSE:
            {
                enterOuterAlt(_localctx, 4);
                setState(439);
                dynamic_cast<LiteralContext *> (_localctx)->f = match(oRatioParser::FALSE);
                break;
            }

            default:
                throw NoViableAltException(this);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Qualified_idContext ------------------------------------------------------------------

oRatioParser::Qualified_idContext::Qualified_idContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<tree::TerminalNode *> oRatioParser::Qualified_idContext::ID() {
    return getTokens(oRatioParser::ID);
}

tree::TerminalNode* oRatioParser::Qualified_idContext::ID(size_t i) {
    return getToken(oRatioParser::ID, i);
}

size_t oRatioParser::Qualified_idContext::getRuleIndex() const {
    return oRatioParser::RuleQualified_id;
}

void oRatioParser::Qualified_idContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterQualified_id(this);
}

void oRatioParser::Qualified_idContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitQualified_id(this);
}

antlrcpp::Any oRatioParser::Qualified_idContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitQualified_id(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Qualified_idContext* oRatioParser::qualified_id() {
    Qualified_idContext *_localctx = _tracker.createInstance<Qualified_idContext>(_ctx, getState());
    enterRule(_localctx, 54, oRatioParser::RuleQualified_id);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        size_t alt;
        enterOuterAlt(_localctx, 1);
        setState(444);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::THIS:
            {
                setState(442);
                dynamic_cast<Qualified_idContext *> (_localctx)->t = match(oRatioParser::THIS);
                break;
            }

            case oRatioParser::ID:
            {
                setState(443);
                match(oRatioParser::ID);
                break;
            }

            default:
                throw NoViableAltException(this);
        }
        setState(450);
        _errHandler->sync(this);
        alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 45, _ctx);
        while (alt != 2 && alt != atn::ATN::INVALID_ALT_NUMBER) {
            if (alt == 1) {
                setState(446);
                match(oRatioParser::DOT);
                setState(447);
                match(oRatioParser::ID);
            }
            setState(452);
            _errHandler->sync(this);
            alt = getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 45, _ctx);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- TypeContext ------------------------------------------------------------------

oRatioParser::TypeContext::TypeContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

oRatioParser::Class_typeContext* oRatioParser::TypeContext::class_type() {
    return getRuleContext<oRatioParser::Class_typeContext>(0);
}

oRatioParser::Primitive_typeContext* oRatioParser::TypeContext::primitive_type() {
    return getRuleContext<oRatioParser::Primitive_typeContext>(0);
}

size_t oRatioParser::TypeContext::getRuleIndex() const {
    return oRatioParser::RuleType;
}

void oRatioParser::TypeContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterType(this);
}

void oRatioParser::TypeContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitType(this);
}

antlrcpp::Any oRatioParser::TypeContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitType(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::TypeContext* oRatioParser::type() {
    TypeContext *_localctx = _tracker.createInstance<TypeContext>(_ctx, getState());
    enterRule(_localctx, 56, oRatioParser::RuleType);

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        setState(455);
        _errHandler->sync(this);
        switch (_input->LA(1)) {
            case oRatioParser::ID:
            {
                enterOuterAlt(_localctx, 1);
                setState(453);
                class_type();
                break;
            }

            case oRatioParser::REAL:
            case oRatioParser::BOOL:
            case oRatioParser::STRING:
            {
                enterOuterAlt(_localctx, 2);
                setState(454);
                primitive_type();
                break;
            }

            default:
                throw NoViableAltException(this);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Class_typeContext ------------------------------------------------------------------

oRatioParser::Class_typeContext::Class_typeContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<tree::TerminalNode *> oRatioParser::Class_typeContext::ID() {
    return getTokens(oRatioParser::ID);
}

tree::TerminalNode* oRatioParser::Class_typeContext::ID(size_t i) {
    return getToken(oRatioParser::ID, i);
}

size_t oRatioParser::Class_typeContext::getRuleIndex() const {
    return oRatioParser::RuleClass_type;
}

void oRatioParser::Class_typeContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterClass_type(this);
}

void oRatioParser::Class_typeContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitClass_type(this);
}

antlrcpp::Any oRatioParser::Class_typeContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitClass_type(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Class_typeContext* oRatioParser::class_type() {
    Class_typeContext *_localctx = _tracker.createInstance<Class_typeContext>(_ctx, getState());
    enterRule(_localctx, 58, oRatioParser::RuleClass_type);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(457);
        match(oRatioParser::ID);
        setState(462);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::DOT) {
            setState(458);
            match(oRatioParser::DOT);
            setState(459);
            match(oRatioParser::ID);
            setState(464);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Primitive_typeContext ------------------------------------------------------------------

oRatioParser::Primitive_typeContext::Primitive_typeContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

size_t oRatioParser::Primitive_typeContext::getRuleIndex() const {
    return oRatioParser::RulePrimitive_type;
}

void oRatioParser::Primitive_typeContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterPrimitive_type(this);
}

void oRatioParser::Primitive_typeContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitPrimitive_type(this);
}

antlrcpp::Any oRatioParser::Primitive_typeContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitPrimitive_type(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Primitive_typeContext* oRatioParser::primitive_type() {
    Primitive_typeContext *_localctx = _tracker.createInstance<Primitive_typeContext>(_ctx, getState());
    enterRule(_localctx, 60, oRatioParser::RulePrimitive_type);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(465);
        _la = _input->LA(1);
        if (!((((_la & ~0x3fULL) == 0) &&
                ((1ULL << _la) & ((1ULL << oRatioParser::REAL)
                | (1ULL << oRatioParser::BOOL)
                | (1ULL << oRatioParser::STRING))) != 0))) {
            _errHandler->recoverInline(this);
        } else {
            consume();
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Type_listContext ------------------------------------------------------------------

oRatioParser::Type_listContext::Type_listContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::TypeContext *> oRatioParser::Type_listContext::type() {
    return getRuleContexts<oRatioParser::TypeContext>();
}

oRatioParser::TypeContext* oRatioParser::Type_listContext::type(size_t i) {
    return getRuleContext<oRatioParser::TypeContext>(i);
}

size_t oRatioParser::Type_listContext::getRuleIndex() const {
    return oRatioParser::RuleType_list;
}

void oRatioParser::Type_listContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterType_list(this);
}

void oRatioParser::Type_listContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitType_list(this);
}

antlrcpp::Any oRatioParser::Type_listContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitType_list(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Type_listContext* oRatioParser::type_list() {
    Type_listContext *_localctx = _tracker.createInstance<Type_listContext>(_ctx, getState());
    enterRule(_localctx, 62, oRatioParser::RuleType_list);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(467);
        type();
        setState(472);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::COMMA) {
            setState(468);
            match(oRatioParser::COMMA);
            setState(469);
            type();
            setState(474);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

//----------------- Typed_listContext ------------------------------------------------------------------

oRatioParser::Typed_listContext::Typed_listContext(ParserRuleContext *parent, size_t invokingState)
: ParserRuleContext(parent, invokingState) { }

std::vector<oRatioParser::TypeContext *> oRatioParser::Typed_listContext::type() {
    return getRuleContexts<oRatioParser::TypeContext>();
}

oRatioParser::TypeContext* oRatioParser::Typed_listContext::type(size_t i) {
    return getRuleContext<oRatioParser::TypeContext>(i);
}

std::vector<tree::TerminalNode *> oRatioParser::Typed_listContext::ID() {
    return getTokens(oRatioParser::ID);
}

tree::TerminalNode* oRatioParser::Typed_listContext::ID(size_t i) {
    return getToken(oRatioParser::ID, i);
}

size_t oRatioParser::Typed_listContext::getRuleIndex() const {
    return oRatioParser::RuleTyped_list;
}

void oRatioParser::Typed_listContext::enterRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->enterTyped_list(this);
}

void oRatioParser::Typed_listContext::exitRule(tree::ParseTreeListener *listener) {
    auto parserListener = dynamic_cast<oRatioListener *> (listener);
    if (parserListener != nullptr)
        parserListener->exitTyped_list(this);
}

antlrcpp::Any oRatioParser::Typed_listContext::accept(tree::ParseTreeVisitor *visitor) {
    if (dynamic_cast<oRatioVisitor*> (visitor) != nullptr)
        return ((oRatioVisitor *) visitor)->visitTyped_list(this);
    else
        return visitor->visitChildren(this);
}

oRatioParser::Typed_listContext* oRatioParser::typed_list() {
    Typed_listContext *_localctx = _tracker.createInstance<Typed_listContext>(_ctx, getState());
    enterRule(_localctx, 64, oRatioParser::RuleTyped_list);
    size_t _la = 0;

    auto onExit = finally([ = ]{
        exitRule();
    });
    try {
        enterOuterAlt(_localctx, 1);
        setState(475);
        type();
        setState(476);
        match(oRatioParser::ID);
        setState(483);
        _errHandler->sync(this);
        _la = _input->LA(1);
        while (_la == oRatioParser::COMMA) {
            setState(477);
            match(oRatioParser::COMMA);
            setState(478);
            type();
            setState(479);
            match(oRatioParser::ID);
            setState(485);
            _errHandler->sync(this);
            _la = _input->LA(1);
        }

    } catch (RecognitionException &e) {
        _errHandler->reportError(this, e);
        _localctx->exception = std::current_exception();
        _errHandler->recover(this, _localctx->exception);
    }

    return _localctx;
}

bool oRatioParser::sempred(RuleContext *context, size_t ruleIndex, size_t predicateIndex) {
    switch (ruleIndex) {
        case 24: return exprSempred(dynamic_cast<ExprContext *> (context), predicateIndex);

        default:
            break;
    }
    return true;
}

bool oRatioParser::exprSempred(ExprContext *_localctx, size_t predicateIndex) {
    switch (predicateIndex) {
        case 0: return precpred(_ctx, 21);
        case 1: return precpred(_ctx, 10);
        case 2: return precpred(_ctx, 9);
        case 3: return precpred(_ctx, 8);
        case 4: return precpred(_ctx, 7);
        case 5: return precpred(_ctx, 6);
        case 6: return precpred(_ctx, 5);
        case 7: return precpred(_ctx, 4);
        case 8: return precpred(_ctx, 22);
        case 9: return precpred(_ctx, 20);
        case 10: return precpred(_ctx, 19);
        case 11: return precpred(_ctx, 3);
        case 12: return precpred(_ctx, 2);
        case 13: return precpred(_ctx, 1);

        default:
            break;
    }
    return true;
}

// Static vars and initialization.
std::vector<dfa::DFA> oRatioParser::_decisionToDFA;
atn::PredictionContextCache oRatioParser::_sharedContextCache;

// We own the ATN which in turn owns the ATN states.
atn::ATN oRatioParser::_atn;
std::vector<uint16_t> oRatioParser::_serializedATN;

std::vector<std::string> oRatioParser::_ruleNames = {
    "compilation_unit", "type_declaration", "typedef_declaration", "enum_declaration",
    "enum_constants", "class_declaration", "member", "field_declaration",
    "variable_dec", "method_declaration", "constructor_declaration", "initializer_element",
    "predicate_declaration", "statement", "block", "assignment_statement",
    "local_variable_statement", "expression_statement", "disjunction_statement",
    "conjunction", "formula_statement", "return_statement", "assignment_list",
    "assignment", "expr", "expr_list", "literal", "qualified_id", "type",
    "class_type", "primitive_type", "type_list", "typed_list"
};

std::vector<std::string> oRatioParser::_literalNames = {
    "", "'typedef'", "'real'", "'bool'", "'string'", "'enum'", "'class'",
    "'goal'", "'fact'", "'predicate'", "'new'", "'or'", "'this'", "'void'",
    "'true'", "'false'", "'return'", "'.'", "','", "':'", "';'", "'('", "')'",
    "'['", "']'", "'{'", "'}'", "'+'", "'-'", "'*'", "'/'", "'&'", "'|'",
    "'='", "'>'", "'<'", "'!'", "'=='", "'<='", "'>='", "'!='", "'->'", "'^'"
};

std::vector<std::string> oRatioParser::_symbolicNames = {
    "", "TYPE_DEF", "REAL", "BOOL", "STRING", "ENUM", "CLASS", "GOAL", "FACT",
    "PREDICATE", "NEW", "OR", "THIS", "VOID", "TRUE", "FALSE", "RETURN", "DOT",
    "COMMA", "COLON", "SEMICOLON", "LPAREN", "RPAREN", "LBRACKET", "RBRACKET",
    "LBRACE", "RBRACE", "PLUS", "MINUS", "STAR", "SLASH", "AMP", "BAR", "EQUAL",
    "GT", "LT", "BANG", "EQEQ", "LTEQ", "GTEQ", "BANGEQ", "IMPLICATION", "CARET",
    "ID", "NumericLiteral", "StringLiteral", "LINE_COMMENT", "COMMENT", "WS"
};

dfa::Vocabulary oRatioParser::_vocabulary(_literalNames, _symbolicNames);

std::vector<std::string> oRatioParser::_tokenNames;

oRatioParser::Initializer::Initializer() {
    for (size_t i = 0; i < _symbolicNames.size(); ++i) {
        std::string name = _vocabulary.getLiteralName(i);
        if (name.empty()) {
            name = _vocabulary.getSymbolicName(i);
        }

        if (name.empty()) {
            _tokenNames.push_back("<INVALID>");
        } else {
            _tokenNames.push_back(name);
        }
    }

    _serializedATN = {
        0x3, 0x430, 0xd6d1, 0x8206, 0xad2d, 0x4417, 0xaef1, 0x8d80, 0xaadd,
        0x3, 0x32, 0x1e9, 0x4, 0x2, 0x9, 0x2, 0x4, 0x3, 0x9, 0x3, 0x4, 0x4,
        0x9, 0x4, 0x4, 0x5, 0x9, 0x5, 0x4, 0x6, 0x9, 0x6, 0x4, 0x7, 0x9, 0x7,
        0x4, 0x8, 0x9, 0x8, 0x4, 0x9, 0x9, 0x9, 0x4, 0xa, 0x9, 0xa, 0x4, 0xb,
        0x9, 0xb, 0x4, 0xc, 0x9, 0xc, 0x4, 0xd, 0x9, 0xd, 0x4, 0xe, 0x9, 0xe,
        0x4, 0xf, 0x9, 0xf, 0x4, 0x10, 0x9, 0x10, 0x4, 0x11, 0x9, 0x11, 0x4,
        0x12, 0x9, 0x12, 0x4, 0x13, 0x9, 0x13, 0x4, 0x14, 0x9, 0x14, 0x4, 0x15,
        0x9, 0x15, 0x4, 0x16, 0x9, 0x16, 0x4, 0x17, 0x9, 0x17, 0x4, 0x18, 0x9,
        0x18, 0x4, 0x19, 0x9, 0x19, 0x4, 0x1a, 0x9, 0x1a, 0x4, 0x1b, 0x9, 0x1b,
        0x4, 0x1c, 0x9, 0x1c, 0x4, 0x1d, 0x9, 0x1d, 0x4, 0x1e, 0x9, 0x1e, 0x4,
        0x1f, 0x9, 0x1f, 0x4, 0x20, 0x9, 0x20, 0x4, 0x21, 0x9, 0x21, 0x4, 0x22,
        0x9, 0x22, 0x3, 0x2, 0x3, 0x2, 0x3, 0x2, 0x3, 0x2, 0x7, 0x2, 0x49, 0xa,
        0x2, 0xc, 0x2, 0xe, 0x2, 0x4c, 0xb, 0x2, 0x3, 0x2, 0x3, 0x2, 0x3, 0x3,
        0x3, 0x3, 0x3, 0x3, 0x5, 0x3, 0x53, 0xa, 0x3, 0x3, 0x4, 0x3, 0x4, 0x3,
        0x4, 0x3, 0x4, 0x3, 0x4, 0x3, 0x4, 0x3, 0x5, 0x3, 0x5, 0x3, 0x5, 0x3,
        0x5, 0x3, 0x5, 0x7, 0x5, 0x60, 0xa, 0x5, 0xc, 0x5, 0xe, 0x5, 0x63, 0xb,
        0x5, 0x3, 0x5, 0x3, 0x5, 0x3, 0x6, 0x3, 0x6, 0x3, 0x6, 0x3, 0x6, 0x7,
        0x6, 0x6b, 0xa, 0x6, 0xc, 0x6, 0xe, 0x6, 0x6e, 0xb, 0x6, 0x3, 0x6, 0x3,
        0x6, 0x5, 0x6, 0x72, 0xa, 0x6, 0x3, 0x7, 0x3, 0x7, 0x3, 0x7, 0x3, 0x7,
        0x5, 0x7, 0x78, 0xa, 0x7, 0x3, 0x7, 0x3, 0x7, 0x7, 0x7, 0x7c, 0xa, 0x7,
        0xc, 0x7, 0xe, 0x7, 0x7f, 0xb, 0x7, 0x3, 0x7, 0x3, 0x7, 0x3, 0x8, 0x3,
        0x8, 0x3, 0x8, 0x3, 0x8, 0x3, 0x8, 0x5, 0x8, 0x88, 0xa, 0x8, 0x3, 0x9,
        0x3, 0x9, 0x3, 0x9, 0x3, 0x9, 0x7, 0x9, 0x8e, 0xa, 0x9, 0xc, 0x9, 0xe,
        0x9, 0x91, 0xb, 0x9, 0x3, 0x9, 0x3, 0x9, 0x3, 0xa, 0x3, 0xa, 0x3, 0xa,
        0x5, 0xa, 0x98, 0xa, 0xa, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x5,
        0xb, 0x9e, 0xa, 0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb,
        0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x5, 0xb, 0xa9, 0xa, 0xb, 0x3,
        0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x3, 0xb, 0x5, 0xb, 0xb0, 0xa, 0xb,
        0x3, 0xc, 0x3, 0xc, 0x3, 0xc, 0x5, 0xc, 0xb5, 0xa, 0xc, 0x3, 0xc, 0x3,
        0xc, 0x3, 0xc, 0x3, 0xc, 0x3, 0xc, 0x7, 0xc, 0xbc, 0xa, 0xc, 0xc, 0xc,
        0xe, 0xc, 0xbf, 0xb, 0xc, 0x5, 0xc, 0xc1, 0xa, 0xc, 0x3, 0xc, 0x3, 0xc,
        0x3, 0xc, 0x3, 0xc, 0x3, 0xd, 0x3, 0xd, 0x3, 0xd, 0x5, 0xd, 0xca, 0xa,
        0xd, 0x3, 0xd, 0x3, 0xd, 0x3, 0xe, 0x3, 0xe, 0x3, 0xe, 0x3, 0xe, 0x5,
        0xe, 0xd2, 0xa, 0xe, 0x3, 0xe, 0x3, 0xe, 0x3, 0xe, 0x5, 0xe, 0xd7, 0xa,
        0xe, 0x3, 0xe, 0x3, 0xe, 0x3, 0xe, 0x3, 0xe, 0x3, 0xf, 0x3, 0xf, 0x3,
        0xf, 0x3, 0xf, 0x3, 0xf, 0x3, 0xf, 0x3, 0xf, 0x3, 0xf, 0x3, 0xf, 0x3,
        0xf, 0x5, 0xf, 0xe7, 0xa, 0xf, 0x3, 0x10, 0x7, 0x10, 0xea, 0xa, 0x10,
        0xc, 0x10, 0xe, 0x10, 0xed, 0xb, 0x10, 0x3, 0x11, 0x3, 0x11, 0x3, 0x11,
        0x5, 0x11, 0xf2, 0xa, 0x11, 0x3, 0x11, 0x3, 0x11, 0x3, 0x11, 0x3, 0x11,
        0x3, 0x11, 0x3, 0x12, 0x3, 0x12, 0x3, 0x12, 0x3, 0x12, 0x7, 0x12, 0xfd,
        0xa, 0x12, 0xc, 0x12, 0xe, 0x12, 0x100, 0xb, 0x12, 0x3, 0x12, 0x3, 0x12,
        0x3, 0x13, 0x3, 0x13, 0x3, 0x13, 0x3, 0x14, 0x3, 0x14, 0x3, 0x14, 0x6,
        0x14, 0x10a, 0xa, 0x14, 0xd, 0x14, 0xe, 0x14, 0x10b, 0x3, 0x15, 0x3,
        0x15, 0x3, 0x15, 0x3, 0x15, 0x3, 0x15, 0x3, 0x15, 0x3, 0x15, 0x5, 0x15,
        0x115, 0xa, 0x15, 0x3, 0x16, 0x3, 0x16, 0x5, 0x16, 0x119, 0xa, 0x16,
        0x3, 0x16, 0x3, 0x16, 0x3, 0x16, 0x3, 0x16, 0x3, 0x16, 0x3, 0x16, 0x5,
        0x16, 0x121, 0xa, 0x16, 0x3, 0x16, 0x3, 0x16, 0x3, 0x16, 0x5, 0x16,
        0x126, 0xa, 0x16, 0x3, 0x16, 0x3, 0x16, 0x3, 0x16, 0x3, 0x17, 0x3, 0x17,
        0x3, 0x17, 0x3, 0x17, 0x3, 0x18, 0x3, 0x18, 0x3, 0x18, 0x7, 0x18, 0x132,
        0xa, 0x18, 0xc, 0x18, 0xe, 0x18, 0x135, 0xb, 0x18, 0x3, 0x19, 0x3, 0x19,
        0x3, 0x19, 0x3, 0x19, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3,
        0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a,
        0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x5, 0x1a, 0x14b,
        0xa, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x5, 0x1a, 0x150, 0xa, 0x1a,
        0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3,
        0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a,
        0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x5, 0x1a, 0x162, 0xa, 0x1a, 0x3, 0x1a,
        0x3, 0x1a, 0x5, 0x1a, 0x166, 0xa, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a,
        0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3,
        0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a,
        0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3,
        0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x6, 0x1a, 0x183,
        0xa, 0x1a, 0xd, 0x1a, 0xe, 0x1a, 0x184, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a,
        0x6, 0x1a, 0x18a, 0xa, 0x1a, 0xd, 0x1a, 0xe, 0x1a, 0x18b, 0x3, 0x1a,
        0x3, 0x1a, 0x3, 0x1a, 0x6, 0x1a, 0x191, 0xa, 0x1a, 0xd, 0x1a, 0xe, 0x1a,
        0x192, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x6, 0x1a, 0x198, 0xa, 0x1a,
        0xd, 0x1a, 0xe, 0x1a, 0x199, 0x3, 0x1a, 0x3, 0x1a, 0x3, 0x1a, 0x6, 0x1a,
        0x19f, 0xa, 0x1a, 0xd, 0x1a, 0xe, 0x1a, 0x1a0, 0x3, 0x1a, 0x3, 0x1a,
        0x3, 0x1a, 0x6, 0x1a, 0x1a6, 0xa, 0x1a, 0xd, 0x1a, 0xe, 0x1a, 0x1a7,
        0x7, 0x1a, 0x1aa, 0xa, 0x1a, 0xc, 0x1a, 0xe, 0x1a, 0x1ad, 0xb, 0x1a,
        0x3, 0x1b, 0x3, 0x1b, 0x3, 0x1b, 0x7, 0x1b, 0x1b2, 0xa, 0x1b, 0xc, 0x1b,
        0xe, 0x1b, 0x1b5, 0xb, 0x1b, 0x3, 0x1c, 0x3, 0x1c, 0x3, 0x1c, 0x3, 0x1c,
        0x5, 0x1c, 0x1bb, 0xa, 0x1c, 0x3, 0x1d, 0x3, 0x1d, 0x5, 0x1d, 0x1bf,
        0xa, 0x1d, 0x3, 0x1d, 0x3, 0x1d, 0x7, 0x1d, 0x1c3, 0xa, 0x1d, 0xc, 0x1d,
        0xe, 0x1d, 0x1c6, 0xb, 0x1d, 0x3, 0x1e, 0x3, 0x1e, 0x5, 0x1e, 0x1ca,
        0xa, 0x1e, 0x3, 0x1f, 0x3, 0x1f, 0x3, 0x1f, 0x7, 0x1f, 0x1cf, 0xa, 0x1f,
        0xc, 0x1f, 0xe, 0x1f, 0x1d2, 0xb, 0x1f, 0x3, 0x20, 0x3, 0x20, 0x3, 0x21,
        0x3, 0x21, 0x3, 0x21, 0x7, 0x21, 0x1d9, 0xa, 0x21, 0xc, 0x21, 0xe, 0x21,
        0x1dc, 0xb, 0x21, 0x3, 0x22, 0x3, 0x22, 0x3, 0x22, 0x3, 0x22, 0x3, 0x22,
        0x3, 0x22, 0x7, 0x22, 0x1e4, 0xa, 0x22, 0xc, 0x22, 0xe, 0x22, 0x1e7,
        0xb, 0x22, 0x3, 0x22, 0x2, 0x3, 0x32, 0x23, 0x2, 0x4, 0x6, 0x8, 0xa,
        0xc, 0xe, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e, 0x20, 0x22,
        0x24, 0x26, 0x28, 0x2a, 0x2c, 0x2e, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3a,
        0x3c, 0x3e, 0x40, 0x42, 0x2, 0x3, 0x3, 0x2, 0x4, 0x6, 0x21a, 0x2, 0x4a,
        0x3, 0x2, 0x2, 0x2, 0x4, 0x52, 0x3, 0x2, 0x2, 0x2, 0x6, 0x54, 0x3, 0x2,
        0x2, 0x2, 0x8, 0x5a, 0x3, 0x2, 0x2, 0x2, 0xa, 0x71, 0x3, 0x2, 0x2, 0x2,
        0xc, 0x73, 0x3, 0x2, 0x2, 0x2, 0xe, 0x87, 0x3, 0x2, 0x2, 0x2, 0x10,
        0x89, 0x3, 0x2, 0x2, 0x2, 0x12, 0x94, 0x3, 0x2, 0x2, 0x2, 0x14, 0xaf,
        0x3, 0x2, 0x2, 0x2, 0x16, 0xb1, 0x3, 0x2, 0x2, 0x2, 0x18, 0xc6, 0x3,
        0x2, 0x2, 0x2, 0x1a, 0xcd, 0x3, 0x2, 0x2, 0x2, 0x1c, 0xe6, 0x3, 0x2,
        0x2, 0x2, 0x1e, 0xeb, 0x3, 0x2, 0x2, 0x2, 0x20, 0xf1, 0x3, 0x2, 0x2,
        0x2, 0x22, 0xf8, 0x3, 0x2, 0x2, 0x2, 0x24, 0x103, 0x3, 0x2, 0x2, 0x2,
        0x26, 0x106, 0x3, 0x2, 0x2, 0x2, 0x28, 0x10d, 0x3, 0x2, 0x2, 0x2, 0x2a,
        0x118, 0x3, 0x2, 0x2, 0x2, 0x2c, 0x12a, 0x3, 0x2, 0x2, 0x2, 0x2e, 0x12e,
        0x3, 0x2, 0x2, 0x2, 0x30, 0x136, 0x3, 0x2, 0x2, 0x2, 0x32, 0x165, 0x3,
        0x2, 0x2, 0x2, 0x34, 0x1ae, 0x3, 0x2, 0x2, 0x2, 0x36, 0x1ba, 0x3, 0x2,
        0x2, 0x2, 0x38, 0x1be, 0x3, 0x2, 0x2, 0x2, 0x3a, 0x1c9, 0x3, 0x2, 0x2,
        0x2, 0x3c, 0x1cb, 0x3, 0x2, 0x2, 0x2, 0x3e, 0x1d3, 0x3, 0x2, 0x2, 0x2,
        0x40, 0x1d5, 0x3, 0x2, 0x2, 0x2, 0x42, 0x1dd, 0x3, 0x2, 0x2, 0x2, 0x44,
        0x49, 0x5, 0x4, 0x3, 0x2, 0x45, 0x49, 0x5, 0x14, 0xb, 0x2, 0x46, 0x49,
        0x5, 0x1a, 0xe, 0x2, 0x47, 0x49, 0x5, 0x1c, 0xf, 0x2, 0x48, 0x44, 0x3,
        0x2, 0x2, 0x2, 0x48, 0x45, 0x3, 0x2, 0x2, 0x2, 0x48, 0x46, 0x3, 0x2,
        0x2, 0x2, 0x48, 0x47, 0x3, 0x2, 0x2, 0x2, 0x49, 0x4c, 0x3, 0x2, 0x2,
        0x2, 0x4a, 0x48, 0x3, 0x2, 0x2, 0x2, 0x4a, 0x4b, 0x3, 0x2, 0x2, 0x2,
        0x4b, 0x4d, 0x3, 0x2, 0x2, 0x2, 0x4c, 0x4a, 0x3, 0x2, 0x2, 0x2, 0x4d,
        0x4e, 0x7, 0x2, 0x2, 0x3, 0x4e, 0x3, 0x3, 0x2, 0x2, 0x2, 0x4f, 0x53,
        0x5, 0x6, 0x4, 0x2, 0x50, 0x53, 0x5, 0x8, 0x5, 0x2, 0x51, 0x53, 0x5,
        0xc, 0x7, 0x2, 0x52, 0x4f, 0x3, 0x2, 0x2, 0x2, 0x52, 0x50, 0x3, 0x2,
        0x2, 0x2, 0x52, 0x51, 0x3, 0x2, 0x2, 0x2, 0x53, 0x5, 0x3, 0x2, 0x2,
        0x2, 0x54, 0x55, 0x7, 0x3, 0x2, 0x2, 0x55, 0x56, 0x5, 0x3e, 0x20, 0x2,
        0x56, 0x57, 0x5, 0x32, 0x1a, 0x2, 0x57, 0x58, 0x7, 0x2d, 0x2, 0x2, 0x58,
        0x59, 0x7, 0x16, 0x2, 0x2, 0x59, 0x7, 0x3, 0x2, 0x2, 0x2, 0x5a, 0x5b,
        0x7, 0x7, 0x2, 0x2, 0x5b, 0x5c, 0x7, 0x2d, 0x2, 0x2, 0x5c, 0x61, 0x5,
        0xa, 0x6, 0x2, 0x5d, 0x5e, 0x7, 0x22, 0x2, 0x2, 0x5e, 0x60, 0x5, 0xa,
        0x6, 0x2, 0x5f, 0x5d, 0x3, 0x2, 0x2, 0x2, 0x60, 0x63, 0x3, 0x2, 0x2,
        0x2, 0x61, 0x5f, 0x3, 0x2, 0x2, 0x2, 0x61, 0x62, 0x3, 0x2, 0x2, 0x2,
        0x62, 0x64, 0x3, 0x2, 0x2, 0x2, 0x63, 0x61, 0x3, 0x2, 0x2, 0x2, 0x64,
        0x65, 0x7, 0x16, 0x2, 0x2, 0x65, 0x9, 0x3, 0x2, 0x2, 0x2, 0x66, 0x67,
        0x7, 0x1b, 0x2, 0x2, 0x67, 0x6c, 0x7, 0x2f, 0x2, 0x2, 0x68, 0x69, 0x7,
        0x14, 0x2, 0x2, 0x69, 0x6b, 0x7, 0x2f, 0x2, 0x2, 0x6a, 0x68, 0x3, 0x2,
        0x2, 0x2, 0x6b, 0x6e, 0x3, 0x2, 0x2, 0x2, 0x6c, 0x6a, 0x3, 0x2, 0x2,
        0x2, 0x6c, 0x6d, 0x3, 0x2, 0x2, 0x2, 0x6d, 0x6f, 0x3, 0x2, 0x2, 0x2,
        0x6e, 0x6c, 0x3, 0x2, 0x2, 0x2, 0x6f, 0x72, 0x7, 0x1c, 0x2, 0x2, 0x70,
        0x72, 0x5, 0x3a, 0x1e, 0x2, 0x71, 0x66, 0x3, 0x2, 0x2, 0x2, 0x71, 0x70,
        0x3, 0x2, 0x2, 0x2, 0x72, 0xb, 0x3, 0x2, 0x2, 0x2, 0x73, 0x74, 0x7,
        0x8, 0x2, 0x2, 0x74, 0x77, 0x7, 0x2d, 0x2, 0x2, 0x75, 0x76, 0x7, 0x15,
        0x2, 0x2, 0x76, 0x78, 0x5, 0x40, 0x21, 0x2, 0x77, 0x75, 0x3, 0x2, 0x2,
        0x2, 0x77, 0x78, 0x3, 0x2, 0x2, 0x2, 0x78, 0x79, 0x3, 0x2, 0x2, 0x2,
        0x79, 0x7d, 0x7, 0x1b, 0x2, 0x2, 0x7a, 0x7c, 0x5, 0xe, 0x8, 0x2, 0x7b,
        0x7a, 0x3, 0x2, 0x2, 0x2, 0x7c, 0x7f, 0x3, 0x2, 0x2, 0x2, 0x7d, 0x7b,
        0x3, 0x2, 0x2, 0x2, 0x7d, 0x7e, 0x3, 0x2, 0x2, 0x2, 0x7e, 0x80, 0x3,
        0x2, 0x2, 0x2, 0x7f, 0x7d, 0x3, 0x2, 0x2, 0x2, 0x80, 0x81, 0x7, 0x1c,
        0x2, 0x2, 0x81, 0xd, 0x3, 0x2, 0x2, 0x2, 0x82, 0x88, 0x5, 0x10, 0x9,
        0x2, 0x83, 0x88, 0x5, 0x14, 0xb, 0x2, 0x84, 0x88, 0x5, 0x16, 0xc, 0x2,
        0x85, 0x88, 0x5, 0x1a, 0xe, 0x2, 0x86, 0x88, 0x5, 0x4, 0x3, 0x2, 0x87,
        0x82, 0x3, 0x2, 0x2, 0x2, 0x87, 0x83, 0x3, 0x2, 0x2, 0x2, 0x87, 0x84,
        0x3, 0x2, 0x2, 0x2, 0x87, 0x85, 0x3, 0x2, 0x2, 0x2, 0x87, 0x86, 0x3,
        0x2, 0x2, 0x2, 0x88, 0xf, 0x3, 0x2, 0x2, 0x2, 0x89, 0x8a, 0x5, 0x3a,
        0x1e, 0x2, 0x8a, 0x8f, 0x5, 0x12, 0xa, 0x2, 0x8b, 0x8c, 0x7, 0x14, 0x2,
        0x2, 0x8c, 0x8e, 0x5, 0x12, 0xa, 0x2, 0x8d, 0x8b, 0x3, 0x2, 0x2, 0x2,
        0x8e, 0x91, 0x3, 0x2, 0x2, 0x2, 0x8f, 0x8d, 0x3, 0x2, 0x2, 0x2, 0x8f,
        0x90, 0x3, 0x2, 0x2, 0x2, 0x90, 0x92, 0x3, 0x2, 0x2, 0x2, 0x91, 0x8f,
        0x3, 0x2, 0x2, 0x2, 0x92, 0x93, 0x7, 0x16, 0x2, 0x2, 0x93, 0x11, 0x3,
        0x2, 0x2, 0x2, 0x94, 0x97, 0x7, 0x2d, 0x2, 0x2, 0x95, 0x96, 0x7, 0x23,
        0x2, 0x2, 0x96, 0x98, 0x5, 0x32, 0x1a, 0x2, 0x97, 0x95, 0x3, 0x2, 0x2,
        0x2, 0x97, 0x98, 0x3, 0x2, 0x2, 0x2, 0x98, 0x13, 0x3, 0x2, 0x2, 0x2,
        0x99, 0x9a, 0x7, 0xf, 0x2, 0x2, 0x9a, 0x9b, 0x7, 0x2d, 0x2, 0x2, 0x9b,
        0x9d, 0x7, 0x17, 0x2, 0x2, 0x9c, 0x9e, 0x5, 0x42, 0x22, 0x2, 0x9d, 0x9c,
        0x3, 0x2, 0x2, 0x2, 0x9d, 0x9e, 0x3, 0x2, 0x2, 0x2, 0x9e, 0x9f, 0x3,
        0x2, 0x2, 0x2, 0x9f, 0xa0, 0x7, 0x18, 0x2, 0x2, 0xa0, 0xa1, 0x7, 0x1b,
        0x2, 0x2, 0xa1, 0xa2, 0x5, 0x1e, 0x10, 0x2, 0xa2, 0xa3, 0x7, 0x1c, 0x2,
        0x2, 0xa3, 0xb0, 0x3, 0x2, 0x2, 0x2, 0xa4, 0xa5, 0x5, 0x3a, 0x1e, 0x2,
        0xa5, 0xa6, 0x7, 0x2d, 0x2, 0x2, 0xa6, 0xa8, 0x7, 0x17, 0x2, 0x2, 0xa7,
        0xa9, 0x5, 0x42, 0x22, 0x2, 0xa8, 0xa7, 0x3, 0x2, 0x2, 0x2, 0xa8, 0xa9,
        0x3, 0x2, 0x2, 0x2, 0xa9, 0xaa, 0x3, 0x2, 0x2, 0x2, 0xaa, 0xab, 0x7,
        0x18, 0x2, 0x2, 0xab, 0xac, 0x7, 0x1b, 0x2, 0x2, 0xac, 0xad, 0x5, 0x1e,
        0x10, 0x2, 0xad, 0xae, 0x7, 0x1c, 0x2, 0x2, 0xae, 0xb0, 0x3, 0x2, 0x2,
        0x2, 0xaf, 0x99, 0x3, 0x2, 0x2, 0x2, 0xaf, 0xa4, 0x3, 0x2, 0x2, 0x2,
        0xb0, 0x15, 0x3, 0x2, 0x2, 0x2, 0xb1, 0xb2, 0x7, 0x2d, 0x2, 0x2, 0xb2,
        0xb4, 0x7, 0x17, 0x2, 0x2, 0xb3, 0xb5, 0x5, 0x42, 0x22, 0x2, 0xb4, 0xb3,
        0x3, 0x2, 0x2, 0x2, 0xb4, 0xb5, 0x3, 0x2, 0x2, 0x2, 0xb5, 0xb6, 0x3,
        0x2, 0x2, 0x2, 0xb6, 0xc0, 0x7, 0x18, 0x2, 0x2, 0xb7, 0xb8, 0x7, 0x15,
        0x2, 0x2, 0xb8, 0xbd, 0x5, 0x18, 0xd, 0x2, 0xb9, 0xba, 0x7, 0x14, 0x2,
        0x2, 0xba, 0xbc, 0x5, 0x18, 0xd, 0x2, 0xbb, 0xb9, 0x3, 0x2, 0x2, 0x2,
        0xbc, 0xbf, 0x3, 0x2, 0x2, 0x2, 0xbd, 0xbb, 0x3, 0x2, 0x2, 0x2, 0xbd,
        0xbe, 0x3, 0x2, 0x2, 0x2, 0xbe, 0xc1, 0x3, 0x2, 0x2, 0x2, 0xbf, 0xbd,
        0x3, 0x2, 0x2, 0x2, 0xc0, 0xb7, 0x3, 0x2, 0x2, 0x2, 0xc0, 0xc1, 0x3,
        0x2, 0x2, 0x2, 0xc1, 0xc2, 0x3, 0x2, 0x2, 0x2, 0xc2, 0xc3, 0x7, 0x1b,
        0x2, 0x2, 0xc3, 0xc4, 0x5, 0x1e, 0x10, 0x2, 0xc4, 0xc5, 0x7, 0x1c, 0x2,
        0x2, 0xc5, 0x17, 0x3, 0x2, 0x2, 0x2, 0xc6, 0xc7, 0x7, 0x2d, 0x2, 0x2,
        0xc7, 0xc9, 0x7, 0x17, 0x2, 0x2, 0xc8, 0xca, 0x5, 0x34, 0x1b, 0x2, 0xc9,
        0xc8, 0x3, 0x2, 0x2, 0x2, 0xc9, 0xca, 0x3, 0x2, 0x2, 0x2, 0xca, 0xcb,
        0x3, 0x2, 0x2, 0x2, 0xcb, 0xcc, 0x7, 0x18, 0x2, 0x2, 0xcc, 0x19, 0x3,
        0x2, 0x2, 0x2, 0xcd, 0xce, 0x7, 0xb, 0x2, 0x2, 0xce, 0xcf, 0x7, 0x2d,
        0x2, 0x2, 0xcf, 0xd1, 0x7, 0x17, 0x2, 0x2, 0xd0, 0xd2, 0x5, 0x42, 0x22,
        0x2, 0xd1, 0xd0, 0x3, 0x2, 0x2, 0x2, 0xd1, 0xd2, 0x3, 0x2, 0x2, 0x2,
        0xd2, 0xd3, 0x3, 0x2, 0x2, 0x2, 0xd3, 0xd6, 0x7, 0x18, 0x2, 0x2, 0xd4,
        0xd5, 0x7, 0x15, 0x2, 0x2, 0xd5, 0xd7, 0x5, 0x40, 0x21, 0x2, 0xd6, 0xd4,
        0x3, 0x2, 0x2, 0x2, 0xd6, 0xd7, 0x3, 0x2, 0x2, 0x2, 0xd7, 0xd8, 0x3,
        0x2, 0x2, 0x2, 0xd8, 0xd9, 0x7, 0x1b, 0x2, 0x2, 0xd9, 0xda, 0x5, 0x1e,
        0x10, 0x2, 0xda, 0xdb, 0x7, 0x1c, 0x2, 0x2, 0xdb, 0x1b, 0x3, 0x2, 0x2,
        0x2, 0xdc, 0xe7, 0x5, 0x20, 0x11, 0x2, 0xdd, 0xe7, 0x5, 0x22, 0x12,
        0x2, 0xde, 0xe7, 0x5, 0x24, 0x13, 0x2, 0xdf, 0xe7, 0x5, 0x26, 0x14,
        0x2, 0xe0, 0xe7, 0x5, 0x2a, 0x16, 0x2, 0xe1, 0xe7, 0x5, 0x2c, 0x17,
        0x2, 0xe2, 0xe3, 0x7, 0x1b, 0x2, 0x2, 0xe3, 0xe4, 0x5, 0x1e, 0x10, 0x2,
        0xe4, 0xe5, 0x7, 0x1c, 0x2, 0x2, 0xe5, 0xe7, 0x3, 0x2, 0x2, 0x2, 0xe6,
        0xdc, 0x3, 0x2, 0x2, 0x2, 0xe6, 0xdd, 0x3, 0x2, 0x2, 0x2, 0xe6, 0xde,
        0x3, 0x2, 0x2, 0x2, 0xe6, 0xdf, 0x3, 0x2, 0x2, 0x2, 0xe6, 0xe0, 0x3,
        0x2, 0x2, 0x2, 0xe6, 0xe1, 0x3, 0x2, 0x2, 0x2, 0xe6, 0xe2, 0x3, 0x2,
        0x2, 0x2, 0xe7, 0x1d, 0x3, 0x2, 0x2, 0x2, 0xe8, 0xea, 0x5, 0x1c, 0xf,
        0x2, 0xe9, 0xe8, 0x3, 0x2, 0x2, 0x2, 0xea, 0xed, 0x3, 0x2, 0x2, 0x2,
        0xeb, 0xe9, 0x3, 0x2, 0x2, 0x2, 0xeb, 0xec, 0x3, 0x2, 0x2, 0x2, 0xec,
        0x1f, 0x3, 0x2, 0x2, 0x2, 0xed, 0xeb, 0x3, 0x2, 0x2, 0x2, 0xee, 0xef,
        0x5, 0x38, 0x1d, 0x2, 0xef, 0xf0, 0x7, 0x13, 0x2, 0x2, 0xf0, 0xf2, 0x3,
        0x2, 0x2, 0x2, 0xf1, 0xee, 0x3, 0x2, 0x2, 0x2, 0xf1, 0xf2, 0x3, 0x2,
        0x2, 0x2, 0xf2, 0xf3, 0x3, 0x2, 0x2, 0x2, 0xf3, 0xf4, 0x7, 0x2d, 0x2,
        0x2, 0xf4, 0xf5, 0x7, 0x23, 0x2, 0x2, 0xf5, 0xf6, 0x5, 0x32, 0x1a, 0x2,
        0xf6, 0xf7, 0x7, 0x16, 0x2, 0x2, 0xf7, 0x21, 0x3, 0x2, 0x2, 0x2, 0xf8,
        0xf9, 0x5, 0x3a, 0x1e, 0x2, 0xf9, 0xfe, 0x5, 0x12, 0xa, 0x2, 0xfa, 0xfb,
        0x7, 0x14, 0x2, 0x2, 0xfb, 0xfd, 0x5, 0x12, 0xa, 0x2, 0xfc, 0xfa, 0x3,
        0x2, 0x2, 0x2, 0xfd, 0x100, 0x3, 0x2, 0x2, 0x2, 0xfe, 0xfc, 0x3, 0x2,
        0x2, 0x2, 0xfe, 0xff, 0x3, 0x2, 0x2, 0x2, 0xff, 0x101, 0x3, 0x2, 0x2,
        0x2, 0x100, 0xfe, 0x3, 0x2, 0x2, 0x2, 0x101, 0x102, 0x7, 0x16, 0x2,
        0x2, 0x102, 0x23, 0x3, 0x2, 0x2, 0x2, 0x103, 0x104, 0x5, 0x32, 0x1a,
        0x2, 0x104, 0x105, 0x7, 0x16, 0x2, 0x2, 0x105, 0x25, 0x3, 0x2, 0x2,
        0x2, 0x106, 0x109, 0x5, 0x28, 0x15, 0x2, 0x107, 0x108, 0x7, 0xd, 0x2,
        0x2, 0x108, 0x10a, 0x5, 0x28, 0x15, 0x2, 0x109, 0x107, 0x3, 0x2, 0x2,
        0x2, 0x10a, 0x10b, 0x3, 0x2, 0x2, 0x2, 0x10b, 0x109, 0x3, 0x2, 0x2,
        0x2, 0x10b, 0x10c, 0x3, 0x2, 0x2, 0x2, 0x10c, 0x27, 0x3, 0x2, 0x2, 0x2,
        0x10d, 0x10e, 0x7, 0x1b, 0x2, 0x2, 0x10e, 0x10f, 0x5, 0x1e, 0x10, 0x2,
        0x10f, 0x114, 0x7, 0x1c, 0x2, 0x2, 0x110, 0x111, 0x7, 0x19, 0x2, 0x2,
        0x111, 0x112, 0x5, 0x32, 0x1a, 0x2, 0x112, 0x113, 0x7, 0x1a, 0x2, 0x2,
        0x113, 0x115, 0x3, 0x2, 0x2, 0x2, 0x114, 0x110, 0x3, 0x2, 0x2, 0x2,
        0x114, 0x115, 0x3, 0x2, 0x2, 0x2, 0x115, 0x29, 0x3, 0x2, 0x2, 0x2, 0x116,
        0x119, 0x7, 0x9, 0x2, 0x2, 0x117, 0x119, 0x7, 0xa, 0x2, 0x2, 0x118,
        0x116, 0x3, 0x2, 0x2, 0x2, 0x118, 0x117, 0x3, 0x2, 0x2, 0x2, 0x119,
        0x11a, 0x3, 0x2, 0x2, 0x2, 0x11a, 0x11b, 0x7, 0x2d, 0x2, 0x2, 0x11b,
        0x11c, 0x7, 0x23, 0x2, 0x2, 0x11c, 0x120, 0x7, 0xc, 0x2, 0x2, 0x11d,
        0x11e, 0x5, 0x38, 0x1d, 0x2, 0x11e, 0x11f, 0x7, 0x13, 0x2, 0x2, 0x11f,
        0x121, 0x3, 0x2, 0x2, 0x2, 0x120, 0x11d, 0x3, 0x2, 0x2, 0x2, 0x120,
        0x121, 0x3, 0x2, 0x2, 0x2, 0x121, 0x122, 0x3, 0x2, 0x2, 0x2, 0x122,
        0x123, 0x7, 0x2d, 0x2, 0x2, 0x123, 0x125, 0x7, 0x17, 0x2, 0x2, 0x124,
        0x126, 0x5, 0x2e, 0x18, 0x2, 0x125, 0x124, 0x3, 0x2, 0x2, 0x2, 0x125,
        0x126, 0x3, 0x2, 0x2, 0x2, 0x126, 0x127, 0x3, 0x2, 0x2, 0x2, 0x127,
        0x128, 0x7, 0x18, 0x2, 0x2, 0x128, 0x129, 0x7, 0x16, 0x2, 0x2, 0x129,
        0x2b, 0x3, 0x2, 0x2, 0x2, 0x12a, 0x12b, 0x7, 0x12, 0x2, 0x2, 0x12b,
        0x12c, 0x5, 0x32, 0x1a, 0x2, 0x12c, 0x12d, 0x7, 0x16, 0x2, 0x2, 0x12d,
        0x2d, 0x3, 0x2, 0x2, 0x2, 0x12e, 0x133, 0x5, 0x30, 0x19, 0x2, 0x12f,
        0x130, 0x7, 0x14, 0x2, 0x2, 0x130, 0x132, 0x5, 0x30, 0x19, 0x2, 0x131,
        0x12f, 0x3, 0x2, 0x2, 0x2, 0x132, 0x135, 0x3, 0x2, 0x2, 0x2, 0x133,
        0x131, 0x3, 0x2, 0x2, 0x2, 0x133, 0x134, 0x3, 0x2, 0x2, 0x2, 0x134,
        0x2f, 0x3, 0x2, 0x2, 0x2, 0x135, 0x133, 0x3, 0x2, 0x2, 0x2, 0x136, 0x137,
        0x7, 0x2d, 0x2, 0x2, 0x137, 0x138, 0x7, 0x15, 0x2, 0x2, 0x138, 0x139,
        0x5, 0x32, 0x1a, 0x2, 0x139, 0x31, 0x3, 0x2, 0x2, 0x2, 0x13a, 0x13b,
        0x8, 0x1a, 0x1, 0x2, 0x13b, 0x166, 0x5, 0x36, 0x1c, 0x2, 0x13c, 0x13d,
        0x7, 0x17, 0x2, 0x2, 0x13d, 0x13e, 0x5, 0x32, 0x1a, 0x2, 0x13e, 0x13f,
        0x7, 0x18, 0x2, 0x2, 0x13f, 0x166, 0x3, 0x2, 0x2, 0x2, 0x140, 0x141,
        0x7, 0x1d, 0x2, 0x2, 0x141, 0x166, 0x5, 0x32, 0x1a, 0x14, 0x142, 0x143,
        0x7, 0x1e, 0x2, 0x2, 0x143, 0x166, 0x5, 0x32, 0x1a, 0x13, 0x144, 0x145,
        0x7, 0x26, 0x2, 0x2, 0x145, 0x166, 0x5, 0x32, 0x1a, 0x12, 0x146, 0x166,
        0x5, 0x38, 0x1d, 0x2, 0x147, 0x148, 0x5, 0x38, 0x1d, 0x2, 0x148, 0x149,
        0x7, 0x13, 0x2, 0x2, 0x149, 0x14b, 0x3, 0x2, 0x2, 0x2, 0x14a, 0x147,
        0x3, 0x2, 0x2, 0x2, 0x14a, 0x14b, 0x3, 0x2, 0x2, 0x2, 0x14b, 0x14c,
        0x3, 0x2, 0x2, 0x2, 0x14c, 0x14d, 0x7, 0x2d, 0x2, 0x2, 0x14d, 0x14f,
        0x7, 0x17, 0x2, 0x2, 0x14e, 0x150, 0x5, 0x34, 0x1b, 0x2, 0x14f, 0x14e,
        0x3, 0x2, 0x2, 0x2, 0x14f, 0x150, 0x3, 0x2, 0x2, 0x2, 0x150, 0x151,
        0x3, 0x2, 0x2, 0x2, 0x151, 0x166, 0x7, 0x18, 0x2, 0x2, 0x152, 0x153,
        0x7, 0x17, 0x2, 0x2, 0x153, 0x154, 0x5, 0x3a, 0x1e, 0x2, 0x154, 0x155,
        0x7, 0x18, 0x2, 0x2, 0x155, 0x156, 0x5, 0x32, 0x1a, 0xf, 0x156, 0x166,
        0x3, 0x2, 0x2, 0x2, 0x157, 0x158, 0x7, 0x19, 0x2, 0x2, 0x158, 0x159,
        0x5, 0x32, 0x1a, 0x2, 0x159, 0x15a, 0x7, 0x14, 0x2, 0x2, 0x15a, 0x15b,
        0x5, 0x32, 0x1a, 0x2, 0x15b, 0x15c, 0x7, 0x1a, 0x2, 0x2, 0x15c, 0x166,
        0x3, 0x2, 0x2, 0x2, 0x15d, 0x15e, 0x7, 0xc, 0x2, 0x2, 0x15e, 0x15f,
        0x5, 0x3a, 0x1e, 0x2, 0x15f, 0x161, 0x7, 0x17, 0x2, 0x2, 0x160, 0x162,
        0x5, 0x34, 0x1b, 0x2, 0x161, 0x160, 0x3, 0x2, 0x2, 0x2, 0x161, 0x162,
        0x3, 0x2, 0x2, 0x2, 0x162, 0x163, 0x3, 0x2, 0x2, 0x2, 0x163, 0x164,
        0x7, 0x18, 0x2, 0x2, 0x164, 0x166, 0x3, 0x2, 0x2, 0x2, 0x165, 0x13a,
        0x3, 0x2, 0x2, 0x2, 0x165, 0x13c, 0x3, 0x2, 0x2, 0x2, 0x165, 0x140,
        0x3, 0x2, 0x2, 0x2, 0x165, 0x142, 0x3, 0x2, 0x2, 0x2, 0x165, 0x144,
        0x3, 0x2, 0x2, 0x2, 0x165, 0x146, 0x3, 0x2, 0x2, 0x2, 0x165, 0x14a,
        0x3, 0x2, 0x2, 0x2, 0x165, 0x152, 0x3, 0x2, 0x2, 0x2, 0x165, 0x157,
        0x3, 0x2, 0x2, 0x2, 0x165, 0x15d, 0x3, 0x2, 0x2, 0x2, 0x166, 0x1ab,
        0x3, 0x2, 0x2, 0x2, 0x167, 0x168, 0xc, 0x17, 0x2, 0x2, 0x168, 0x169,
        0x7, 0x20, 0x2, 0x2, 0x169, 0x1aa, 0x5, 0x32, 0x1a, 0x18, 0x16a, 0x16b,
        0xc, 0xc, 0x2, 0x2, 0x16b, 0x16c, 0x7, 0x27, 0x2, 0x2, 0x16c, 0x1aa,
        0x5, 0x32, 0x1a, 0xd, 0x16d, 0x16e, 0xc, 0xb, 0x2, 0x2, 0x16e, 0x16f,
        0x7, 0x29, 0x2, 0x2, 0x16f, 0x1aa, 0x5, 0x32, 0x1a, 0xc, 0x170, 0x171,
        0xc, 0xa, 0x2, 0x2, 0x171, 0x172, 0x7, 0x28, 0x2, 0x2, 0x172, 0x1aa,
        0x5, 0x32, 0x1a, 0xb, 0x173, 0x174, 0xc, 0x9, 0x2, 0x2, 0x174, 0x175,
        0x7, 0x24, 0x2, 0x2, 0x175, 0x1aa, 0x5, 0x32, 0x1a, 0xa, 0x176, 0x177,
        0xc, 0x8, 0x2, 0x2, 0x177, 0x178, 0x7, 0x25, 0x2, 0x2, 0x178, 0x1aa,
        0x5, 0x32, 0x1a, 0x9, 0x179, 0x17a, 0xc, 0x7, 0x2, 0x2, 0x17a, 0x17b,
        0x7, 0x2a, 0x2, 0x2, 0x17b, 0x1aa, 0x5, 0x32, 0x1a, 0x8, 0x17c, 0x17d,
        0xc, 0x6, 0x2, 0x2, 0x17d, 0x17e, 0x7, 0x2b, 0x2, 0x2, 0x17e, 0x1aa,
        0x5, 0x32, 0x1a, 0x7, 0x17f, 0x182, 0xc, 0x18, 0x2, 0x2, 0x180, 0x181,
        0x7, 0x1f, 0x2, 0x2, 0x181, 0x183, 0x5, 0x32, 0x1a, 0x2, 0x182, 0x180,
        0x3, 0x2, 0x2, 0x2, 0x183, 0x184, 0x3, 0x2, 0x2, 0x2, 0x184, 0x182,
        0x3, 0x2, 0x2, 0x2, 0x184, 0x185, 0x3, 0x2, 0x2, 0x2, 0x185, 0x1aa,
        0x3, 0x2, 0x2, 0x2, 0x186, 0x189, 0xc, 0x16, 0x2, 0x2, 0x187, 0x188,
        0x7, 0x1d, 0x2, 0x2, 0x188, 0x18a, 0x5, 0x32, 0x1a, 0x2, 0x189, 0x187,
        0x3, 0x2, 0x2, 0x2, 0x18a, 0x18b, 0x3, 0x2, 0x2, 0x2, 0x18b, 0x189,
        0x3, 0x2, 0x2, 0x2, 0x18b, 0x18c, 0x3, 0x2, 0x2, 0x2, 0x18c, 0x1aa,
        0x3, 0x2, 0x2, 0x2, 0x18d, 0x190, 0xc, 0x15, 0x2, 0x2, 0x18e, 0x18f,
        0x7, 0x1e, 0x2, 0x2, 0x18f, 0x191, 0x5, 0x32, 0x1a, 0x2, 0x190, 0x18e,
        0x3, 0x2, 0x2, 0x2, 0x191, 0x192, 0x3, 0x2, 0x2, 0x2, 0x192, 0x190,
        0x3, 0x2, 0x2, 0x2, 0x192, 0x193, 0x3, 0x2, 0x2, 0x2, 0x193, 0x1aa,
        0x3, 0x2, 0x2, 0x2, 0x194, 0x197, 0xc, 0x5, 0x2, 0x2, 0x195, 0x196,
        0x7, 0x22, 0x2, 0x2, 0x196, 0x198, 0x5, 0x32, 0x1a, 0x2, 0x197, 0x195,
        0x3, 0x2, 0x2, 0x2, 0x198, 0x199, 0x3, 0x2, 0x2, 0x2, 0x199, 0x197,
        0x3, 0x2, 0x2, 0x2, 0x199, 0x19a, 0x3, 0x2, 0x2, 0x2, 0x19a, 0x1aa,
        0x3, 0x2, 0x2, 0x2, 0x19b, 0x19e, 0xc, 0x4, 0x2, 0x2, 0x19c, 0x19d,
        0x7, 0x21, 0x2, 0x2, 0x19d, 0x19f, 0x5, 0x32, 0x1a, 0x2, 0x19e, 0x19c,
        0x3, 0x2, 0x2, 0x2, 0x19f, 0x1a0, 0x3, 0x2, 0x2, 0x2, 0x1a0, 0x19e,
        0x3, 0x2, 0x2, 0x2, 0x1a0, 0x1a1, 0x3, 0x2, 0x2, 0x2, 0x1a1, 0x1aa,
        0x3, 0x2, 0x2, 0x2, 0x1a2, 0x1a5, 0xc, 0x3, 0x2, 0x2, 0x1a3, 0x1a4,
        0x7, 0x2c, 0x2, 0x2, 0x1a4, 0x1a6, 0x5, 0x32, 0x1a, 0x2, 0x1a5, 0x1a3,
        0x3, 0x2, 0x2, 0x2, 0x1a6, 0x1a7, 0x3, 0x2, 0x2, 0x2, 0x1a7, 0x1a5,
        0x3, 0x2, 0x2, 0x2, 0x1a7, 0x1a8, 0x3, 0x2, 0x2, 0x2, 0x1a8, 0x1aa,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x167, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x16a,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x16d, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x170,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x173, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x176,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x179, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x17c,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x17f, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x186,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x18d, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x194,
        0x3, 0x2, 0x2, 0x2, 0x1a9, 0x19b, 0x3, 0x2, 0x2, 0x2, 0x1a9, 0x1a2,
        0x3, 0x2, 0x2, 0x2, 0x1aa, 0x1ad, 0x3, 0x2, 0x2, 0x2, 0x1ab, 0x1a9,
        0x3, 0x2, 0x2, 0x2, 0x1ab, 0x1ac, 0x3, 0x2, 0x2, 0x2, 0x1ac, 0x33, 0x3,
        0x2, 0x2, 0x2, 0x1ad, 0x1ab, 0x3, 0x2, 0x2, 0x2, 0x1ae, 0x1b3, 0x5,
        0x32, 0x1a, 0x2, 0x1af, 0x1b0, 0x7, 0x14, 0x2, 0x2, 0x1b0, 0x1b2, 0x5,
        0x32, 0x1a, 0x2, 0x1b1, 0x1af, 0x3, 0x2, 0x2, 0x2, 0x1b2, 0x1b5, 0x3,
        0x2, 0x2, 0x2, 0x1b3, 0x1b1, 0x3, 0x2, 0x2, 0x2, 0x1b3, 0x1b4, 0x3,
        0x2, 0x2, 0x2, 0x1b4, 0x35, 0x3, 0x2, 0x2, 0x2, 0x1b5, 0x1b3, 0x3, 0x2,
        0x2, 0x2, 0x1b6, 0x1bb, 0x7, 0x2e, 0x2, 0x2, 0x1b7, 0x1bb, 0x7, 0x2f,
        0x2, 0x2, 0x1b8, 0x1bb, 0x7, 0x10, 0x2, 0x2, 0x1b9, 0x1bb, 0x7, 0x11,
        0x2, 0x2, 0x1ba, 0x1b6, 0x3, 0x2, 0x2, 0x2, 0x1ba, 0x1b7, 0x3, 0x2,
        0x2, 0x2, 0x1ba, 0x1b8, 0x3, 0x2, 0x2, 0x2, 0x1ba, 0x1b9, 0x3, 0x2,
        0x2, 0x2, 0x1bb, 0x37, 0x3, 0x2, 0x2, 0x2, 0x1bc, 0x1bf, 0x7, 0xe, 0x2,
        0x2, 0x1bd, 0x1bf, 0x7, 0x2d, 0x2, 0x2, 0x1be, 0x1bc, 0x3, 0x2, 0x2,
        0x2, 0x1be, 0x1bd, 0x3, 0x2, 0x2, 0x2, 0x1bf, 0x1c4, 0x3, 0x2, 0x2,
        0x2, 0x1c0, 0x1c1, 0x7, 0x13, 0x2, 0x2, 0x1c1, 0x1c3, 0x7, 0x2d, 0x2,
        0x2, 0x1c2, 0x1c0, 0x3, 0x2, 0x2, 0x2, 0x1c3, 0x1c6, 0x3, 0x2, 0x2,
        0x2, 0x1c4, 0x1c2, 0x3, 0x2, 0x2, 0x2, 0x1c4, 0x1c5, 0x3, 0x2, 0x2,
        0x2, 0x1c5, 0x39, 0x3, 0x2, 0x2, 0x2, 0x1c6, 0x1c4, 0x3, 0x2, 0x2, 0x2,
        0x1c7, 0x1ca, 0x5, 0x3c, 0x1f, 0x2, 0x1c8, 0x1ca, 0x5, 0x3e, 0x20, 0x2,
        0x1c9, 0x1c7, 0x3, 0x2, 0x2, 0x2, 0x1c9, 0x1c8, 0x3, 0x2, 0x2, 0x2,
        0x1ca, 0x3b, 0x3, 0x2, 0x2, 0x2, 0x1cb, 0x1d0, 0x7, 0x2d, 0x2, 0x2,
        0x1cc, 0x1cd, 0x7, 0x13, 0x2, 0x2, 0x1cd, 0x1cf, 0x7, 0x2d, 0x2, 0x2,
        0x1ce, 0x1cc, 0x3, 0x2, 0x2, 0x2, 0x1cf, 0x1d2, 0x3, 0x2, 0x2, 0x2,
        0x1d0, 0x1ce, 0x3, 0x2, 0x2, 0x2, 0x1d0, 0x1d1, 0x3, 0x2, 0x2, 0x2,
        0x1d1, 0x3d, 0x3, 0x2, 0x2, 0x2, 0x1d2, 0x1d0, 0x3, 0x2, 0x2, 0x2, 0x1d3,
        0x1d4, 0x9, 0x2, 0x2, 0x2, 0x1d4, 0x3f, 0x3, 0x2, 0x2, 0x2, 0x1d5, 0x1da,
        0x5, 0x3a, 0x1e, 0x2, 0x1d6, 0x1d7, 0x7, 0x14, 0x2, 0x2, 0x1d7, 0x1d9,
        0x5, 0x3a, 0x1e, 0x2, 0x1d8, 0x1d6, 0x3, 0x2, 0x2, 0x2, 0x1d9, 0x1dc,
        0x3, 0x2, 0x2, 0x2, 0x1da, 0x1d8, 0x3, 0x2, 0x2, 0x2, 0x1da, 0x1db,
        0x3, 0x2, 0x2, 0x2, 0x1db, 0x41, 0x3, 0x2, 0x2, 0x2, 0x1dc, 0x1da, 0x3,
        0x2, 0x2, 0x2, 0x1dd, 0x1de, 0x5, 0x3a, 0x1e, 0x2, 0x1de, 0x1e5, 0x7,
        0x2d, 0x2, 0x2, 0x1df, 0x1e0, 0x7, 0x14, 0x2, 0x2, 0x1e0, 0x1e1, 0x5,
        0x3a, 0x1e, 0x2, 0x1e1, 0x1e2, 0x7, 0x2d, 0x2, 0x2, 0x1e2, 0x1e4, 0x3,
        0x2, 0x2, 0x2, 0x1e3, 0x1df, 0x3, 0x2, 0x2, 0x2, 0x1e4, 0x1e7, 0x3,
        0x2, 0x2, 0x2, 0x1e5, 0x1e3, 0x3, 0x2, 0x2, 0x2, 0x1e5, 0x1e6, 0x3,
        0x2, 0x2, 0x2, 0x1e6, 0x43, 0x3, 0x2, 0x2, 0x2, 0x1e7, 0x1e5, 0x3, 0x2,
        0x2, 0x2, 0x34, 0x48, 0x4a, 0x52, 0x61, 0x6c, 0x71, 0x77, 0x7d, 0x87,
        0x8f, 0x97, 0x9d, 0xa8, 0xaf, 0xb4, 0xbd, 0xc0, 0xc9, 0xd1, 0xd6, 0xe6,
        0xeb, 0xf1, 0xfe, 0x10b, 0x114, 0x118, 0x120, 0x125, 0x133, 0x14a, 0x14f,
        0x161, 0x165, 0x184, 0x18b, 0x192, 0x199, 0x1a0, 0x1a7, 0x1a9, 0x1ab,
        0x1b3, 0x1ba, 0x1be, 0x1c4, 0x1c9, 0x1d0, 0x1da, 0x1e5,
    };

    atn::ATNDeserializer deserializer;
    _atn = deserializer.deserialize(_serializedATN);

    size_t count = _atn.getNumberOfDecisions();
    _decisionToDFA.reserve(count);
    for (size_t i = 0; i < count; i++) {
        _decisionToDFA.emplace_back(_atn.getDecisionState(i), i);
    }
}

oRatioParser::Initializer oRatioParser::_init;
