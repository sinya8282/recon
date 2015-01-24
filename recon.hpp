#ifndef RECON_H_
#define RECON_H_

// Copyright Ryoma Sin'ya, 2012. All Rights Reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <bitset>
#include <deque>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <exception>
#include <cassert>
#include <math.h>

namespace recon {

const std::string SYNTAX = 
"RECON extended regular expression syntax:                    \n"
"  regex      ::= union* EOP                                  \n"
"  union      ::= concat ('|' concat)*                        \n"
"  concat     ::= repetition+                                 \n"
"  repetition ::= atom quantifier*                            \n"
"  quantifier ::= [*+?] | '{' (\\d+ | \\d* ',' \\d* ) '}'     \n"
"  negation   ::= '!' atom                                    \n"
"  emptyset   ::= '@'                                           "
"  atom       ::= literal | dot | charclass | '(' union ')'   \n"
"               | utf8char # optional (--utf8)                \n"
"               | negation | emptyset                         \n"
"  charclass  ::= '[' ']'? [^]]* ']'                          \n"
"  literal    ::= [^*+?[\\]|]                                 \n"
"  dot        ::= '.' # NOTE: dot matchs also newline('\\n')  \n"
"  utf8char   ::= [\\x00-\\x7f] | [\\xC0-\\xDF][\\x80-\\xBF]  \n"
"               | [\\xE0-\\xEF][\\x80-\\xBF]{2}               \n"
"               | [\\xF0-\\xF7][\\x80-\\xBF]{3}               \n";

enum Encoding {
  ASCII = 0, // default
  UTF8 = 1
};

unsigned char opposite_case(const unsigned char c) {
  if ('a' <= c && c <= 'z') {
    return c - 'a' + 'A';
  } else if ('A' <= c && c <= 'Z') {
    return c - 'A' + 'a';
  } else {
    return c;
  }
}

std::size_t utf8_byte_length(const unsigned char c)
{
  static const std::size_t len[256] = {
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    4,4,4,4,4,4,4,4,
    0,0,0,0,0,0,0,0
  };

  return len[c];
}

bool is_valid_utf8_sequence(const unsigned char *s)
{
  std::size_t len = utf8_byte_length(*s);
  if (len == 0) return false;

  for (std::size_t i = 1; i < len; i++) {
    if (!(0x80 <= *(s+i) && *(s+i) <= 0xBF)) return false;
  }

  return true;
}

class Parser {
  friend class RECON;
 public:
  enum ExprType {
    kLiteral = 0, kDot, kCharClass, kUTF8,
    kConcat, kUnion,
    kStar, kPlus, kRepetition, kQmark,
    kEOP, kLpar, kRpar, kByteRange, kEpsilon,
    kNegation, kEmptySet, kBadExpr
  };
  struct Expr {
    Expr() { type = kEpsilon; lhs = rhs = 0; }
    Expr(ExprType t, Expr* lhs = NULL, Expr* rhs = NULL) { init(t, lhs, rhs); }
    void init(ExprType, Expr*, Expr*);
    void set_union(std::set<Expr*> &src1, std::set<Expr*> &src2, std::set<Expr*> &dst)
    { dst.insert(src1.begin(), src1.end()); dst.insert(src2.begin(), src2.end()); }
    const char* type_name();

    // fiesds
    ExprType type;
    bool nullable;
    unsigned char literal;
    std::bitset<256> cc_table;
    Expr* lhs;
    Expr* rhs;
    std::size_t id;
    std::set<Expr*> follow;
    std::set<Expr*> first;
    std::set<Expr*> last;
    void dump(std::size_t tab);
    friend std::ostream& operator<<(std::ostream&, Expr&);
  };
  
  Parser(const std::string &, Encoding);
  Expr* expr_root() { return _expr_root; }
  Expr* expr(std::size_t);
  Expr* new_expr(ExprType, Expr*, Expr*);
  Expr* clone_expr(Expr*);
  bool ok() const { return _ok; }
  const std::string& error() const { return _error; }
 private:
  ExprType consume();
  ExprType lex();
  std::size_t offset() const { return _regex_end - _regex_ptr; }
  unsigned char consume_char();
  int consume_int();
  bool lex_is_int() { return '0' <= lex_char() && lex_char() <= '9'; }
  unsigned char lex_char();
  bool regex_empty() { return _regex_ptr == _regex_end; }
  bool lex_is_atom();
  bool lex_is_quantifier();
  ExprType consume_repetition();
  ExprType consume_metachar();
  bool metachar() const { return _metachar; }
  void metachar(bool m) { _metachar = m; }

  void parse();
  Expr* parse_union();
  Expr* parse_concat();
  Expr* parse_repetition();
  Expr* parse_atom();
  Expr* parse_charclass();

  void copy_expr(Expr *);
  void fill_transition(Expr *);
  void fill_pos(Expr *);
  void connect(std::set<Expr*>, std::set<Expr*>);

  // fields
  static const int repeat_infinitely = -1;
  bool _ok;
  std::string _error;
  std::string _regex;
  Encoding _encoding;
  const unsigned char* _regex_begin;
  const unsigned char* _regex_end;
  const unsigned char* _regex_ptr;
  std::deque<Expr> _expr_tree;
  Expr* _expr_root;
  std::bitset<256> _cc_table;
  unsigned char _literal;
  int _repeat_min, _repeat_max;
  bool _metachar;
  ExprType _token;
};

Parser::Expr* Parser::expr(std::size_t index)
{
  return &_expr_tree[index];
}

void Parser::Expr::init(ExprType t, Expr* lhs_ = NULL, Expr* rhs_ = NULL)
{
  type = t;
  lhs = lhs_;
  rhs = rhs_;

  switch (type) {
    case kLiteral: case kDot: case kCharClass: case kEOP: {
      nullable = false;
      first.insert(this);
      last.insert(this);
      break;
    }
    case kUnion: {
      nullable = lhs->nullable || rhs->nullable;
      set_union(lhs->first, rhs->first, first);
      set_union(lhs->last, rhs->last, last);
      break;
    }
    case kConcat: {
      nullable = lhs->nullable && rhs->nullable;
      if (lhs->nullable) set_union(lhs->first, rhs->first, first);
      else first = lhs->first;
      if (rhs->nullable) set_union(lhs->last, rhs->last, last);
      else last = rhs->last;
      break;
    }
    case kStar: case kQmark: case kPlus: {
      nullable = (type == kPlus || type == kEOP) ? lhs->nullable : true;
      first = lhs->first; last = lhs->last;
      break;
    }
    case kEpsilon: nullable = true; break;
    case kEmptySet: nullable = false; break;
    case kNegation: break;
    default: throw "can't handle the type";
  }
}

std::ostream& operator<<(std::ostream& stream, const std::set<Parser::Expr*>& expr_set)
{
  for (std::set<Parser::Expr*>::iterator iter = expr_set.begin();
       iter != expr_set.end(); ++iter) {
    stream << (*iter)->id << ", ";
  }
  return stream;
}

void dump(const std::set<Parser::Expr*> &expr_set)
{
  std::cout << expr_set << std::endl;
}

void Parser::Expr::dump(std::size_t tab = 0)
{
  static std::string tabs = "    ";
  for (std::size_t i = 0; i < tab; i++) std::cout << tabs;
  switch (type) {
    case Parser::kUnion: case Parser::kConcat: {
      std::cout << *this << std::endl;
      lhs->dump(tab + 1);
      rhs->dump(tab + 1);
      break;
    }
    case Parser::kStar: case Parser::kPlus: case Parser::kQmark:
    case Parser::kNegation: {
      std::cout << *this << std::endl;
      lhs->dump(tab + 1);
      break;
    }
    default: std::cout << *this << std::endl;
  }
}

std::ostream& operator<<(std::ostream& stream, Parser::Expr& expr)
{
  stream << expr.type_name() << ": " << "id = " << expr.id
         << ", nullable = " << expr.nullable << ", literal = "
         << static_cast<int>(expr.literal) << ", #cc_table = " << expr.cc_table.count()
         << ", follow = " << expr.follow.size();
  return stream;
}

const char* Parser::Expr::type_name()
{
  static const char* type_name_[] = {
    "Literal", "Dot", "CharClass", "UTF8",
    "Concat", "Union", 
    "kStar", "kPlus", "kRepetition", "kQmark",
    "kEOP", "kLpar", "kRpar", "kByteRange", "kEpsilon",
    "kNegation", "kBadExpr"
  };

  return type_name_[type];
}

Parser::Expr* Parser::new_expr(ExprType t = kEpsilon, Expr* lhs = NULL, Expr* rhs = NULL)
{
  _expr_tree.resize(_expr_tree.size() + 1);
  _expr_tree.back().init(t, lhs, rhs);
  _expr_tree.back().id = _expr_tree.size() - 1;
  return &_expr_tree.back();
}

Parser::Expr* Parser::clone_expr(Expr* orig)
{
  if (orig == NULL) return NULL;

  Expr* clone = new_expr(orig->type, clone_expr(orig->lhs), clone_expr(orig->rhs));

  switch (orig->type) {
    case kLiteral:
      clone->literal = orig->literal;
      break;
    case kCharClass:
      clone->cc_table = orig->cc_table;
      break;
    default: break;
  }

  return clone;
}

Parser::Parser(const std::string& regex, Encoding enc): _ok(false), _regex(regex), _encoding(enc), _metachar(false)
{
  _regex_begin = _regex_ptr = reinterpret_cast<const unsigned char*>(_regex.data());
  _regex_end = reinterpret_cast<const unsigned char*>(_regex.data()) + _regex.length();
}

Parser::ExprType Parser::consume()
{
  if (regex_empty()) {
    _literal = '\0';
    _token = kEOP;
    return _token;
  }

  bool meta = false;
  switch (_literal = lex_char()) {
    case '[': _token = kCharClass; break;
    case '.': _token = kDot;   break;
    case '|': _token = kUnion; break;
    case '?': _token = kQmark; break;
    case '+': _token = kPlus;  break;
    case '*': _token = kStar;  break;
    case '(': _token = kLpar;  break;
    case ')': _token = kRpar;  break;
    case '!': _token = kNegation; break;
    case '@': _token = kEmptySet; break;
    case '{': consume_char(); _token = consume_repetition(); break;
    case '\\':consume_char(); meta = true; _token = consume_metachar(); break;
    default:
      if (_encoding == UTF8 && utf8_byte_length(_literal) != 1) {
        if (!is_valid_utf8_sequence(_regex_ptr)) throw "invalid utf8 sequence";
        _token = kUTF8;
        metachar(false);
        return _token;
      } else {
        _token = kLiteral;
      }
      break;
  }

  consume_char();
  metachar(meta);
  
  return _token;
}

Parser::ExprType Parser::lex()
{
  return _token;
}

unsigned char Parser::consume_char()
{
  if (!regex_empty()) _regex_ptr++;

  return lex_char();
}

unsigned char Parser::lex_char()
{
  return *_regex_ptr;
}

bool Parser::lex_is_atom()
{
  switch (lex()) {
    case kLiteral: case kCharClass: case kDot:
    case kByteRange: case kEpsilon: case kLpar:
    case kUTF8: case kNegation: case kEmptySet:
      return true;
    default: return false;
  }
}

bool Parser::lex_is_quantifier()
{
  switch (lex()) {
    case kStar: case kPlus: case kQmark:
    case kRepetition:
      return true;
    default: return false;
  }
}

int Parser::consume_int()
{
  int val = 0;
  
  while (lex_is_int()) {
    val *= 10;
    val += lex_char() - '0';
    consume_char();
  }

  return val;
}

Parser::ExprType Parser::consume_repetition()
{
  ExprType token;

  if (lex_char() == '}') throw "bad repetition";

  _repeat_min = _repeat_max = consume_int();
  if (lex_char() == ',') {
    if (consume_char() == '}') {
      _repeat_max = repeat_infinitely;
    } else {
      _repeat_max = consume_int();
    }
  }

  if ((_repeat_max != repeat_infinitely && (_repeat_min > _repeat_max))
      || lex_char() != '}') throw "bad repetition";

  if (_repeat_min == 0 && _repeat_max == repeat_infinitely) token = kStar;
  else if (_repeat_min == 1 && _repeat_max == repeat_infinitely) token = kPlus;
  else if (_repeat_min == 0 && _repeat_max == repeat_infinitely) token = kQmark;
  else token = kRepetition;

  return token;
}

Parser::ExprType Parser::consume_metachar()
{
  ExprType token;

  switch (lex_char()) {
    case '\0': throw "bad '\\'";
    case 'a': /* bell */
      _literal = '\a';
      token = kLiteral;
      break;
    case 'd': /*digits /[0-9]/  */
    case 'D': // or not
      _cc_table.reset();
      for (unsigned char c = '0'; c <= '9'; c++) _cc_table.set(c);
      if (lex_char() == 'D') _cc_table.flip();
      token = kByteRange;
      break;
    case 'f': /* form feed */
      _literal = '\f';
      token = kLiteral;
      break;
    case 'n': /* new line */
      _literal = '\n';
      token = kLiteral;
      break;
    case 'r': /* carriage retur */
      _literal = '\r';
      token = kLiteral;
      break;
    case 's': /* whitespace [\t\n\f\r ] */
    case 'S': // or not
      _cc_table.reset();
      _cc_table.set('\t'); _cc_table.set('\n'); _cc_table.set('\f');
      _cc_table.set('\r'); _cc_table.set(' ');
      if (lex_char() == 'S') _cc_table.flip();
      token = kByteRange;
      break;
    case 't': /* horizontal tab */
      _literal = '\t';
      token = kLiteral;
      break;
    case 'v': /* vertical tab */
      _literal = '\v';
      token = kLiteral;
      break;
    case 'w': /* word characters [0-9A-Za-z_] */
    case 'W': // or not
      _cc_table.reset();
      for (unsigned char c = '0'; c <= '9'; c++) _cc_table.set(c);
      for (unsigned char c = 'A'; c <= 'Z'; c++) _cc_table.set(c);
      for (unsigned char c = 'a'; c <= 'z'; c++) _cc_table.set(c);
      _cc_table.set('_');
      if (lex_char() == 'W') _cc_table.flip();
      token = kByteRange;
      break;
    case 'x': {
      unsigned char hex = 0;
      for (int i = 0; i < 2; i++) {
        _literal = consume_char();
        hex <<= 4;
        if ('A' <= _literal && _literal <= 'F') {
          hex += _literal - 'A' + 10;
        } else if ('a' <= _literal && _literal <= 'f') {
          hex += _literal - 'a' + 10;
        } else if ('0' <= _literal && _literal <= '9') {
          hex += _literal - '0';
        } else {
          if (i == 0) {
            hex = 0;
          } else {
            hex >>= 4;
          }
          _regex_ptr--;
          break;
        }
      }
      token = kLiteral;
      _literal = hex;
      break;
    }
    default:
      token = kLiteral;
      _literal = lex_char();
      break;
  }

  return token;
}

void Parser::parse()
{
  consume();

  if (lex() == kEOP) {
    _expr_root = new_expr(kEOP);
  } else {
    Expr* expr = parse_union();
    if (lex() != kEOP) throw "bad EOP";
    Expr* eop = new_expr(kEOP);
    _expr_root = new_expr(kConcat, expr, eop);
  }
}

Parser::Expr* Parser::parse_union()
{
  Expr* e;
  
  e = parse_concat();

  while (lex() == kUnion) {
    consume();
    if (lex() == kUnion || lex() == kRpar || lex() == kEOP) {
      e = new_expr(kQmark, e);
      if (lex() == kRpar || lex() == kEOP) break;
    } else {
      Expr* f = parse_concat();
      Expr* g = new_expr(kUnion, e, f);
      e = g;
    }
  }

  return e;
}

Parser::Expr* Parser::parse_concat()
{
  Expr* e = parse_repetition();

  while (lex_is_atom()) {
    Expr* f = parse_repetition();
    Expr* g = new_expr(kConcat, e, f);
    e = g;
  }

  return e;
}

Parser::Expr* Parser::parse_repetition()
{
  Expr* e = parse_atom();

  while (lex_is_quantifier()) {
    switch (lex()) {
      case kStar: case kPlus: case kQmark: {
        Expr* f = new_expr(lex(), e);
        e = f;
        break;
      }
      case kRepetition: {
        Expr* orig = e;
        if (_repeat_min == 0) {
          if (_repeat_max == 0) {
            e = new_expr(kEpsilon, e);
          } else {
            e = new_expr(kQmark, e);
            _repeat_min = 1;
          }
        }
        for (int i = 1; i < _repeat_min; i++) {
          Expr* f = clone_expr(orig);
          e = new_expr(kConcat, e, f);
        }
        if (_repeat_max == repeat_infinitely) {
          Expr* f = clone_expr(orig);
          f = new_expr(kStar, f);
          e = new_expr(kConcat, e, f);          
        } else {
          for (int i = _repeat_min; i < _repeat_max; i++) {
            Expr* f = clone_expr(orig);
            f = new_expr(kQmark, f);
            e = new_expr(kConcat, e, f);
          }
        }
        break;
      }
      default: throw "can't handle the type";
    }
    consume();
  }

  return e;
}

Parser::Expr* Parser::parse_atom()
{
  Expr* e; 

  switch (lex()) {
    case kLiteral: {
      e = new_expr(kLiteral);
      e->literal = _literal;
      break;
    }
    case kCharClass: {
      e = parse_charclass();
      break;
    }
    case kDot: {
      e = new_expr(kDot);
      break;
    }
    case kByteRange: {
      e = new_expr(kCharClass);
      e->cc_table = _cc_table;
      break;
    }
    case kEpsilon: {
      e = new_expr(kEpsilon);
      break;
    }
    case kUnion: {
      consume();
      e = parse_union();
      e = new_expr(kQmark, e);
      return e;
    }
    case kUTF8: {
      unsigned char top = _literal;
      e = new_expr(kLiteral);
      e->literal = top;
      for (std::size_t i = 1; i < utf8_byte_length(top); i++) {
        Expr* f = new_expr(kLiteral);
        f->literal = consume_char();
        Expr* g = new_expr(kConcat, e, f);
        e = g;
      }
      consume_char();
      break;
    }
    case kNegation: {
      consume();
      e = parse_atom();
      e = new_expr(kNegation, e);
      return e;
    }
    case kEmptySet: {
      e = new_expr(kEmptySet);
      break;
    }
    case kLpar: {
      consume();
      if (lex() == kRpar) { // () is regarded as epsilon
        e = new_expr(kEpsilon);
      } else {
        e = parse_union();
        if (lex() != kRpar) throw "bad parentheses";
      }
      break;
    }
    default: throw "can't handle the type ";
  }

  consume();
  
  return e;
}

Parser::Expr* Parser::parse_charclass()
{
  Expr* cc = new_expr(kCharClass);
  bool range = false;
  bool negative = false;
  unsigned char last = '\0';

  consume();
  
  if (_literal == '^' && !metachar()) {
    consume();
    negative = true;
  }
  if (_literal == '-' ||
      _literal == ']') {
    cc->cc_table.set(_literal);
    last = _literal;
    consume();
  }

  for (; lex() != kEOP && _literal != ']'; consume()) {
    if (!range && _literal == '-' && !metachar()) {
      range = true;
      continue;
    }

    if (lex() == kByteRange) cc->cc_table |= _cc_table;
    else cc->cc_table.set(_literal);

    if (range) {
      for (std::size_t c = last; c <= _literal; c++) cc->cc_table.set(c);
      range = false;
    }

    last = _literal;
  }

  if (lex() == kEOP) throw "invalid character class";
  if (range) cc->cc_table.set('-');
  if (negative) cc->cc_table.flip();
  if (cc->cc_table.count() == 1) {
    cc->type = kLiteral;
    for (std::size_t c = 0; c < 256; c++) {
      if (cc->cc_table[c]) {
        cc->literal = c;
        break;
      }
    }
    cc->cc_table.reset();
  }

  return cc;
}

void Parser::fill_pos(Expr *expr)
{
  ExprType type = expr->type;
  Expr* lhs = expr->lhs;
  Expr* rhs = expr->rhs;
  bool& nullable = expr->nullable;
  std::set<Expr*>& first = expr->first;
  std::set<Expr*>& last = expr->last;
  first.clear();
  last.clear();

  switch (type) {
    case kLiteral: case kCharClass: case kDot: case kEmptySet: case kEOP: {
      nullable = false;
      first.insert(expr);
      last.insert(expr);
      break;
    }
    case kEpsilon: {
      nullable = true;
      first.insert(expr);
      last.insert(expr);
      break;
    }
    case kConcat: {
      fill_pos(expr->lhs); fill_pos(expr->rhs);
      nullable = lhs->nullable && rhs->nullable;
      if (lhs->nullable) expr->set_union(lhs->first, rhs->first, first);
      else first = lhs->first;
      if (rhs->nullable) expr->set_union(lhs->last, rhs->last, last);
      else last = rhs->last;
      break;
    } 
    case kUnion: {
      fill_pos(expr->lhs); fill_pos(expr->rhs);
      nullable = lhs->nullable || rhs->nullable;
      expr->set_union(lhs->first, rhs->first, first);
      expr->set_union(lhs->last, rhs->last, last);
      break;
    }
    case kStar: case kPlus: case kQmark: {
      fill_pos(expr->lhs);
      nullable = (type == kPlus || type == kEOP) ? lhs->nullable : true;
      first = lhs->first; last = lhs->last;
      break;
    }
    default: throw "can't handle the type";
  }
}

void Parser::fill_transition(Expr *expr)
{
  switch (expr->type) {
    case kLiteral: case kCharClass: case kDot: case kEOP: case kEmptySet:
      break;
    case kEpsilon:
      break;
    case kConcat:
      connect(expr->lhs->last, expr->rhs->first);
      // fall through
    case kUnion:
      fill_transition(expr->lhs); fill_transition(expr->rhs);
      break;
    case kStar: case kPlus:
      connect(expr->lhs->last, expr->lhs->first);
      // fall through
    case kQmark:
      fill_transition(expr->lhs);
      break;
    default: throw "can't handle the type";
  }
}

void Parser::connect(std::set<Expr*> src, std::set<Expr*> dst)
{
  for (std::set<Expr*>::iterator iter = src.begin(); iter != src.end(); ++iter) {
    (*iter)->follow.insert(dst.begin(), dst.end());
  }
}

class NFA {
 public:
  struct State {
    std::vector<std::set<int> > t;
    int id;
    bool accept;
    const std::set<int>& operator[](std::size_t i) const { return t[i]; }
    std::set<int>& operator[](std::size_t i) { return t[i]; }
  };
  std::size_t size() const { return _states.size(); }
  const State& state(std::size_t i) const { return _states[i]; }
  State& state(std::size_t i) { return _states[i]; }
  bool accept(int state) const { return _states[state].accept; }
  bool accept(const std::string&) const;
  const State& operator[](std::size_t i) const { return _states[i]; }
  State& operator[](std::size_t i) { return _states[i]; }
  const std::set<int>& start_states() const { return _start_states; }
  std::set<int>& start_states() { return _start_states; }

  State& new_state();
  //fields
  std::deque<State> _states;
  std::set<int> _start_states;
};

bool NFA::accept(const std::string& text) const
{
  std::set<int> state = _start_states;
  std::set<int> next;

  for (std::size_t i = 0; i < text.length(); i++) {
    next.clear();
    for (std::set<int>::iterator iter = state.begin(); iter != state.end(); ++iter) {
      const std::set<int> &next_ = _states[*iter][static_cast<unsigned char>(text[i])];
      next.insert(next_.begin(), next_.end());
    }
    state = next;
  }

  for (std::set<int>::iterator iter = state.begin(); iter != state.end(); ++iter) {
    if (accept(*iter)) return true;
  }
  
  return false;
}

NFA::State& NFA::new_state()
{
  _states.resize(_states.size() + 1);
  State& state = _states.back();
  state.id = _states.size() - 1;
  state.t.resize(256);
  state.accept = false;
  return state;
}

class DFA {
  friend class RECON;
 public:
  enum State_t { REJECT = -1, START = 0 };
  typedef std::set<Parser::Expr*> Subset;
  typedef std::set<int> NSubset;
  struct State {
    int t[256];
    int id;
    bool accept;
    int operator[](std::size_t i) const { return t[i]; }
    int& operator[](std::size_t i) { return t[i]; }
  };
  DFA(): _ignorecase(false), _ok(false) {};
  DFA(const NFA&, bool, bool);
  bool ok() const { return _ok; }
  bool ignorecase() const { return _ignorecase; }
  const std::string& error() const { return _error; }
  std::size_t size() const { return _states.size(); }
  const State& state(std::size_t i) const { return _states[i]; }
  State& state(std::size_t i) { return _states[i]; }
  bool accept(int state) const { return state != REJECT && _states[state].accept; }
  bool accept(const std::string&) const;
  const State& operator[](std::size_t i) const { return _states[i]; }
  State& operator[](std::size_t i) { return _states[i]; }
  void minimize();
  void complemente();
  bool operator==(const DFA&) const;
  friend std::ostream& operator<<(std::ostream& stream, const DFA& dfa);
 private:
  void construct(Parser::Expr* expr);
  void construct(const NFA& nfa);
  void fill_transition(Parser::Expr*, std::vector<Subset>&);
  State& new_state();
  static std::string& pretty(unsigned char, std::string &);

  //fields
  bool _ok;
  bool _ignorecase;
  std::string _error;
  std::deque<State> _states;
};

bool DFA::operator==(const DFA& lhs) const
{
  if (state(START).accept != lhs.state(START).accept) return false;
  
  std::set<std::pair<int, int> > equivalent_states;
  equivalent_states.insert(std::pair<int, int>(REJECT, REJECT));
  equivalent_states.insert(std::pair<int, int>(START, START));
  std::queue<std::pair<int, int> > queue;
  queue.push(std::pair<int, int>(START, START));

  while (!queue.empty()) {
    std::pair<int, int> pair = queue.front();
    queue.pop();

    const State& s1 = state(pair.first);
    const State& s2 = lhs.state(pair.second);
    if (s1.accept != s2.accept) return false;

    for (std::size_t c = 0; c < 256; c++) {
      std::pair<int, int> next(s1[c], s2[c]);

      if (equivalent_states.find(next) == equivalent_states.end()) {
        if (next.first == REJECT || next.second == REJECT) return false;

        equivalent_states.insert(next);
        queue.push(next);
      }
    }
  }
  
  return true;
}

std::ostream& operator<<(std::ostream& stream, const DFA& dfa)
{
  static const char* const state_circle = "circle";
  static const char* const accept_circle = "doublecircle";
  static const char* const style  = "fillcolor=lightsteelblue1, style=filled, color = navyblue ";

  stream << "digraph DFA {\n  rankdir=\"LR\"" << std::endl;
  for (std::size_t i = 0; i < dfa.size(); i++) {
    stream << "  " << i << " [shape= " << (dfa[i].accept ? accept_circle : state_circle)
           << ", " << style << "]" << std::endl;
  }
  stream << "  start [shape=point]\n  start -> " << DFA::START << std::endl;

  std::string label;
  for (std::size_t i = 0; i < dfa.size(); i++) {
    bool range = false; unsigned char last;
    for (unsigned int c = 0; c < 256; c++) {
      if (dfa[i][c] != DFA::REJECT) {
        if (range && !(c + 1 < 256 && dfa[i][c] == dfa[i][c+1])) {
          stream << DFA::pretty(c, label) << "]\"]" << std::endl;
          range = false;
        } else if (c + 1 < 256 && dfa[i][c] == dfa[i][c+1]) {
          if (range) continue;
          range = true;
          last = c;
          stream << "  " << i << " -> " << dfa[i][c]
                 << " [label=\"[" << DFA::pretty(c, label) << "-";
        } else {
          stream << "  " << i << " -> " << dfa[i][c]
                 << " [label=\"" << DFA::pretty(c, label) << "\"]" << std::endl;
        }
      }
    }
  }
  stream << "}" << std::endl;
  
  return stream;
}

std::string& DFA::pretty(unsigned char c, std::string &label)
{
  std::stringstream label_;
  if (' ' <= c && c <= '~') {
    if (c == '"' || c == '\\') label_ << '\\';
    label_ << c;
  } else {
    label_ << std::showbase << std::hex << static_cast<unsigned int>(c);
  }

  label_ >> label;
  return label;
}

DFA::DFA(const NFA &nfa, bool minimizing = true, bool ignorecase = false)
{
  try {
    construct(nfa);
  } catch (const char* error) {
    _ok = false;
    _error = "dfa construct error: ";
    _error += error;
    return;
  }

  try {
    minimize();
  } catch (const char* error) {
    _ok = false;
    _error = "dfa minimize error: ";
    _error += error;
  }  
}

void DFA::construct(Parser::Expr* expr_root)
{
  int state_num = 0;
  std::vector<Subset> transition(256);
  std::queue<Subset> queue;
  std::map<Subset, int> subset_to_state;
  queue.push(expr_root->first);
  subset_to_state[expr_root->first] = state_num++;

  while (!queue.empty()) {
    bool accept = false;
    Subset &subset = queue.front();
    std::fill(transition.begin(), transition.end(), Subset());

    for (Subset::iterator iter = subset.begin(); iter != subset.end(); ++iter) {
      accept |=  ((*iter)->type == Parser::kEOP);
      fill_transition(*iter, transition);
    }
    queue.pop();

    State& state = new_state();
    state.accept = accept;

    for (std::size_t c = 0; c < 256; c++) {
      Subset& next = transition[c];

      if (next.empty()) continue;

      if (subset_to_state.find(next) == subset_to_state.end()) {
        subset_to_state[next] = state_num++;
        queue.push(next);
      }

      state[c] = subset_to_state[next];
    }
  }

  _ok = true;
}

void DFA::construct(const NFA& nfa)
{
  int state_num = 0;
  std::vector<NSubset> transition(256);
  std::queue<NSubset> queue;
  std::map<NSubset, int> subset_to_state;
  queue.push(nfa.start_states());
  subset_to_state[nfa.start_states()] = state_num++;

  while (!queue.empty()) {
    bool accept = false;
    NSubset &subset = queue.front();
    std::fill(transition.begin(), transition.end(), NSubset());

    for (NSubset::iterator iter = subset.begin(); iter != subset.end(); ++iter) {
      accept |= nfa.accept(*iter);
      for (int c = 0; c < 256; c++) {
        transition[c].insert(nfa[*iter].t[c].begin(), nfa[*iter].t[c].end());
      }
    }
    queue.pop();

    State& state = new_state();
    state.accept = accept;

    for (std::size_t c = 0; c < 256; c++) {
      NSubset& next = transition[c];

      if (next.empty()) continue;

      if (subset_to_state.find(next) == subset_to_state.end()) {
        subset_to_state[next] = state_num++;
        queue.push(next);
      }

      state[c] = subset_to_state[next];
    }
  }

  _ok = true;
}

void DFA::fill_transition(Parser::Expr* expr, std::vector<DFA::Subset>& transition)
{
  switch (expr->type) {
    case Parser::kLiteral: {
      unsigned char index = expr->literal;
      transition[index].insert(expr->follow.begin(), expr->follow.end());
      if (ignorecase()) {
        unsigned char index_ = opposite_case(index);
        if (index_ != index) transition[index_].insert(expr->follow.begin(), expr->follow.end());
      }
      break;
    }
    case Parser::kCharClass: {
      for (std::size_t c = 0; c < 256; c++) {
        if (expr->cc_table[c]) {
          transition[c].insert(expr->follow.begin(), expr->follow.end());
          if (ignorecase()) {
            unsigned char c_ = opposite_case(static_cast<unsigned char>(c));
            if (c_ != c && !expr->cc_table[c_]) transition[c_].insert(expr->follow.begin(), expr->follow.end());
          }
        }
      }
      break;
    }
    case Parser::kDot: {
      for (std::size_t c = 0; c < 256; c++) {
        transition[c].insert(expr->follow.begin(), expr->follow.end());
      }
      break;
    }
    case Parser::kEmptySet: break;
    case Parser::kEpsilon: case Parser::kEOP: break;
    default: throw "can't handle the type";
  }
}

DFA::State& DFA::new_state()
{
  _states.resize(_states.size() + 1);
  State& state = _states.back();
  std::fill(state.t, state.t+256, REJECT);
  state.id = _states.size() - 1;
  state.accept = false;
  return state;
}

void DFA::minimize()
{
  std::vector<std::vector<bool> > distinction_table;
  distinction_table.resize(size()-1);
  for (std::size_t i = 0; i < size()-1; i++) {
    distinction_table[i].resize(size()-i-1);
    for (std::size_t j = i+1; j < size(); j++) {
      distinction_table[i][size()-j-1] = _states[i].accept != _states[j].accept;
    }
  }

  bool distinction_flag = true;
  while (distinction_flag) {
    distinction_flag = false;
    for (std::size_t i = 0; i < size()-1; i++) {
      for (std::size_t j = i+1; j < size(); j++) {
        if (!distinction_table[i][size()-j-1]) {
          for (std::size_t input = 0; input < 256; input++) {
            int n1, n2;
            n1 = _states[i][input];
            n2 = _states[j][input];
            if (n1 != n2) {
              if (n1 > n2) std::swap(n1, n2);
              if ((n1 == REJECT || n2 == REJECT) ||
                  distinction_table[n1][size()-n2-1]) {
                distinction_flag = true;
                distinction_table[i][size()-j-1] = true;
                break;
              }
            }
          }
        }
      }
    }
  }

  std::map<int, int> swap_map;
  for (std::size_t i = 0; i < size()-1; i++) {
    for (std::size_t j = i+1; j < size(); j++) {
      if (swap_map.find(j) == swap_map.end()) {
        if (!distinction_table[i][size()-j-1]) {
          swap_map[j] = i;
        }
      }
    }
  }

  if (swap_map.empty()) return;

  std::size_t minimum_size = size() - swap_map.size();
  std::vector<std::size_t> replace_map(size());
  for (std::size_t s = 0, d = 0; s < size(); s++) {
    if (swap_map.find(s) == swap_map.end()) {
      replace_map[s] = d++;
      if (s != replace_map[s]) {
        _states[replace_map[s]] = _states[s];
        _states[replace_map[s]].id = replace_map[s];
      }
    } else {
      replace_map[s] = replace_map[swap_map[s]];
    }
  }

  for (std::size_t i = 0; _states[i].id < static_cast<int>(minimum_size); i++) {
    for (std::size_t input = 0; input < 256; input++) {
      int n = _states[i][input];
      if (n != REJECT) _states[i][input] = replace_map[n];
    }
  }

  _states.resize(minimum_size);
}

bool DFA::accept(const std::string& text) const
{
  int state = START;
  for (std::size_t i = 0; i < text.length(); i++) {
    state = _states[state][static_cast<unsigned char>(text[i])];
    if (state == REJECT) return false;
  }

  return accept(state);
}

void DFA::complemente()
{
  int accept_state = REJECT;
  std::size_t s = size();
  for (std::size_t i = 0; i < s; i++) {
    _states[i].accept ^= 1;
    if (std::find(_states[i].t, _states[i].t+256, REJECT) != _states[i].t+256) {
      if (accept_state == REJECT) {
        State& state = new_state();
        accept_state = state.id;
        state.accept = true;
        std::fill(state.t, state.t+256, accept_state);
      }
      for (int c = 0; c < 256; c++) {
        if(_states[i][c] == REJECT) _states[i][c] = accept_state;
      }
    }
  }
}

class Matrix {
 public:
  Matrix(std::size_t i = 0): _size(i), m(_size*_size) {}
  Matrix(std::size_t row, std::size_t col): _size(row), m(_size*_size) {}
  Matrix(const Matrix &M) { *this = M; }
  void resize(std::size_t n) { resize(n, n); }
  void resize(std::size_t row, std::size_t col) { _size = row; m.resize(_size*_size); }
  friend std::ostream& operator<<(std::ostream& stream, const Matrix& matrix);
  std::size_t size() const { return _size; }
  void clear();
  void init();
  void swap(Matrix &M) { m.swap(M.m); }
  const unsigned char& operator()(std::size_t i, std::size_t j) const { return m[i*_size+j]; }
  unsigned char& operator()(std::size_t i, std::size_t j) { return m[i*_size+j]; }
  Matrix& operator*=(const Matrix&);
  bool operator<(const Matrix& mat) const
  { return m < mat.m; }
  bool operator==(const Matrix& mat) const
  { return m == mat.m; }

 private:
  std::size_t _size;
  std::vector<unsigned char> m;
};

class IdentityMatrix: public Matrix {
 public:
  IdentityMatrix(std::size_t n): Matrix(n, n)
  {
    for (std::size_t i = 0; i < n; i++) (*this)(i, i) = 1;
  }
};

Matrix& Matrix::operator*=(const Matrix &M)
{
  Matrix tmp(size(), size());
  for (std::size_t i = 0; i < size(); i++) {
    for (std::size_t j = 0; j < size(); j++) {
      for (std::size_t k = 0; k < size(); k++) {
        tmp(i, j) += (*this)(i, k) * M(k, j);
      }
    }
  }
  swap(tmp);
  return *this;
}

void Matrix::clear()
{
  for (std::size_t i = 0; i < size(); i++) {
    for (std::size_t j = 0; j < size(); j++) {
      (*this)(i, j) = 0;
    }
  }
}

void Matrix::init()
{
  for (std::size_t i = 0; i < size(); i++) {
    for (std::size_t j = 0; j < size(); j++) {
      (*this)(i, j) = i == j ? 1 : 0;
    }
  }
}

std::ostream& operator<<(std::ostream& stream, const Matrix& matrix)
{
  for (std::size_t i = 0; i < matrix.size(); i++) {
    stream << "{";
    for (std::size_t j = 0; j < matrix.size(); j++) {
      stream << matrix(i, j);
      if (j != matrix.size() - 1) stream << ",";
    }
    stream << "}";
    if (i != matrix.size() - 1) stream << ",";
    stream << std::endl;
  }
  return stream;
}

class SyntacticMonoid {
 public:
  typedef unsigned int Element;
  typedef std::set<Element> ElementSet;
  SyntacticMonoid(): _ok(false) {}
  bool identity(Element e) const { return e == 0; }
  bool construct(const DFA&, const std::bitset<256>&);
  bool ok() const { return _ok; }
  size_t size() const { return _transitions.size(); }
  bool accept(std::size_t index) const { return _accept[index]; }
  bool aperiodic() const;
  Element morphism(const std::string&) const;
  Element morphism(unsigned char c) const
  { return _charmorphism.find(c)->second; }
  Element operator()(unsigned char c) const { return morphism(c); }
  Element operator()(const std::string& str) const { return morphism(str); }
  Element& multiply(Element i, Element j)
  { return _multiplication_table[i*size()+j]; }
  const Element& multiply(Element i, Element j) const
  { return _multiplication_table[i*size()+j]; } 
  Element& operator()(Element i, Element j)
  { return multiply(i, j); }
  const Element& operator()(Element i, Element j) const
  { return multiply(i, j); }
  ElementSet operator()() const;
  ElementSet operator()(const ElementSet&, Element) const;
  ElementSet operator()(Element, const ElementSet&) const;
  ElementSet operator()(const ElementSet&, const ElementSet&) const;
  friend std::ostream& operator<<(std::ostream& stream, const SyntacticMonoid& monoid);
  
 private:
  std::vector<Element> _multiplication_table;
  std::vector<bool> _accept;
  std::bitset<256> _alphabets;
  DFA _dfa;
  bool _ok;
  std::vector<Matrix> _transitions;
  std::map<Matrix, std::size_t> _transitions_map;
  std::map<unsigned char, Element> _charmorphism;
};

std::ostream& operator<<(std::ostream& stream, const SyntacticMonoid& monoid)
{
  static const char* const state_circle = "circle";
  static const char* const accept_circle = "doublecircle";
  static const char* const style  = "fillcolor=lightsteelblue1, style=filled, color = navyblue ";

  stream << "digraph Monoid {\n  rankdir=\"LR\"" << std::endl;
  for (std::size_t i = 0; i < monoid.size(); i++) {
    stream << "  " << i << " [shape= " << (monoid.accept(i) ? accept_circle : state_circle)
           << ", " << style << "]" << std::endl;
  }
  stream << "  start [shape=point]\n  start -> 0" << std::endl;

  std::string label;
    for (SyntacticMonoid::Element m = 0; m < monoid.size(); m++) {
    for (SyntacticMonoid::Element n = 0; n < monoid.size(); n++) {
      if (monoid(n, m) != n) {
        stream << "  " << n << " -> " << monoid(n, m)
               << " [label=\"" << m << "\", color=blue]" << std::endl;
      }
      if (monoid(m,n) != n) {
        stream << "  " << n << " -> " << monoid(m, n)
               << " [label=\"" << m << "\", color=red]" << std::endl;
      }
    }
  }
  stream << "}" << std::endl;
  
  return stream;
}

bool SyntacticMonoid::construct(const DFA& dfa, const std::bitset<256>& alphabets)
{
  _dfa = dfa;
  _alphabets = alphabets;
  IdentityMatrix ident(dfa.size());
  _transitions_map[ident] = 0;
  std::queue<Matrix> queue;
  queue.push(ident);

  while (!queue.empty()) {
    Matrix& mat = queue.front();
    
    for (std::size_t c = 0; c < 256; c++) {
      if (!alphabets[c]) continue;
      Matrix next(dfa.size());
      for (std::size_t i = 0; i < dfa.size(); i++) {
        for (std::size_t j = 0; j < dfa.size(); j++) {
          if (mat(i, j) == 1) {
            if (dfa[j][c] != DFA::REJECT) {
              next(i, dfa[j][c]) = 1;
            }
          }
        }
      }
      if (_transitions_map.find(next) == _transitions_map.end()) {
        _transitions_map[next] = _transitions_map.size();
        queue.push(next);
      }
      if (mat == ident) _charmorphism[c] = _transitions_map[next];
    }
    queue.pop();
  }

  _accept.resize(_transitions_map.size());
  _transitions.resize(_transitions_map.size());
  _multiplication_table.resize(size()*size());
  for (std::map<Matrix, std::size_t>::iterator iter1 = _transitions_map.begin();
       iter1 != _transitions_map.end(); ++iter1) {
    _transitions[iter1->second] = iter1->first;
    _accept[iter1->second] = false;

    for (std::size_t i = 0; i < dfa.size(); i++) {
      if ((iter1->first)(0, i) != 0 && dfa[i].accept) {
        _accept[iter1->second] = true;
      }
    }
    
    for (std::map<Matrix, std::size_t>::iterator iter2 = _transitions_map.begin();
         iter2 != _transitions_map.end(); ++iter2) {
      Matrix mat = iter1->first;
      mat *= iter2->first;
      multiply(iter1->second, iter2->second) = _transitions_map[mat];
      mat = iter2->first;
      mat *= iter1->first;
      multiply(iter2->second, iter1->second) = _transitions_map[mat];
    }
  }

  _ok = true;
  return true;
}

SyntacticMonoid::ElementSet SyntacticMonoid::operator()() const
{
  ElementSet M;
  for (Element i = 0; i < size(); i++) M.insert(i);
  return M;
}

SyntacticMonoid::ElementSet SyntacticMonoid::operator()(const ElementSet& s, Element e) const
{
  ElementSet result;
  for (ElementSet::iterator iter = s.begin(); iter != s.end(); ++iter) {
    result.insert(multiply(*iter, e));
  }
  return result;
}

SyntacticMonoid::ElementSet SyntacticMonoid::operator()(Element e, const ElementSet& s) const
{
  ElementSet result;
  for (ElementSet::iterator iter = s.begin(); iter != s.end(); ++iter) {
    result.insert(multiply(e, *iter));
  }
  return result;
}

SyntacticMonoid::ElementSet SyntacticMonoid::operator()(const ElementSet& s1, const ElementSet& s2) const
{
  ElementSet result;
  for (ElementSet::iterator iter1 = s1.begin(); iter1 != s1.end(); ++iter1) {
    for (ElementSet::iterator iter2 = s2.begin(); iter2 != s2.end(); ++iter2) {
      result.insert(multiply(*iter1, *iter2));
    }    
  }
  return result;
}

SyntacticMonoid::Element SyntacticMonoid::morphism(const std::string& str) const
{
  Matrix mat(_dfa.size());
  for (std::size_t i = 0; i < _dfa.size(); i++) {
    int state = i;
    bool t = true;
    for (std::size_t j = 0; j < str.length(); j++) {
      if (!_alphabets[static_cast<unsigned char>(str[j])]) {
        throw "invalid string";
      }
      if (state != DFA::REJECT) state = _dfa[state][str[j]];
    }
    if (state != DFA::REJECT) mat(i, state) = 1;
  }

  return _transitions_map.find(mat)->second;
}

bool SyntacticMonoid::aperiodic() const
{
  for (std::size_t i = 0; i < size(); i++) {
    Element e = i;
    for (std::size_t j = 0; j < size(); j++) {
      e = multiply(e, i);
    }
    if (e != multiply(e, i)) return false;
  }

  return true;
}

class RECON {
 public:
  RECON(const std::string&);
  bool compile();
  bool scompile();
  const DFA& dfa() { return _dfa; };
  const SyntacticMonoid& monoid() { return _monoid; };
  bool ok() const { return _ok; };
  std::bitset<256> alphabets() const { return _alphabets; }
  bool alphabets(const std::string&);
  bool alphabets(unsigned char c) const { return _alphabets[c]; }
  const std::string& error() const { return _error; };
  void complemente() { _dfa.complemente(); };
  const std::string& expression(std::string& regex) const
  { return expression(regex, _dfa); }
  std::string const expression() const
  { std::string s; expression(s); return s; };
  const std::string& starfree_expression(std::string& regex) const;
  std::string const starfree_expression() const
  { std::string s; starfree_expression(s); return s; };
  static std::string charclass(const std::bitset<256>&);
 private:
  std::string const starfree_recursion(SyntacticMonoid::Element e, std::map<SyntacticMonoid::Element, std::string>& memo) const;
  void normalize(Parser::Expr* expr, Parser& p);
  void negate(Parser::Expr* expr, Parser& p);
  const std::string& expression(std::string&, const DFA& d) const;
  //fields
  std::string _regex;
  std::bitset<256> _alphabets;
  DFA _dfa;
  SyntacticMonoid _monoid;
  Encoding _enc;
  bool _minimizing;
  bool _ok;
  std::string _error;
};

RECON::RECON(const std::string &regex): _regex(regex), _enc(ASCII),
                                        _minimizing(true), _ok(false), _error("") { _alphabets.set(); }

bool RECON::alphabets(const std::string& alphre)
{
  DFA d;
  _alphabets.reset();
  try {
    Parser palph(alphre, ASCII);
    palph.parse();
    palph.fill_pos(palph._expr_root);
    palph.fill_transition(palph._expr_root);
    d.construct(palph.expr_root());
  } catch (const char* error) {
    _ok = false;
    _error = "invalid alphabets";
    return false;
  }

  for (std::size_t c = 0; c < 256; c++) {
    if (d.accept(std::string(1, c))) _alphabets.set(c);
  }

  return true;
}

bool RECON::compile()
{
  Parser p(_regex, _enc);
  try {
    p.parse();
  } catch (const char* error) {
    _ok = false;
    _error = "regular expression parsing error: ";
    _error += error;
    return false;
  }

  try {
    normalize(p._expr_root, p);
  } catch (const char* error) {
    _ok = false;
    _error = "regular expression normalization error: ";
    _error += error;
    return false;
  }
  
  try {
    p.fill_pos(p._expr_root);
    p.fill_transition(p._expr_root);
  } catch (const char* error) {
    _ok = false;
    _error = "regular expression transition error: ";
    _error += error;
    return false;
  }
  
  try {
    _dfa.construct(p.expr_root());
  } catch (const char* error) {
    _ok = false;
    _error = "dfa construction error: ";
    _error += error;
    return false;
   }

  try {
    if (_minimizing) _dfa.minimize();
  } catch (const char* error) {
    _ok = false;
    _error = "dfa minimization error: ";
    _error += error;
    return false;
  }

  _ok = true;
  
  return _ok;
}

bool RECON::scompile()
{
  if (!_dfa.ok() && !compile()) return false;
  _monoid.construct(_dfa, alphabets());

  return _monoid.ok();
}

void RECON::normalize(Parser::Expr* expr, Parser& p)
{
  switch (expr->type) {
    case Parser::kLiteral: case Parser::kCharClass: case Parser::kDot:
    case Parser::kEOP: case Parser::kEpsilon: case Parser::kEmptySet:
      break;
    case Parser::kConcat: case Parser::kUnion: {
      normalize(expr->lhs, p); normalize(expr->rhs, p);
      break;
    }
    case Parser::kStar: case Parser::kPlus: case Parser::kQmark: {
      normalize(expr->lhs, p);
      break;
    }
    case Parser::kNegation: {
      normalize(expr->lhs, p);
      Parser::Expr* eop = p.new_expr(Parser::kEOP);
      expr->type = Parser::kConcat;
      expr->rhs = eop;
      p.fill_pos(expr);
      p.fill_transition(expr);
      DFA d;
      d.construct(expr);
      d.complemente();
      std::string regex;
      expression(regex, d);
      Parser negp(regex, _enc);
      negp.parse();
      if (negp.expr_root()->type == Parser::kEOP) {
        expr->init(Parser::kEpsilon);
      } else {
        Parser::Expr* nege = p.clone_expr(negp.expr_root()->lhs);
        expr->init(nege->type, nege->lhs, nege->rhs);
        switch (nege->type) {
          case Parser::kLiteral:
            expr->literal = nege->literal;
            break;
          case Parser::kCharClass:
            expr->cc_table = nege->cc_table;
            break;
          default: break;
        }
      }
      break;
    }
    default: throw "can't handle the type";
  }

  return;
}

const std::string& RECON::expression(std::string& regex, const DFA& d) const
{
  typedef std::pair<size_t, size_t> key;
  std::map<key, std::string> expressions;
  std::size_t nstates = d.size() + 2;
  const std::size_t START = d.size(), END = d.size() + 1;

  for (std::size_t i = 0; i < d.size(); i++) {
    const DFA::State& state = d[i];
    if (i == DFA::START) expressions[key(START, i)] = "";
    if (state.accept) expressions[key(i, END)] = "";
    for (std::size_t c = 0; c < 256; c++) {
      if (state[c] == DFA::REJECT || !alphabets(c)) continue;
      if (expressions.find(key(i, state[c])) == expressions.end()) {
        expressions[key(i, state[c])] = c;
      } else {
        expressions[key(i, state[c])] += "|";
        expressions[key(i, state[c])] += c;
      }
    }
  }
  
  for (std::size_t i = 0; i < d.size(); i++) {
    std::string loop = "";
    if (expressions.find(key(i, i)) != expressions.end()) {
      loop = "(" + expressions[key(i, i)] + ")*";
    }
    for (std::size_t j = i + 1; j < nstates; j++) {
      if (expressions.find(key(i, j)) == expressions.end()) continue;
      std::string to = expressions[key(i, j)];
      if (to.find('|') != std::string::npos) to = "(" + to + ")";
      for (std::size_t k = i + 1; k < nstates; k++) {
        if (expressions.find(key(k, i)) == expressions.end()) continue;
        std::string from = expressions[key(k, i)];
        if (from.find('|') != std::string::npos) from = "(" + from + ")";
        if (expressions.find(key(k, j)) != expressions.end()) {
          if (expressions[key(k, j)] == "") {
            expressions[key(k, j)] = "(" + from + loop + to + ")?";
          } else {
            expressions[key(k, j)] = expressions[key(k, j)] + "|" + from + loop + to;
          }
        } else {
          expressions[key(k, j)] = from + loop + to;
        }
      }
    }
  }

  if (expressions.find(key(START, END)) == expressions.end()) {
    regex = "@";
  } else {
    regex = expressions[key(START, END)];
  }

  return regex;
}

const std::string& RECON::starfree_expression(std::string& regex) const
{
  if (!_monoid.aperiodic()) {
    regex = _regex;
    return regex;
  }

  regex = "@";
  std::map<SyntacticMonoid::Element, std::string> memo;

  for (SyntacticMonoid::Element e = 0; e < _monoid.size(); e++) {
    if (_monoid.accept(e)) {
      regex += "|";
      regex += starfree_recursion(e, memo);
    }
  }

  return regex;
}

std::string const RECON::starfree_recursion(SyntacticMonoid::Element m, std::map<SyntacticMonoid::Element, std::string>& memo) const
{
  if (memo.find(m) != memo.end()) return memo[m];
  
  std::ostringstream regex;

  if (_monoid.identity(m)) {
    // \phi^{-1}(1) = A* \ A* W A* = !(A* W A*)
    // where W = { a \in A: \phi(a) != 1 }
    std::bitset<256> W;
    for (std::size_t c = 0; c < 256; c++) {
      if (!alphabets(c)) continue;
      if (!_monoid.identity(_monoid.morphism(c))) W.set(c);
    }
    if (W.count() == alphabets().count()) {
      regex << ""; // empty string (epsilon) because !(.*..*) = epsilon
    } else if (W.count() == alphabets().count() - 1) {
      for (std::size_t c = 0; c < 256; c++) {
        if (alphabets(c) && !W[c]) {
          regex << c;
          regex << "*";
          break;
        }
      }
    } else {
      regex << "[";
      for (std::size_t c = 0; c < 256; c++) {
        if (alphabets(c) && !W[c]) regex << c;
      }
      regex << "]*";
    }
  } else {    
    std::ostringstream UA, AV, AWA;
    // \phi^{-1}(m) = (U A* & A* V) \ A* W A*
    // where U = { \phi^{-1}(n)a  | n \in M, a \in A,
    //                              n \phi(a) M = mM but n \notin mM }
    //       V = { a \phi^{-1}(n) | n \in M, a \in A,
    //                              M \phi(a) n = Mm but n \notin Mm }
    //       W = { a \in A | m \notin M \phi(a) M }
    //         | { a \phi^{-1}(n) b | n \in M, a,b \in A,
    //                                m \in M \phi(a) n M & M n \phi(b) M
    //                            but m \notin M \phi(a) n \phi(b) M      }

    const SyntacticMonoid::ElementSet M = _monoid();
    const SyntacticMonoid::ElementSet Mm = _monoid(M, m);
    const SyntacticMonoid::ElementSet mM = _monoid(m, M);

    // build U A*
    // where U = { \phi^{-1}(n)a  | n \in M, a \in A,
    //                              n \phi(a) M = mM but n \notin mM }
    std::vector<std::string> tmp;
    for (SyntacticMonoid::Element n = 0; n < _monoid.size(); n++) {
      for (std::size_t a = 0; a < 256; a++) {
        if (!alphabets(a)) continue;
        if (mM.find(n) != mM.end()) continue;
        SyntacticMonoid::ElementSet naM = _monoid(_monoid(n, _monoid(a)), M);
        if (naM != mM) continue;
        tmp.push_back(starfree_recursion(n, memo) + std::string(1, a));
      }
    }
    if (tmp.empty()) {
      UA << "@";
    } else if (tmp.size() == 1) {
      UA << tmp[0] << "!@";
    } else {
      UA << "(";
      for (std::size_t i = 0; i < tmp.size(); i++) {
        UA << tmp[i];
        if (i < tmp.size() - 1) UA << "|";
      }
      UA << ")!@";
    }

    // build A* V
    // where V = { a \phi^{-1}(n) | n \in M, a \in A,
    //                              M \phi(a) n = Mm but n \notin Mm }
    tmp.clear();
    for (SyntacticMonoid::Element n = 0; n < _monoid.size(); n++) {
      for (std::size_t a = 0; a < 256; a++) {
        if (!alphabets(a)) continue;
        if (Mm.find(n) != Mm.end()) continue;
        SyntacticMonoid::ElementSet Man = _monoid(M, _monoid(_monoid(a), n));
        if (Man != Mm) continue;
        tmp.push_back(std::string(1, a) + starfree_recursion(n, memo));
      }
    }
    if (tmp.empty()) {
      AV << "@";
    } else if (tmp.size() == 1) {
      AV << "!@" << tmp[0];
    } else {
      AV << "!@(";
      for (std::size_t i = 0; i < tmp.size(); i++) {
        AV << tmp[i];
        if (i < tmp.size() - 1) AV << "|";
      }
      AV << ")";
    }

    // build A* W A*
    // where W = { a \in A | m \notin M \phi(a) M }
    //         | { a \phi^{-1}(n) b | n \in M, a,b \in A,
    //                                m \in M \phi(a) n M & M n \phi(b) M
    //                            but m \notin M \phi(a) n \phi(b) M      }
    tmp.clear();
    std::set<unsigned char> W_;
    for (std::size_t a = 0; a < 256; a++) {
      if (!alphabets(a)) continue;
      const SyntacticMonoid::ElementSet MaM = _monoid(M, _monoid(_monoid(a), M));
      if (MaM.find(m) == MaM.end()) W_.insert(a);
    }
    for (std::size_t a = 0; a < 256; a++) {
      if (!alphabets(a)) continue;
      const SyntacticMonoid::ElementSet Ma = _monoid(M, _monoid(a));
      for (std::size_t b = 0; b < 256; b++) {
        if (!alphabets(b)) continue;
        const SyntacticMonoid::ElementSet bM = _monoid(_monoid(b), M);
        for (SyntacticMonoid::Element n = 0; n < _monoid.size(); n++) {
          const SyntacticMonoid::ElementSet ManbM
              = _monoid(Ma, _monoid(n, bM));
          if (ManbM.find(m) != ManbM.end()) continue;
          const SyntacticMonoid::ElementSet ManM
              = _monoid(Ma, _monoid(n, M));
          const SyntacticMonoid::ElementSet MnbM
              = _monoid(M, _monoid(n, bM));
          if (ManM.find(m) != ManM.end() && MnbM.find(m) != MnbM.end()) {
            tmp.push_back(
                std::string(1, a)+starfree_recursion(n, memo)+std::string(1, b));
          }
        }
      }
    }
    if (W_.empty() && tmp.empty()) {
      AWA << "@";
    } else {
      AWA << "!@(";
      if (!W_.empty()) {
        if (W_.size() == 1) {
          for (std::set<unsigned char>::iterator iter = W_.begin();
               iter != W_.end(); ++iter) {
            AWA << *iter;
          }          
        } else {
          AWA << "[";
          for (std::set<unsigned char>::iterator iter = W_.begin();
               iter != W_.end(); ++iter) {
            AWA << *iter;
          }
          AWA << "]";
        }
        if (!tmp.empty()) AWA << "|";
      }
      for (std::size_t i = 0; i < tmp.size(); i++) {
        AWA << tmp[i];
        if (i < tmp.size() - 1) AWA << "|";
      }
      AWA << ")!@";
    }
    
    // finally, build \phi^{-1}(m) = (U A* & A* V) \ A* W A*
    // note that: (A & B) \ C = !(!A | !B | C)
    regex << "!("
          <<     "!("<<UA.str()<<")" <<"|"<< "!("<<AV.str()<<")" <<"|"<< AWA.str()
        <<   ")";
  }

  memo[m] = regex.str();
  return memo[m];
}

} // namespace recon

using recon::RECON;
using recon::SyntacticMonoid;

#endif // RECON_H_
