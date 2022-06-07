#include "rbs_lexer.h"

namespace rbs
{
    RBS_EXPORT lexer::lexer(std::istream &is)
    {
        char buffer[1024];
        while (is.read(buffer, sizeof(buffer)))
            sb.append(buffer, sizeof(buffer));
        sb.append(buffer, is.gcount());
        ch = next_char();
    }

    RBS_EXPORT token *lexer::next()
    {
        switch (ch)
        {
        case '"':
        {
            // string literal..
            std::string str;
            while (true)
                switch (ch = next_char())
                {
                case '"':
                    ch = next_char();
                    return mk_string_token(str);
                case '\\':
                    // read escaped char..
                    str += next_char();
                    break;
                case '\r':
                case '\n':
                    error("newline in string literal..");
                    return nullptr;
                default:
                    str += ch;
                }
        }
        case '/':
            switch (ch = next_char())
            {
            case '/': // in single-line comment
                while (true)
                    switch (ch = next_char())
                    {
                    case '\r':
                    case '\n':
                        return next();
                    case -1:
                        return mk_token(EOF_ID);
                    }
            case '*': // in multi-line comment
                while (true)
                    switch (ch = next_char())
                    {
                    case '*':
                        if ((ch = next_char()) == '/')
                        {
                            ch = next_char();
                            return next();
                        }
                        break;
                    }
            }
            return mk_token(SLASH_ID);
        case '>':
            if ((ch = next_char()) == '=')
            {
                ch = next_char();
                return mk_token(GTEQ_ID);
            }
            return mk_token(GT_ID);
        case '<':
            if ((ch = next_char()) == '=')
            {
                ch = next_char();
                return mk_token(LTEQ_ID);
            }
            return mk_token(LT_ID);
        case '+':
            ch = next_char();
            return mk_token(PLUS_ID);
        case '-':
            ch = next_char();
            return mk_token(MINUS_ID);
        case '*':
            ch = next_char();
            return mk_token(STAR_ID);
        case '|':
            ch = next_char();
            return mk_token(BAR_ID);
        case '&':
            ch = next_char();
            return mk_token(AMP_ID);
        case '!':
            if ((ch = next_char()) == '=')
            {
                ch = next_char();
                return mk_token(BANGEQ_ID);
            }
            return mk_token(BANG_ID);
        case '.':
            ch = next_char();
            if ('0' <= ch && ch <= '9')
            {
                // in a number literal..
                std::string dec;
                dec += ch;
                while (true)
                {
                    switch (ch = next_char())
                    {
                    case '0':
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        dec += ch;
                        break;
                    case '.':
                        error("invalid numeric literal..");
                        return nullptr;
                    default:
                        return mk_rational_token("", dec);
                    }
                }
            }
            return mk_token(DOT_ID);
        case ',':
            ch = next_char();
            return mk_token(COMMA_ID);
        case ';':
            ch = next_char();
            return mk_token(SEMICOLON_ID);
        case ':':
            ch = next_char();
            return mk_token(COLON_ID);
        case '(':
            ch = next_char();
            return mk_token(LPAREN_ID);
        case ')':
            ch = next_char();
            return mk_token(RPAREN_ID);
        case '[':
            ch = next_char();
            return mk_token(LBRACKET_ID);
        case ']':
            ch = next_char();
            return mk_token(RBRACKET_ID);
        case '=':
            if ((ch = next_char()) == '>')
            {
                ch = next_char();
                return mk_token(IMPLICATION_ID);
            }
            return mk_token(EQ_ID);
        case '0': // in a number literal..
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        {
            std::string intgr; // the integer part..
            intgr += ch;
            while (true)
                switch (ch = next_char())
                {
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    intgr += ch;
                    break;
                case '.':
                {
                    std::string dcml; // the decimal part..
                    while (true)
                        switch (ch = next_char())
                        {
                        case '0':
                        case '1':
                        case '2':
                        case '3':
                        case '4':
                        case '5':
                        case '6':
                        case '7':
                        case '8':
                        case '9':
                            dcml += ch;
                            break;
                        case '.':
                            error("invalid numeric literal..");
                            return nullptr;
                        default:
                            return mk_rational_token(intgr, dcml);
                        }
                }
                default:
                    return mk_integer_token(intgr);
                }
        }
        case 'a':
        {
            std::string str;
            if (str += ch; (ch = next_char()) != 's')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 's')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'e')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'r')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 't')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != -1 && is_id_part(ch))
                return finish_id(str);
            else
                return mk_token(ASSERT_ID);
        }
        case 'f':
        {
            std::string str;
            if (str += ch; (ch = next_char()) != 'a')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'l')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 's')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'e')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != -1 && is_id_part(ch))
                return finish_id(str);
            else
                return mk_bool_token(false);
        }
        case 'r':
        {
            std::string str;
            if (str += ch; (ch = next_char()) != 'e')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 't')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'r')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'a')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'c')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 't')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != -1 && is_id_part(ch))
                return finish_id(str);
            else
                return mk_token(RETRACT_ID);
        }
        case 't':
        {
            std::string str;
            if (str += ch; (ch = next_char()) != 'r')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'u')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != 'e')
                return finish_id(str);
            if (str += ch; (ch = next_char()) != -1 && is_id_part(ch))
                return finish_id(str);
            else
                return mk_bool_token(true);
        }
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'g':
        case 'h':
        case 'i':
        case 'j':
        case 'k':
        case 'l':
        case 'm':
        case 'n':
        case 'o':
        case 'p':
        case 'q':
        case 's':
        case 'u':
        case 'v':
        case 'w':
        case 'x':
        case 'y':
        case 'z':
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        case 'G':
        case 'H':
        case 'I':
        case 'J':
        case 'K':
        case 'L':
        case 'M':
        case 'N':
        case 'O':
        case 'P':
        case 'Q':
        case 'R':
        case 'S':
        case 'T':
        case 'U':
        case 'V':
        case 'W':
        case 'X':
        case 'Y':
        case 'Z':
        case '_':
        {
            std::string str;
            return finish_id(str);
        }
        case '\t':
        case ' ':
        case '\r':
        case '\n':
            while (true)
                switch (ch = next_char())
                {
                case ' ':
                case '\t':
                case '\r':
                case '\n':
                    break;
                case -1:
                    return mk_token(EOF_ID);
                default:
                    return next();
                }
        case -1:
            return mk_token(EOF_ID);
        default:
            error("invalid token..");
            return nullptr;
        }
    }

    char lexer::next_char() noexcept
    {
        if (pos == sb.length())
            return -1;
        switch (sb[pos])
        {
        case ' ':
            start_pos++;
            end_pos++;
            break;
        case '\t':
            start_pos += 4 - (start_pos % 4);
            end_pos += 4 - (end_pos % 4);
            break;
        case '\r':
            if (pos + 1 != sb.length() && sb[pos + 1] == '\n')
            {
                pos++;
                end_line++;
                end_pos = 0;
                break;
            }
            [[fallthrough]];
        case '\n':
            end_line++;
            end_pos = 0;
            break;
        default:
            end_pos++;
            break;
        }
        return sb[pos++];
    }

    token *lexer::finish_id(std::string &str) noexcept
    {
        if (!is_id_part(ch))
            return mk_id_token(str);
        str += ch;
        while ((ch = next_char()) != -1 && is_id_part(ch))
            str += ch;
        return mk_id_token(str);
    }

    void lexer::error(const std::string &err) { throw std::invalid_argument("[" + std::to_string(start_line) + ", " + std::to_string(start_pos) + "] " + err); }
} // namespace rbs
