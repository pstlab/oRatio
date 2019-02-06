/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.cnr.istc.riddle;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import org.antlr.v4.runtime.ANTLRErrorListener;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.Parser;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;
import org.antlr.v4.runtime.atn.ATNConfigSet;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.tree.ParseTreeProperty;
import org.antlr.v4.runtime.tree.ParseTreeWalker;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class Core implements Scope {

    private static final Logger LOG = Logger.getLogger(Core.class.getName());
    public static final String BOOL = "bool";
    public static final String INT = "int";
    public static final String REAL = "real";
    public static final String STRING = "string";
    final Map<String, Field> fields = new LinkedHashMap<>();
    final Map<String, Collection<Method>> methods = new HashMap<>();
    final Map<String, Type> types = new LinkedHashMap<>();
    final Map<String, Predicate> predicates = new LinkedHashMap<>();
    final Map<String, Item> exprs = new LinkedHashMap<>();
    final Map<Item, String> expr_names = new IdentityHashMap<>();
    final ParseTreeProperty<Scope> scopes = new ParseTreeProperty<>();

    public Core() {
        // we add the primitive types..
        types.put(BOOL, new Type(this, this, BOOL));
        types.put(INT, new Type(this, this, INT));
        types.put(REAL, new Type(this, this, REAL));
        types.put(STRING, new Type(this, this, STRING));

        predicates.put("ImpulsivePredicate",
                new Predicate(this, this, "ImpulsivePredicate", new Field(types.get(REAL), "at")));
        predicates.put("IntervalPredicate",
                new Predicate(this, this, "IntervalPredicate", new Field(types.get(REAL), "start"),
                        new Field(types.get(REAL), "end"), new Field(types.get(REAL), "duration")));

        // we add some complex types..
        Type sv = new Type(this, this, "StateVariable");
        types.put(sv.name, sv);

        Type rr = new Type(this, this, "ReusableResource");
        Predicate rr_use = new Predicate(this, rr, "Use", new Field(types.get(REAL), "amount"));
        rr_use.superclasses.add(predicates.get("IntervalPredicate"));
        rr.predicates.put(rr_use.name, rr_use);
        types.put(rr.name, rr);

        Type ps = new Type(this, this, "PropositionalState");
        Predicate ps_use = new Predicate(this, ps, "PropositionalStatePredicate",
                new Field(types.get(BOOL), "polarity"));
        ps_use.superclasses.add(predicates.get("IntervalPredicate"));
        ps.predicates.put(ps_use.name, ps_use);
        types.put(ps.name, ps);

        Type pa = new Type(this, this, "PropositionalAgent");
        types.put(pa.name, pa);
    }

    @Override
    public Core getCore() {
        return this;
    }

    @Override
    public Scope getScope() {
        return this;
    }

    @Override
    public Field getField(final String name) throws NoSuchFieldException {
        Field field = fields.get(name);
        if (field != null)
            return field;
        else
            throw new NoSuchFieldException(name);
    }

    @Override
    public Map<String, Field> getFields() {
        return Collections.unmodifiableMap(fields);
    }

    @Override
    public Method getMethod(final String name, final Type... parameterTypes) throws NoSuchMethodException {
        boolean is_correct;
        if (methods.containsKey(name))
            for (Method m : methods.get(name))
                if (m.pars.length == parameterTypes.length) {
                    is_correct = true;
                    for (int i = 0; i < m.pars.length; i++)
                        if (!m.pars[i].type.isAssignableFrom(parameterTypes[i])) {
                            is_correct = false;
                            break;
                        }
                    if (is_correct)
                        return m;
                }

        // not found
        throw new NoSuchMethodException(name);
    }

    @Override
    public Collection<Method> getMethods() {
        Collection<Method> c_methods = new ArrayList<>(methods.size());
        methods.values().forEach(ms -> c_methods.addAll(ms));
        return Collections.unmodifiableCollection(c_methods);
    }

    void defineMethod(final Method method) {
        if (!methods.containsKey(method.name))
            methods.put(method.name, new ArrayList<>());
        methods.get(method.name).add(method);
    }

    @Override
    public Type getType(String name) throws ClassNotFoundException {
        Type type = types.get(name);
        if (type != null)
            return type;

        // not found
        throw new ClassNotFoundException(name);
    }

    @Override
    public Map<String, Type> getTypes() {
        return Collections.unmodifiableMap(types);
    }

    @Override
    public Predicate getPredicate(final String name) {
        return predicates.get(name);
    }

    @Override
    public Map<String, Predicate> getPredicates() {
        return Collections.unmodifiableMap(predicates);
    }

    public Map<String, Item> getExprs() {
        return Collections.unmodifiableMap(exprs);
    }

    /**
     * @param name the name of the field identifying the desired item.
     * @return the item having the given name
     */
    public Item getExpr(final String name) {
        return exprs.get(name);
    }

    public String guessName(final Item itm) {
        return expr_names.get(itm);
    }

    public void read(final String script) {
        CodeSnippet snippet = new CodeSnippet(null,
                new riddleParser(new CommonTokenStream(new riddleLexer(CharStreams.fromString(script)))));
        ParseTreeWalker.DEFAULT.walk(new TypeDeclarationListener(this, snippet.parser), snippet.cu);
        ParseTreeWalker.DEFAULT.walk(new TypeRefinementListener(this, snippet.parser), snippet.cu);
    }

    public void read(final File... files) throws IOException {
        List<File> fs = new ArrayList<>(files.length);
        for (File file : files)
            fs.addAll(Files.walk(Paths.get(file.toURI())).filter(Files::isRegularFile).map(path -> path.toFile())
                    .collect(Collectors.toList()));

        Collection<CodeSnippet> snippets = new ArrayList<>(fs.size());
        for (File f : fs)
            snippets.add(new CodeSnippet(f,
                    new riddleParser(new CommonTokenStream(new riddleLexer(CharStreams.fromPath(f.toPath()))))));
        for (CodeSnippet snippet : snippets) {
            ParseTreeWalker.DEFAULT.walk(new TypeDeclarationListener(this, snippet.parser), snippet.cu);
            ParseTreeWalker.DEFAULT.walk(new TypeRefinementListener(this, snippet.parser), snippet.cu);
        }
    }

    private static class CodeSnippet implements ANTLRErrorListener {

        private final File file;
        private final riddleParser parser;
        private final riddleParser.Compilation_unitContext cu;

        CodeSnippet(File file, riddleParser parser) {
            this.file = file;
            this.parser = parser;
            this.parser.addErrorListener(this);
            this.cu = parser.compilation_unit();
        }

        @Override
        public void syntaxError(Recognizer<?, ?> recognizer, Object offendingSymbol, int line, int charPositionInLine,
                String msg, RecognitionException e) {
            LOG.severe(msg);
        }

        @Override
        public void reportAmbiguity(Parser recognizer, DFA dfa, int startIndex, int stopIndex, boolean exact,
                BitSet ambigAlts, ATNConfigSet configs) {
        }

        @Override
        public void reportAttemptingFullContext(Parser recognizer, DFA dfa, int startIndex, int stopIndex,
                BitSet conflictingAlts, ATNConfigSet configs) {
        }

        @Override
        public void reportContextSensitivity(Parser recognizer, DFA dfa, int startIndex, int stopIndex, int prediction,
                ATNConfigSet configs) {
        }
    }
}
