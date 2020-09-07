package it.cnr.istc.pst.oratio.riddle;

public class Constructor extends BaseScope {

    final Field[] pars;

    Constructor(final Core core, final Scope scope, final Field... parameters) {
        super(core, scope);
        this.pars = parameters;

        for (Field par : parameters)
            fields.put(par.getName(), par);
    }

    public Field[] getParemeters() {
        return pars;
    }
}
