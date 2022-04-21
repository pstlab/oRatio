const animation = false;

const sc = chroma.scale(['#90EE90', 'yellow', '#A91101']);
let max_cost = 1;
let color_domain = sc.domain([0, max_cost]);

class Reasoner {

    constructor(timelines_div, graph_div) {
        this.items = new Map();
        this.atoms = new Map();

        this.timelines = new vis.DataSet([]);
        this.timeline_values = new vis.DataSet([]);

        this.timeline = new vis.Timeline(timelines_div, this.timeline_values, this.timelines, { selectable: false, showCurrentTime: false, verticalScroll: true, zoomKey: 'ctrlKey', margin: { item: { horizontal: 0 } } });

        this.current_time = 0;
        this.timeline.addCustomTime(this.current_time);
        this.timeline.customTimes[this.timeline.customTimes.length - 1].hammer.off("panstart panmove panend");

        this.executing_tasks = new Set();

        this.nodes = new vis.DataSet([]);
        this.edges = new vis.DataSet([]);
        this.current_flaw;
        this.current_resolver;

        this.network = new vis.Network(graph_div, { nodes: this.nodes, edges: this.edges }, { layout: { hierarchical: { direction: "RL", } } });
    }

    state_changed(message) {
        this.items.clear(); if (message.state.items) for (const itm of message.state.items) this.items.set(parseInt(itm.id), itm);
        this.atoms.clear(); if (message.state.atoms) for (const atm of message.state.atoms) this.atoms.set(parseInt(atm.id), atm);
        this.executing_tasks.clear();
        this.timelines.update(message.timelines.map(tl => { return { id: tl.id, content: tl.name } }));
        const origin_var = message.state.exprs.find(xpr => xpr.name == 'origin');
        const horizon_var = message.state.exprs.find(xpr => xpr.name == 'horizon');
        const origin_val = origin_var.value.num / origin_var.value.den;
        const horizon_val = horizon_var.value.num / horizon_var.value.den;
        this.timeline.setWindow(origin_val - 10, origin_val == horizon_val ? horizon_val + 100 : horizon_val + 10);
        const vals = [];
        for (const tl of message.timelines)
            switch (tl.type) {
                case 'StateVariable': {
                    const sv_atms = new Set();
                    tl.values.forEach((val, id) => {
                        vals.push({
                            id: '' + tl.id + id,
                            className: sv_value_class(val),
                            start: val.from.num / val.from.den,
                            end: val.to.num / val.to.den,
                            type: 'background',
                            group: tl.id
                        });
                        for (const atm_id of val.atoms)
                            sv_atms.add(atm_id);
                    });
                    for (const atm_id of sv_atms) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        const end_var = atm.pars.find(xpr => xpr.name == 'end');
                        vals.push({
                            id: '' + atm.id,
                            content: this.atom_content(atm),
                            title: this.atom_title(atm),
                            start: start_var.value.num / start_var.value.den,
                            end: end_var.value.num / end_var.value.den,
                            group: tl.id
                        });
                    }
                    break;
                }
                case 'Agent':
                    for (const atm_id of tl.values) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        if (start_var) {
                            const end_var = atm.pars.find(xpr => xpr.name == 'end');
                            vals.push({
                                id: '' + atm.id,
                                content: this.atom_content(atm),
                                title: this.atom_title(atm),
                                start: start_var.value.num / start_var.value.den,
                                end: end_var.value.num / end_var.value.den,
                                group: tl.id
                            });
                        } else {
                            const at_var = atm.pars.find(xpr => xpr.name == 'at');
                            vals.push({
                                id: '' + atm.id,
                                content: this.atom_content(atm),
                                start: at_var.value.num / at_var.value.den,
                                group: tl.id
                            });
                        }
                    }
                    break;
                case 'ReusableResource':
                    const rr_atms = new Set();
                    tl.values.forEach((val, id) => {
                        vals.push({
                            id: '' + tl.id + id,
                            content: '' + val.usage.num / val.usage.den,
                            className: rr_value_class(tl, val),
                            start: val.from.num / val.from.den,
                            end: val.to.num / val.to.den,
                            type: 'background',
                            group: tl.id
                        });
                        for (const atm_id of val.atoms)
                            rr_atms.add(atm_id);
                    });
                    for (const atm_id of rr_atms) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        const end_var = atm.pars.find(xpr => xpr.name == 'end');
                        vals.push({
                            id: '' + atm.id,
                            content: this.atom_content(atm),
                            title: this.atom_title(atm),
                            start: start_var.value.num / start_var.value.den,
                            end: end_var.value.num / end_var.value.den,
                            group: tl.id
                        });
                    }
                    break;
                case 'ConsumableResource':
                    const cr_atms = new Set();
                    tl.values.forEach((val, id) => {
                        vals.push({
                            id: '' + tl.id + id,
                            content: val.start.num / val.start.den + ' - ' + val.end.num / val.end.den,
                            className: cr_value_class(tl, val),
                            start: val.from.num / val.from.den,
                            end: val.to.num / val.to.den,
                            type: 'background',
                            group: tl.id
                        });
                        for (const atm_id of val.atoms)
                            cr_atms.add(atm_id);
                    });
                    for (const atm_id of cr_atms) {
                        const atm = this.atoms.get(atm_id);
                        const start_var = atm.pars.find(xpr => xpr.name == 'start');
                        const end_var = atm.pars.find(xpr => xpr.name == 'end');
                        vals.push({
                            id: '' + atm.id,
                            content: this.atom_content(atm),
                            title: this.atom_title(atm),
                            start: start_var.value.num / start_var.value.den,
                            end: end_var.value.num / end_var.value.den,
                            group: tl.id
                        });
                    }
                    break;
                default:
                    break;
            }
        this.timeline_values.update(vals);
        for (const t of message.executing)
            this.executing_tasks.add(t);
        this.timeline.setSelection(Array.from(this.executing_tasks), { animate: animation });
        this.current_time = message.time.num / message.time.den;
        this.timeline.setCustomTime(this.current_time);
    }

    solution_found() {
        this.current_flaw = undefined;
        this.current_resolver = undefined;
        this.network.unselectAll();
    }

    inconsistent_problem() {
        this.current_flaw = undefined;
        this.current_resolver = undefined;
        this.network.unselectAll();
    }

    tick(message) {
        this.current_time = message.time.num / message.time.den;
        this.timeline.setCustomTime(this.current_time);
    }

    graph(message) {
        this.nodes.clear();
        this.edges.clear();
        this.current_flaw = undefined;
        this.current_resolver = undefined;

        for (const f of message.flaws) {
            const flaw = {
                type: 'flaw',
                id: f.id,
                causes: f.causes,
                state: f.state,
                cost: f.cost.num / f.cost.den,
                pos: f.pos,
                data: f.data
            };
            flaw.label = flaw_label(flaw);
            flaw.title = flaw_tooltip(flaw);
            flaw.shapeProperties = { borderDashes: stroke_dasharray(flaw) };
            if (flaw.cost != Number.POSITIVE_INFINITY && max_cost < flaw.cost)
                max_cost = flaw.cost;
            this.nodes.add(flaw);
        }
        color_domain = sc.domain([0, max_cost]);
        const all_nodes = this.nodes.get();
        all_nodes.forEach(n => n.color = color(n));
        this.nodes.update(all_nodes);

        for (const r of message.resolvers) {
            const resolver = {
                type: 'resolver',
                id: r.id,
                preconditions: r.preconditions,
                effect: r.effect,
                state: r.state,
                intrinsic_cost: r.intrinsic_cost.num / r.intrinsic_cost.den,
                data: r.data
            };
            resolver.cost = this.estimate_cost(resolver);
            resolver.label = resolver_label(resolver);
            resolver.title = resolver_tooltip(resolver);
            resolver.shape = 'box';
            resolver.shapeProperties = { borderDashes: stroke_dasharray(resolver) };
            resolver.color = color(resolver);
            this.nodes.add(resolver);
            this.edges.add({ from: r.id, to: r.effect, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
            for (const f of resolver.preconditions)
                this.edges.add({ from: f, to: resolver.id, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
        }

        if (message.current_flaw) {
            this.current_flaw = message.current_flaw;
            if (message.current_resolver) {
                this.current_resolver = message.current_resolver;
                this.network.selectNodes([this.current_flaw, this.current_resolver]);
                this.network.focus(this.current_resolver, { animation: animation });
            } else {
                this.current_resolver = undefined;
                this.network.selectNodes([this.current_flaw]);
                this.network.focus(this.current_flaw, { animation: animation });
            }
        }
    }

    flaw_created(message) {
        const flaw = {
            type: 'flaw',
            id: message.id,
            causes: message.causes,
            state: message.state,
            cost: message.cost.num / message.cost.den,
            pos: message.pos,
            data: message.data
        };
        flaw.label = flaw_label(flaw);
        flaw.title = flaw_tooltip(flaw);
        flaw.shapeProperties = { borderDashes: stroke_dasharray(message) };
        flaw.color = color(flaw);
        this.nodes.add(flaw);
        const causes = this.nodes.get(flaw.causes);
        const causes_edges = [];
        for (const c of causes) {
            c.preconditions.push(flaw.id);
            const c_res_cost = this.estimate_cost(c);
            if (c.cost != c_res_cost) {
                c.cost = c_res_cost;
                c.title = resolver_tooltip(c);
                c.color = color(c);
            }
            causes_edges.push({ from: message.id, to: c.id, arrows: { to: true }, dashes: stroke_dasharray(this.nodes.get(c)) });
        }
        this.edges.add(causes_edges);
        this.nodes.update(causes);
    }

    flaw_state_changed(message) {
        const flaw = this.nodes.get(message.id);
        flaw.state = message.state;
        flaw.shapeProperties.borderDashes = stroke_dasharray(message);
        flaw.color = color(flaw);
        this.nodes.update(flaw);
    }

    flaw_cost_changed(message) {
        const flaw = this.nodes.get(message.id);
        flaw.cost = message.cost.num / message.cost.den;
        flaw.title = flaw_tooltip(flaw);
        if (flaw.cost != Number.POSITIVE_INFINITY && max_cost < flaw.cost) {
            max_cost = flaw.cost;
            color_domain = sc.domain([0, max_cost]);
            const all_nodes = this.nodes.get();
            all_nodes.forEach(n => n.color = color(n));
            this.nodes.update(all_nodes);
        } else {
            flaw.color = color(flaw);
            this.nodes.update(flaw);
        }
        const updated_res = [];
        for (const c of flaw.causes.map(r_id => this.nodes.get(r_id))) {
            const c_res_cost = this.estimate_cost(c);
            if (c.cost != c_res_cost) {
                c.cost = c_res_cost;
                c.title = resolver_tooltip(c);
                c.color = color(c);
                updated_res.push(c);
            }
        }
        if (updated_res)
            this.nodes.update(updated_res);
    }

    flaw_position_changed(message) {
        const flaw = this.nodes.get(message.id);
        flaw.pos = message.pos;
        flaw.title = flaw_tooltip(flaw);
        this.nodes.update(flaw);
    }

    current_flaw_changed(message) {
        this.current_flaw = message.id;
        this.current_resolver = undefined;
        this.network.selectNodes([this.current_flaw]);
        this.network.focus(this.current_flaw, { animation: animation });
    }

    resolver_created(message) {
        message.cost = message.intrinsic_cost.num / message.intrinsic_cost.den;
        if (message.cost != Number.POSITIVE_INFINITY && max_cost < message.cost) {
            max_cost = message.cost;
            color_domain = sc.domain([0, max_cost]);
            const all_nodes = this.nodes.get();
            all_nodes.forEach(n => n.color = color_domain(n.cost));
            this.nodes.update(all_nodes);
        }
        const resolver = {
            type: 'resolver',
            id: message.id,
            preconditions: message.preconditions,
            effect: message.effect,
            state: message.state,
            intrinsic_cost: message.intrinsic_cost.num / message.intrinsic_cost.den,
            data: message.data
        };
        resolver.cost = this.estimate_cost(resolver);
        resolver.label = resolver_label(resolver);
        resolver.title = resolver_tooltip(resolver);
        resolver.shape = 'box';
        resolver.shapeProperties = { borderDashes: stroke_dasharray(resolver) };
        resolver.color = color(resolver);
        this.nodes.add(resolver);
        this.edges.add({ from: message.id, to: message.effect, arrows: { to: true }, dashes: stroke_dasharray(message) });
    }

    resolver_state_changed(message) {
        const resolver = this.nodes.get(message.id);
        resolver.state = message.state;
        resolver.shapeProperties.borderDashes = stroke_dasharray(resolver);
        resolver.color = color(resolver);
        this.nodes.update(resolver);
        const c_edges = this.network.getConnectedEdges(message.id);
        c_edges.forEach((e_id, i) => {
            c_edges[i] = this.edges.get(e_id);
            c_edges[i].dashes = stroke_dasharray(resolver);
        });
        this.edges.update(c_edges);
    }

    current_resolver_changed(message) {
        this.current_resolver = message.id;
        this.network.selectNodes([this.current_flaw, this.current_resolver]);
        this.network.focus(this.current_resolver, { animation: animation });
    }

    causal_link_added(message) {
        const flaw = this.nodes.get(message.flaw_id);
        const resolver = this.nodes.get(message.resolver_id);
        resolver.preconditions.push(flaw);
        this.edges.add({ from: message.flaw_id, to: message.resolver_id, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
    }

    estimate_cost(res) {
        return (res.preconditions ? Math.max.apply(Math, res.preconditions.map(f_id => this.nodes.get(f_id).cost)) : 0) + res.intrinsic_cost;
    }

    starting(message) {
        console.log('starting');
        for (const t of message.starting)
            console.log(this.atom_content(this.atoms.get(t)));
    }

    start(message) {
        for (const t of message.start)
            this.executing_tasks.add(t);
        this.timeline.setSelection(Array.from(this.executing_tasks), { animate: animation });
    }

    ending(message) {
        console.log('ending');
        for (const t of message.ending)
            console.log(this.atom_content(this.atoms.get(t)));
    }

    end(message) {
        for (const t of message.end)
            this.executing_tasks.delete(t);
        this.timeline.setSelection(Array.from(this.executing_tasks), { animate: animation });
    }

    executing_changed(message) {
        this.executing_tasks.clear();
        for (const t of message.executing)
            this.executing_tasks.add(t);
        this.timeline.setSelection(Array.from(this.executing_tasks), { animate: animation });
    }

    atom_content(atm) { return atm.predicate + '(' + atm.pars.filter(par => par.name != 'start' && par.name != 'end' && par.name != 'duration' && par.name != 'tau').map(par => par.name).sort().join(', ') + ')'; }

    atom_title(atm) { return '\u03C3' + atm.sigma + ' ' + atm.predicate + '(' + atm.pars.filter(par => par.name != 'tau').map(par => '<br>' + this.par_to_string(par)).sort().join(',') + '<br>)'; }

    par_to_string(par) {
        switch (par.type) {
            case 'bool': return par.name + ': ' + par.value;
            case 'real':
                const lb = par.value.lb ? par.value.lb.num / par.value.lb.den : Number.NEGATIVE_INFINITY;
                const ub = par.value.ub ? par.value.ub.num / par.value.ub.den : Number.POSITIVE_INFINITY;
                if (lb == ub)
                    return par.name + ': ' + par.value.num / par.value.den;
                else
                    return par.name + ': ' + par.value.num / par.value.den + ' [' + lb + ', ' + ub + ']';
            case 'string': return par.name + ': ' + par.value;
            default:
                if (typeof par.value == 'object') {
                    if (par.value.vals.length == 1) return par.name + ': ' + this.items.get(par.value.vals[0]).name;
                    else return par.name + ': [' + par.value.vals.map(itm_id => this.items.get(itm_id).name).sort().join(',') + ']';
                } else
                    return par.name + ': ' + this.items.get(par.value).name;
        }
    }
}

function sv_value_class(val) {
    switch (val.atoms.length) {
        case 0: return 'sv-empty';
        case 1: return 'sv-consistent';
        default: return 'sv-inconsistent';
    }
}

function rr_value_class(rr, val) { return rr.capacity.num / rr.capacity.den < val.usage.num / val.usage.den ? 'rr-inconsistent' : 'rr-consistent' }

function cr_value_class(cr, val) {
    if (cr.capacity.num / cr.capacity.den < val.start.num / val.start.den || cr.capacity.num / cr.capacity.den < val.end.num / val.end.den) return 'rr-inconsistent';
    if (0 > val.start.num / val.start.den || 0 > val.end.num / val.end.den) return 'rr-inconsistent';
    return 'rr-consistent'
}

function color(n) {
    switch (n.state) {
        case 0: // False
            return chroma('lightgray').hex();
        case 1: // True
        case 2: // Undefined
            if (n.cost < Number.POSITIVE_INFINITY)
                return color_domain(Math.min(max_cost, n.cost)).hex();
            else
                return chroma('darkgray').hex();
        default:
            break;
    }
}

function stroke_dasharray(n) {
    switch (n.state) {
        case 0: // False
            return [2, 2];
        case 1: // True
            return false;
        case 2: // Undefined
            return [4, 4];
        default:
            break;
    }
}

function flaw_label(flaw) {
    switch (flaw.data.type) {
        case 'fact':
            return 'fact \u03C3' + flaw.data.sigma + ' ' + flaw.data.predicate;
        case 'goal':
            return 'goal \u03C3' + flaw.data.sigma + ' ' + flaw.data.predicate;
        case 'enum':
            return 'enum';
        case 'bool':
            return 'bool';
        default:
            switch (flaw.data.phi) {
                case 'b0':
                case '\u00ACb0':
                    return flaw.data.type;
                default:
                    return flaw.data.phi.replace('b', '\u03C6') + ' ' + flaw.data.type;
            }
    }
}

function flaw_tooltip(flaw) {
    switch (flaw.data.phi) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + flaw.cost + ', pos: ' + flaw.pos.lb;
        default:
            return flaw.data.phi.replace('b', '\u03C6') + ', cost: ' + flaw.cost + ', pos: ' + flaw.pos.lb;
    }
}

function resolver_label(resolver) {
    if (resolver.data.type)
        switch (resolver.data.type) {
            case 'activate':
                return 'activate';
            case 'unify':
                return 'unify';
            case 'assignment':
                return resolver.data.val;
            default:
                switch (resolver.data.rho) {
                    case 'b0':
                    case '\u00ACb0':
                        return resolver.data.type;
                    default:
                        return resolver.data.rho.replace('b', '\u03C1') + ' ' + resolver.data.type;
                }
        }
    switch (resolver.data.rho) {
        case 'b0':
            return '\u22A4';
        case '\u00ACb0':
            return '\u22A5';
        default:
            return resolver.data.rho.replace('b', '\u03C1');
    }
}

function resolver_tooltip(resolver) {
    switch (resolver.data.rho) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + resolver.cost;
        default:
            return resolver.data.rho.replace('b', '\u03C1') + ', cost: ' + resolver.cost;
    }
}