const sc = chroma.scale(['#90EE90', 'yellow', '#A91101']);
let max_cost = 1;
let color_domain = sc.domain([0, max_cost]);

const items = new Map(), atoms = new Map();

const timelines = new vis.DataSet([]);
const timeline_values = new vis.DataSet([]);

const nodes = new vis.DataSet([]);
const edges = new vis.DataSet([]);

const options = {};

const timeline = new vis.Timeline(document.getElementById('timelines'), timeline_values, timelines, options);
timeline.currentTime.stop();
const network = new vis.Network(document.getElementById('graph'), { nodes: nodes, edges: edges }, options);

let current_time = 0, current_flaw, current_resolver;
timeline.setCurrentTime(current_time);

let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'state_changed': {
                items.clear(); for (const itm of c_msg.state.items) items.set(parseInt(itm.id), itm);
                atoms.clear(); for (const atm of c_msg.state.atoms) atoms.set(parseInt(atm.id), atm);
                timelines.update(c_msg.timelines.map(tl => { return { id: tl.id, content: tl.name } }));
                const origin_var = c_msg.state.exprs.find(xpr => xpr.name == 'origin');
                const horizon_var = c_msg.state.exprs.find(xpr => xpr.name == 'horizon');
                const origin_val = origin_var.value.val.num / origin_var.value.val.den;
                const horizon_val = horizon_var.value.val.num / horizon_var.value.val.den;
                timeline.setWindow(origin_val - 10, origin_val == horizon_val ? horizon_val + 100 : horizon_val + 10);
                const vals = [];
                for (const tl of c_msg.timelines)
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
                                const atm = atoms.get(atm_id);
                                const start_var = atm.pars.find(xpr => xpr.name == 'start');
                                const end_var = atm.pars.find(xpr => xpr.name == 'end');
                                vals.push({
                                    id: '' + tl.id + atm.id,
                                    content: atom_content(atm),
                                    title: atom_title(atm),
                                    start: start_var.value.val.num / start_var.value.val.den,
                                    end: end_var.value.val.num / end_var.value.val.den,
                                    group: tl.id
                                });
                            }
                            break;
                        }
                        case 'Agent':
                            for (const atm_id of tl.values) {
                                const atm = atoms.get(atm_id);
                                const start_var = atm.pars.find(xpr => xpr.name == 'start');
                                if (start_var) {
                                    const end_var = atm.pars.find(xpr => xpr.name == 'end');
                                    vals.push({
                                        id: '' + tl.id + atm.id,
                                        content: atom_content(atm),
                                        title: atom_title(atm),
                                        start: start_var.value.val.num / start_var.value.val.den,
                                        end: end_var.value.val.num / end_var.value.val.den,
                                        group: tl.id
                                    });
                                } else {
                                    const at_var = atm.pars.find(xpr => xpr.name == 'at');
                                    vals.push({
                                        id: '' + tl.id + atm.id,
                                        content: atom_content(atm),
                                        start: at_var.value.val.num / at_var.value.val.den,
                                        group: tl.id
                                    });
                                }
                            }
                            break;
                        case 'ReusableResource':
                            const sv_atms = new Set();
                            tl.values.forEach((val, id) => {
                                vals.push({
                                    id: '' + tl.id + id,
                                    content: val.usage.num / val.usage.den,
                                    className: rr_value_class(tl, val),
                                    start: val.from.num / val.from.den,
                                    end: val.to.num / val.to.den,
                                    type: 'background',
                                    group: tl.id
                                });
                                for (const atm_id of val.atoms)
                                    sv_atms.add(atm_id);
                            });
                            for (const atm_id of sv_atms) {
                                const atm = atoms.get(atm_id);
                                const start_var = atm.pars.find(xpr => xpr.name == 'start');
                                const end_var = atm.pars.find(xpr => xpr.name == 'end');
                                vals.push({
                                    id: '' + tl.id + atm.id,
                                    content: atom_content(atm),
                                    title: atom_title(atm),
                                    start: start_var.value.val.num / start_var.value.val.den,
                                    end: end_var.value.val.num / end_var.value.val.den,
                                    group: tl.id
                                });
                            }
                            break;
                        default:
                            break;
                    }
                timeline_values.update(vals);
                timeline.setCurrentTime(current_time);
                break;
            }
            case 'started_solving':
                console.log('solving the problem..');
                break;
            case 'solution_found':
                console.log('hurray!! we have found a solution..');
                current_flaw = undefined;
                current_resolver = undefined;
                network.unselectAll();
                break;
            case 'inconsistent_problem':
                console.log('the problem has no solution..');
                current_flaw = undefined;
                current_resolver = undefined;
                network.unselectAll();
                break;
            case 'flaw_created': {
                const flaw = {
                    type: 'flaw',
                    id: c_msg.id,
                    causes: c_msg.causes,
                    state: c_msg.state,
                    cost: c_msg.cost.num / c_msg.cost.den,
                    pos: c_msg.pos,
                    data: c_msg.data
                };
                flaw.label = flaw_label(flaw);
                flaw.title = flaw_tooltip(flaw);
                flaw.shapeProperties = { borderDashes: stroke_dasharray(c_msg) };
                flaw.color = color(flaw);
                nodes.add(flaw);
                const causes = nodes.get(flaw.causes);
                const causes_edges = [];
                for (const c of causes) {
                    c.preconditions.push(flaw.id);
                    const c_res_cost = estimate_cost(c);
                    if (c.cost != c_res_cost) {
                        c.cost = c_res_cost;
                        c.title = resolver_tooltip(c);
                        c.color = color(c);
                    }
                    causes_edges.push({ from: c_msg.id, to: c.id, arrows: { to: true }, dashes: stroke_dasharray(nodes.get(c)) });
                }
                edges.add(causes_edges);
                nodes.update(causes);
                break;
            }
            case 'flaw_state_changed': {
                const flaw = nodes.get(c_msg.id);
                flaw.state = c_msg.state;
                flaw.shapeProperties.borderDashes = stroke_dasharray(c_msg);
                flaw.color = color(flaw);
                nodes.update(flaw);
                break;
            }
            case 'flaw_cost_changed': {
                const flaw = nodes.get(c_msg.id);
                flaw.cost = c_msg.cost.num / c_msg.cost.den;
                flaw.title = flaw_tooltip(flaw);
                if (flaw.cost != Number.POSITIVE_INFINITY && max_cost < flaw.cost) {
                    max_cost = flaw.cost;
                    color_domain = sc.domain([0, max_cost]);
                    const all_nodes = nodes.get();
                    all_nodes.forEach(n => n.color = color(n));
                    nodes.update(all_nodes);
                } else {
                    flaw.color = color(flaw);
                    nodes.update(flaw);
                }
                const updated_res = [];
                for (const c of flaw.causes.map(r_id => nodes.get(r_id))) {
                    const c_res_cost = estimate_cost(c);
                    if (c.cost != c_res_cost) {
                        c.cost = c_res_cost;
                        c.title = resolver_tooltip(c);
                        c.color = color(c);
                        updated_res.push(c);
                    }
                }
                if (updated_res)
                    nodes.update(updated_res);
                break;
            }
            case 'flaw_position_changed': {
                const flaw = nodes.get(c_msg.id);
                flaw.pos = c_msg.pos;
                flaw.title = flaw_tooltip(flaw);
                nodes.update(flaw);
                break;
            }
            case 'current_flaw': {
                current_flaw = c_msg.id;
                current_resolver = undefined;
                network.selectNodes([current_flaw]);
                network.focus(current_flaw, { animation: true });
                break;
            }
            case 'resolver_created': {
                c_msg.cost = c_msg.intrinsic_cost.num / c_msg.intrinsic_cost.den;
                if (c_msg.cost != Number.POSITIVE_INFINITY && max_cost < c_msg.cost) {
                    max_cost = c_msg.cost;
                    color_domain = sc.domain([0, max_cost]);
                    const all_nodes = nodes.get();
                    all_nodes.forEach(n => n.color = color_domain(n.cost));
                    nodes.update(all_nodes);
                }
                const resolver = {
                    type: 'resolver',
                    id: c_msg.id,
                    preconditions: c_msg.preconditions,
                    effect: c_msg.effect,
                    state: c_msg.state,
                    intrinsic_cost: c_msg.intrinsic_cost.num / c_msg.intrinsic_cost.den,
                    data: c_msg.data
                };
                resolver.cost = estimate_cost(resolver);
                resolver.label = resolver_label(resolver);
                resolver.title = resolver_tooltip(resolver);
                resolver.shape = 'box';
                resolver.shapeProperties = { borderDashes: stroke_dasharray(resolver) };
                resolver.color = color(resolver);
                nodes.add(resolver);
                edges.add({ from: c_msg.id, to: c_msg.effect, arrows: { to: true }, dashes: stroke_dasharray(c_msg) });
                break;
            }
            case 'resolver_state_changed': {
                const resolver = nodes.get(c_msg.id);
                resolver.state = c_msg.state;
                resolver.shapeProperties.borderDashes = stroke_dasharray(resolver);
                resolver.color = color(resolver);
                nodes.update(resolver);
                const c_edges = network.getConnectedEdges(c_msg.id);
                c_edges.forEach((e_id, i) => {
                    c_edges[i] = edges.get(e_id);
                    c_edges[i].dashes = stroke_dasharray(resolver);
                });
                edges.update(c_edges);
                break;
            }
            case 'current_resolver': {
                current_resolver = c_msg.id;
                network.selectNodes([current_flaw, current_resolver]);
                network.focus(current_resolver, { animation: true });
                break;
            }
            case 'causal_link_added': {
                const flaw = nodes.get(c_msg.flaw_id);
                const resolver = nodes.get(c_msg.resolver_id);
                resolver.preconditions.push(flaw);
                edges.add({ from: c_msg.flaw_id, to: c_msg.resolver_id, arrows: { to: true }, dashes: stroke_dasharray(resolver) });
                break;
            }
            case 'tick':
                current_time = c_msg.time.num / c_msg.time.den;
                timeline.setCurrentTime(current_time);
                break;
        }
    };
    ws.onclose = () => setTimeout(setup_ws, 1000);
}

function sv_value_class(val) {
    switch (val.atoms.length) {
        case 0: return 'sv-empty';
        case 1: return 'sv-consistent';
        default: return 'sv-inconsistent';
    }
}

function rr_value_class(rr, val) { return rr.capacity.num / rr.capacity.den < val.usage.num / val.usage.den ? 'rr-inconsistent' : 'rr-consistent' }

function atom_content(atm) { return atm.predicate + '(' + atm.pars.filter(par => par.name != 'start' && par.name != 'end' && par.name != 'duration' && par.name != 'tau').map(par => par.name).sort().join(', ') + ')'; }

function atom_title(atm) { return '\u03C3' + atm.sigma + ' ' + atm.predicate + '(' + atm.pars.filter(par => par.name != 'start' && par.name != 'end' && par.name != 'duration' && par.name != 'tau').map(par => par_to_string(par)).sort().join(', ') + ')'; }

function par_to_string(par) {
    switch (par.type) {
        case 'bool': return par.name + ': ' + par.value;
        case 'real': return par.name + ': ' + par.value.num / par.value.den;
        default: return par.name;
    }
}

function estimate_cost(res) {
    return (res.preconditions ? Math.max.apply(Math, res.preconditions.map(f_id => nodes.get(f_id).cost)) : 0) + res.intrinsic_cost;
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