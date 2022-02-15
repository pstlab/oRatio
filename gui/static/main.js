const sc = chroma.scale(['#90EE90', 'yellow', 'red']);
let max_cost = 1;
let color_domain = sc.domain([0, max_cost]);

const nodes = new vis.DataSet([]);
const edges = new vis.DataSet([]);

const data = {
    nodes: nodes,
    edges: edges
};
const options = {};

const network = new vis.Network(document.getElementById('graph'), data, options);

let current_flaw, current_resolver;

let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'started_solving':
                console.log('solving the problem..');
                break;
            case 'solution_found':
                console.log('hurray!! we have found a solution..');
                network.unselectAll();
                break;
            case 'inconsistent_problem':
                console.log('the problem has no solution..');
                network.unselectAll();
                break;
            case 'flaw_created': {
                c_msg.cost = Number.POSITIVE_INFINITY;
                const flaw = {
                    id: c_msg.id,
                    type: 'flaw',
                    label: flaw_label(c_msg),
                    title: flaw_tooltip(c_msg),
                    shapeProperties: { borderDashes: stroke_dasharray(c_msg) },
                    color: color_domain(max_cost),
                    data: c_msg
                };
                nodes.add(flaw);
                const causes = [];
                for (const c of c_msg.causes) {
                    c.data.preconditions.push(flaw);
                    causes.push({
                        from: c_msg.id,
                        to: c,
                        arrows: { to: true },
                        dashes: stroke_dasharray(nodes.get(c))
                    });
                }
                edges.add(causes);
                break;
            }
            case 'flaw_state_changed': {
                const flaw = nodes.get(c_msg.id);
                flaw.data.state = c_msg.state;
                flaw.shapeProperties.borderDashes = stroke_dasharray(c_msg);
                flaw.color = color(flaw);
                nodes.update(flaw);
                break;
            }
            case 'flaw_cost_changed': {
                const flaw = nodes.get(c_msg.id);
                flaw.data.cost = c_msg.cost.num / c_msg.cost.den;
                if (flaw.data.cost != Number.POSITIVE_INFINITY && max_cost < flaw.data.cost) {
                    max_cost = flaw.data.cost;
                    color_domain = sc.domain([0, max_cost]);
                    const all_nodes = nodes.get();
                    all_nodes.forEach(n => n.color = color(n));
                    nodes.update(all_nodes);
                } else {
                    flaw.color = color(flaw);
                    nodes.update(flaw);
                }
                const updated_res = [];
                for (const c of flaw.data.causes) {
                    const c_res = nodes.get(c);
                    const c_res_cost = c_res.estimate_cost();
                    if (c_res.data.cost != c_res_cost) {
                        c_res.data.cost = c_res_cost;
                        c_res.color = color(c_res);
                        updated_res.push(c_res);
                    }
                }
                if (updated_res.length > 0)
                    nodes.update(updated_res);
                break;
            }
            case 'flaw_position_changed': {
                const flaw = nodes.get(c_msg.id);
                flaw.data.pos = c_msg.pos;
                flaw.title = flaw_tooltip(c_msg);
                nodes.update(flaw);
                break;
            }
            case 'resolver_created': {
                c_msg.preconditions = [];
                c_msg.cost = c_msg.intrinsic_cost.num / c_msg.intrinsic_cost.den;
                if (c_msg.cost != Number.POSITIVE_INFINITY && max_cost < c_msg.cost) {
                    max_cost = c_msg.cost;
                    color_domain = sc.domain([0, max_cost]);
                    const all_nodes = nodes.get();
                    all_nodes.forEach(n => n.color = color_domain(n.data.cost));
                    nodes.update(all_nodes);
                }
                const resolver = {
                    id: c_msg.id,
                    type: 'resolver',
                    label: resolver_label(c_msg),
                    title: resolver_tooltip(c_msg),
                    shape: 'box',
                    shapeProperties: { borderDashes: stroke_dasharray(c_msg) },
                    estimate_cost: function () {
                        const max_flaw = this.data.preconditions.reduce((f0, f1) => { return (f0.data.cost > f1.data.cost) ? f0 : f1 });
                        if (max_flaw)
                            return max_flaw.data.cost + this.data.intrinsic_cost;
                        else
                            return this.data.intrinsic_cost;
                    },
                    color: color_domain(c_msg.cost),
                    data: c_msg
                };
                nodes.add(resolver);
                const effect = {
                    from: c_msg.id,
                    to: c_msg.effect,
                    arrows: { to: true },
                    dashes: stroke_dasharray(c_msg)
                };
                edges.add(effect);
                break;
            }
            case 'resolver_state_changed': {
                const resolver = nodes.get(c_msg.id);
                resolver.data.state = c_msg.state;
                resolver.shapeProperties.borderDashes = stroke_dasharray(c_msg);
                resolver.color = color(resolver);
                nodes.update(resolver);
                const c_edges = network.getConnectedEdges(c_msg.id);
                c_edges.forEach((e_id, i) => {
                    c_edges[i] = edges.get(e_id);
                    c_edges[i].dashes = stroke_dasharray(c_msg);
                });
                edges.update(c_edges);
                break;
            }
            case 'causal_link_added': {
                const flaw = nodes.get(c_msg.flaw_id);
                const resolver = nodes.get(c_msg.resolver_id);
                resolver.preconditions.push(flaw);
                const link = {
                    from: c_msg.flaw_id,
                    to: c_msg.resolver_id,
                    arrows: { to: true },
                    dashes: stroke_dasharray(resolver)
                };
                edges.add(link);
                break;
            }
        }
    };
    ws.onclose = () => setTimeout(setup_ws, 1000);
}

function color(n) {
    switch (n.state) {
        case 0: // False
            return chroma('light-gray').hex();
        case 1: // True
        case 2: // Undefined
            return color_domain(Math.min(max_cost, n.data.cost)).hex();
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
    switch (flaw.label.type) {
        case 'fact':
            return 'fact \u03C3' + flaw.label.sigma + ' ' + flaw.label.predicate;
        case 'goal':
            return 'goal \u03C3' + flaw.label.sigma + ' ' + flaw.label.predicate;
        case 'enum':
            return 'enum';
        case 'bool':
            return 'bool';
        default:
            switch (flaw.label.phi) {
                case 'b0':
                case '\u00ACb0':
                    return flaw.label.type;
                default:
                    return flaw.label.phi.replace('b', '\u03C6') + ' ' + flaw.label.type;
            }
    }
}

function flaw_tooltip(flaw) {
    switch (flaw.label.phi) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + (flaw.cost.num / flaw.cost.den) + ', pos: ' + flaw.pos.lb;
        default:
            return flaw.label.phi.replace('b', '\u03C6') + ', cost: ' + (flaw.cost.num / flaw.cost.den) + ', pos: ' + flaw.pos.lb;
    }
}

function resolver_label(resolver) {
    if (resolver.label.type)
        switch (resolver.label.type) {
            case 'activate':
                return 'activate';
            case 'unify':
                return 'unify';
            case 'assignment':
                return resolver.label.val;
            default:
                switch (resolver.label.rho) {
                    case 'b0':
                    case '\u00ACb0':
                        return resolver.label.type;
                    default:
                        return resolver.label.rho.replace('b', '\u03C1') + ' ' + resolver.label.type;
                }
        }
    switch (resolver.label.rho) {
        case 'b0':
            return '\u22A4';
        case '\u00ACb0':
            return '\u22A5';
        default:
            return resolver.label.rho.replace('b', '\u03C1');
    }
}

function resolver_tooltip(resolver) {
    switch (resolver.label.rho) {
        case 'b0':
        case '\u00ACb0':
            return 'cost: ' + (resolver.intrinsic_cost.num / resolver.intrinsic_cost.den);
        default:
            return resolver.label.rho.replace('b', '\u03C1') + ', cost: ' + (resolver.intrinsic_cost.num / resolver.intrinsic_cost.den);
    }
}