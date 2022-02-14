const sc = chroma.scale(['#90EE90', 'yellow', 'red']).domain([0, 10]);

let core_ws, solver_ws, executor_ws;

setup_ws();

const nodes = new vis.DataSet([]);
const edges = new vis.DataSet([]);

const data = {
    nodes: nodes,
    edges: edges
};
const options = {};

const network = new vis.Network(document.getElementById('graph'), data, options);

function setup_ws() {
    solver_ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    solver_ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'started_solving':
                console.log('solving the problem..');
                break;
            case 'solution_found':
                console.log('hurray!! we have found a solution..');
                break;
            case 'inconsistent_problem':
                console.log('the problem has no solution..');
                break;
            case 'flaw_created':
                c_msg['cost'] = { num: 1, den: 0 };
                const flaw = {
                    id: c_msg['id'],
                    label: flaw_label(c_msg),
                    title: flaw_tooltip(c_msg),
                    data: c_msg
                };
                nodes.add(flaw);
                const causes = [];
                for (const c of c_msg['causes'])
                    causes.push({
                        from: c_msg['id'],
                        to: c,
                        arrows: { to: true }
                    });
                edges.add(causes);
                break;
            case 'resolver_created':
                const resolver = {
                    id: c_msg['id'],
                    label: resolver_label(c_msg),
                    title: resolver_tooltip(c_msg),
                    shape: 'box',
                    data: c_msg
                };
                nodes.add(resolver);
                const effect = {
                    from: c_msg['id'],
                    to: c_msg['effect'],
                    arrows: { to: true }
                };
                edges.add(effect);
                break;
        }
    };
    solver_ws.onclose = () => setTimeout(setup_ws, 1000);
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