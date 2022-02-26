document.querySelector('#tick').addEventListener('click', () => ws.send('tick'));

const reasoner = new Reasoner(document.getElementById('timelines'), document.getElementById('graph'));

let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/solver');
    ws.onmessage = msg => {
        const c_msg = JSON.parse(msg.data);
        switch (c_msg.type) {
            case 'state_changed':
                reasoner.state_changed(c_msg);
                break;
            case 'started_solving':
                console.log('solving the problem..');
                break;
            case 'solution_found':
                console.log('hurray!! we have found a solution..');
                reasoner.solution_found();
                break;
            case 'inconsistent_problem':
                console.log('the problem has no solution..');
                reasoner.inconsistent_problem();
                break;
            case 'graph':
                reasoner.graph(c_msg);
                break;
            case 'flaw_created':
                reasoner.flaw_created(c_msg);
                break;
            case 'flaw_state_changed':
                reasoner.flaw_state_changed(c_msg);
                break;
            case 'flaw_cost_changed':
                reasoner.flaw_cost_changed(c_msg);
                break;
            case 'flaw_position_changed':
                reasoner.flaw_position_changed(c_msg);
                break;
            case 'current_flaw':
                reasoner.current_flaw_changed(c_msg);
                break;
            case 'resolver_created':
                reasoner.resolver_created(c_msg);
                break;
            case 'resolver_state_changed':
                reasoner.resolver_state_changed(c_msg);
                break;
            case 'current_resolver':
                reasoner.current_resolver_changed(c_msg);
                break;
            case 'causal_link_added':
                reasoner.causal_link_added(c_msg);
                break;
            case 'tick':
                reasoner.tick(c_msg);
                break;
            case 'starting':
                reasoner.starting(c_msg);
                break;
            case 'ending':
                reasoner.ending(c_msg);
                break;
            case 'start':
                reasoner.start(c_msg);
                break;
            case 'end':
                reasoner.end(c_msg);
                break;
        }
    };
    ws.onclose = () => setTimeout(setup_ws, 1000);
}