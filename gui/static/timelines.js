const items = new Map(), atoms = new Map();

const timelines = new vis.DataSet([]);
const timeline_values = new vis.DataSet([]);

const options = {};

const timeline = new vis.Timeline(document.getElementById('timelines'), timeline_values, timelines, options);
timeline.currentTime.stop();

let current_time = 0;
timeline.setCurrentTime(current_time);

let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/ws-timelines');
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
                break;
            case 'inconsistent_problem':
                console.log('the problem has no solution..');
                break;
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