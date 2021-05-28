export class TimelinesData {

    constructor() {
        this.timelines = [];
        this.horizon = 1;
    }

    reset(tls) {
        if (tls.timelines) {
            this.timelines.splice(0, this.timelines.length - tls.timelines.length);
            tls.timelines.forEach((tl, i) => {
                this.timelines[i] = tl;
                this.timelines[i].id = i;
                this.timelines[i].values.forEach((v, j) => v.id = j);
                if (this.timelines[i].type === 'ReusableResource' && this.timelines[i].values.length) this.timelines[i].values.push({ usage: 0, from: this.timelines[i].values[this.timelines[i].values.length - 1].to, id: this.timelines[i].values.length });
                if (this.timelines[i].type === 'Agent') {
                    const ends = [0];
                    this.timelines[i].values.forEach(v => v.y = values_y(v.from, v.from === v.to ? v.from + 0.1 : v.to, ends));
                }
            });
        }
        if (this.timelines.length)
            this.horizon = Math.max(d3.max(this.timelines, d => d.horizon), 1);
    }

    tick(time) {
        this.current_time = time.current_time.num / time.current_time.den;
    }

    starting_atoms(atoms) {
    }

    ending_atoms(atoms) {
    }
}

export class Timelines {

    constructor(timelines_id = 'timelines', width = window.innerWidth, height = window.innerHeight) {
        const svg = d3.select('#' + timelines_id).append('svg').attr('viewBox', '0 0 ' + width + ' ' + height);

        this.sv_ok_lg = svg.append('defs').append('linearGradient').attr('id', 'sv-ok-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        this.sv_ok_lg.append('stop').attr('offset', '0%').style('stop-color', 'palegreen').style('stop-opacity', 1);
        this.sv_ok_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        this.sv_ok_lg.append('stop').attr('offset', '100%').style('stop-color', 'palegreen').style('stop-opacity', 1);

        this.sv_inc_lg = svg.append('defs').append('linearGradient').attr('id', 'sv-inc-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        this.sv_inc_lg.append('stop').attr('offset', '0%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);
        this.sv_inc_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        this.sv_inc_lg.append('stop').attr('offset', '100%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);

        this.sv_none_lg = svg.append('defs').append('linearGradient').attr('id', 'sv-none-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        this.sv_none_lg.append('stop').attr('offset', '0%').style('stop-color', 'lightgray').style('stop-opacity', 1);
        this.sv_none_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        this.sv_none_lg.append('stop').attr('offset', '100%').style('stop-color', 'lightgray').style('stop-opacity', 1);

        this.ag_lg = svg.append('defs').append('linearGradient').attr('id', 'ag-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        this.ag_lg.append('stop').attr('offset', '0%').style('stop-color', 'navajowhite').style('stop-opacity', 1);
        this.ag_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        this.ag_lg.append('stop').attr('offset', '100%').style('stop-color', 'navajowhite').style('stop-opacity', 1);

        this.timelines_g = svg.append('g');

        this.timelines_height = height;

        this.timelines_x_scale = d3.scaleLinear().range([0, width]);
        this.timelines_y_scale = d3.scaleBand().rangeRound([0, this.timelines_height]).padding(0.1);

        this.timelines_axis_g = svg.append('g');
        this.timelines_x_axis = d3.axisBottom(this.timelines_x_scale);

        const timelines_zoom = d3.zoom().on('zoom', event => {
            this.timelines_axis_g.call(this.timelines_x_axis.scale(event.transform.rescaleX(this.timelines_x_scale)));
            this.timelines_g.attr('transform', event.transform);
        });
        svg.call(timelines_zoom);

        this.tooltip = d3.select('.tooltip');

        this.tl_name = timeline_name;
        this.sv_val_name = sv_value_name;
        this.sv_val_tooltip = sv_value_tooltip;
        this.rr_val_tooltip = rr_value_tooltip;
        this.ag_val_name = ag_value_name;
        this.ag_val_tooltip = ag_value_tooltip;
    }

    update(data) {
        this.timelines_x_scale.domain([0, data.horizon]);
        this.timelines_axis_g.call(this.timelines_x_axis);
        this.timelines_y_scale.domain(d3.range(data.timelines.length));

        this.timelines_g.selectAll('g.timeline').data(data.timelines, d => d.id).join(
            enter => {
                const tl_g = enter.append('g').attr('class', 'timeline').attr('id', d => 'tl-' + d.id);
                tl_g.append('rect').attr('x', -10).attr('y', d => this.timelines_y_scale(data.timelines.indexOf(d))).attr('width', this.timelines_x_scale(data.horizon) + 20).attr('height', this.timelines_y_scale.bandwidth()).style('fill', 'floralwhite');
                tl_g.append('text').attr('x', 0).attr('y', d => this.timelines_y_scale(data.timelines.indexOf(d)) + this.timelines_y_scale.bandwidth() * 0.08).text(d => this.tl_name(d)).style('text-anchor', 'start');
                return tl_g;
            },
            update => {
                update.select('rect').transition().duration(200).attr('width', this.timelines_x_scale(data.horizon) + 20);
                update.select('text').text(d => this.tl_name(d));
                return update;
            });
        data.timelines.forEach((tl, i) => this.updateTimeline(data, tl, i));
    }

    updateTimeline(data, tl, i) {
        switch (tl.type) {
            case 'StateVariable':
                d3.select('#tl-' + i).selectAll('g').data(tl.values, d => d.id).join(
                    enter => {
                        const tl_val_g = enter.append('g');
                        tl_val_g.append('rect').attr('x', d => this.timelines_x_scale(d.from)).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.1).attr('width', d => this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)).attr('height', this.timelines_y_scale.bandwidth() * 0.9).attr('rx', 5).attr('ry', 5).style('fill', d => sv_value_fill(d)).style('stroke', 'lightgray');
                        tl_val_g.append('text').attr('x', d => this.timelines_x_scale(d.from) + (this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)) / 2).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.5).text(d => this.sv_val_name(d)).style('text-anchor', 'middle').style("font-size", "1px").each(getFontSize).style("font-size", function (d) { return d.scale + "px"; });
                        tl_val_g.on('mouseover', (event, d) => this.tooltip.html(this.sv_val_tooltip(d)).transition().duration(200).style('opacity', .9))
                            .on('mousemove', event => this.tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                            .on('mouseout', event => this.tooltip.transition().duration(500).style('opacity', 0));
                        return tl_val_g;
                    },
                    update => {
                        update.select('rect').transition().duration(200).attr('x', d => this.timelines_x_scale(d.from)).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.1).attr('width', d => this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)).attr('height', this.timelines_y_scale.bandwidth() * 0.9).style('fill', d => sv_value_fill(d));
                        update.select('text').text(d => this.sv_val_name(d)).style("font-size", "1px").each(getFontSize).style("font-size", function (d) { return d.scale + "px"; }).transition().duration(200).attr('x', d => this.timelines_x_scale(d.from) + (this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)) / 2).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.5);
                        return update;
                    }
                );
                break;
            case 'ReusableResource':
                const rr_max_val = (tl.values.length ? Math.max(d3.max(tl.values, d => d.usage), tl.capacity) : tl.capacity);
                const rr_y_scale = d3.scaleLinear().domain([0, rr_max_val + rr_max_val * 0.1]).range([this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth(), this.timelines_y_scale(i)]);
                const rr_g = d3.select('#tl-' + i);
                rr_g.selectAll('path').data([tl.values]).join(
                    enter => {
                        const tl_val_g = enter.append('path').attr('fill', 'aliceblue').attr('stroke', 'lightblue')
                            .attr('d', d3.area().curve(d3.curveStepAfter).x(d => this.timelines_x_scale(d.from)).y0(this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth()).y1(d => rr_y_scale(d.usage)));
                        tl_val_g.on('mouseover', (event, d) => this.tooltip.html(this.rr_val_tooltip(d)).transition().duration(200).style('opacity', .9))
                            .on('mousemove', event => this.tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                            .on('mouseout', event => this.tooltip.transition().duration(500).style('opacity', 0));
                        return tl_val_g;
                    },
                    update => {
                        update.transition().duration(200).attr('d', d3.area().curve(d3.curveStepAfter).x(d => this.timelines_x_scale(d.from)).y0(this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth()).y1(d => rr_y_scale(d.usage)));
                        return update;
                    }
                );
                rr_g.selectAll('line').data([tl.capacity]).join(
                    enter => {
                        const line_g = enter.append('line').attr('stroke-width', 2).attr('stroke-opacity', 0.8).attr('stroke-linecap', 'round').attr('stroke', 'darkslategray');
                        line_g.attr('x1', this.timelines_x_scale(0)).attr('y1', d => rr_y_scale(d)).attr('x2', this.timelines_x_scale(data.horizon)).attr('y2', d => rr_y_scale(d));
                        return line_g;
                    },
                    update => {
                        update.transition().duration(200).attr('y1', d => rr_y_scale(d)).attr('x2', this.timelines_x_scale(data.horizon)).attr('y2', d => rr_y_scale(d));
                        return update;
                    }
                );
                break;
            case 'Agent':
                const max_overlap = d3.max(tl.values, d => d.y) + 1;
                const agent_y_scale = d3.scaleBand().domain(d3.range(max_overlap)).rangeRound([0, this.timelines_y_scale.bandwidth() * 0.9]).padding(0.1);
                d3.select('#tl-' + i).selectAll('g').data(tl.values, d => d.id).join(
                    enter => {
                        const tl_val_g = enter.append('g');
                        tl_val_g.append('rect').attr('x', d => this.timelines_x_scale(d.from)).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.1 + agent_y_scale(max_overlap - 1 - d.y)).attr('width', d => d.from === d.to ? 1 : this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)).attr('height', agent_y_scale.bandwidth()).attr('rx', 5).attr('ry', 5).style('fill', 'url(#ag-lg)').style('stroke', 'lightgray');
                        tl_val_g.append('text').attr('x', d => this.timelines_x_scale(d.from) + (d.from === d.to ? 1 : this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)) / 2).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.1 + agent_y_scale(max_overlap - 1 - d.y) + agent_y_scale.bandwidth() / 2).text(d => this.ag_val_name(d)).style('text-anchor', 'middle').style("font-size", "1px").each(getFontSize).style("font-size", function (d) { return d.scale + "px"; });
                        tl_val_g.on('mouseover', (event, d) => this.tooltip.html(this.ag_val_tooltip(d)).transition().duration(200).style('opacity', .9))
                            .on('mousemove', event => this.tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                            .on('mouseout', event => this.tooltip.transition().duration(500).style('opacity', 0));
                        return tl_val_g;
                    },
                    update => {
                        update.select('rect').transition().duration(200).attr('x', d => this.timelines_x_scale(d.from)).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.1 + agent_y_scale(max_overlap - 1 - d.y)).attr('width', d => d.from === d.to ? 1 : this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)).attr('height', agent_y_scale.bandwidth() * 0.9);
                        update.select('text').text(d => this.ag_val_name(d)).style("font-size", "1px").each(getFontSize).style("font-size", function (d) { return d.scale + "px"; }).transition().duration(200).attr('x', d => this.timelines_x_scale(d.from) + (d.from === d.to ? 1 : this.timelines_x_scale(d.to) - this.timelines_x_scale(d.from)) / 2).attr('y', d => this.timelines_y_scale(i) + this.timelines_y_scale.bandwidth() * 0.1 + agent_y_scale(max_overlap - 1 - d.y) + agent_y_scale.bandwidth() / 2);
                        return update;
                    }
                );
                break;
            default:
                break;
        }
    }

    updateTime(data) {
        if (data.timelines.length)
            this.timelines_g.selectAll('g.time').data([data.current_time]).join(
                enter => {
                    const t_g = enter.append('g').attr('class', 'time');
                    t_g.append('line').attr('stroke-width', 2).attr('stroke-linecap', 'round').attr('stroke', 'lavender').attr('stroke-opacity', 0.4).attr('x1', this.timelines_x_scale(data.current_time)).attr('y1', 0).attr('x2', this.timelines_x_scale(data.current_time)).attr('y2', this.timelines_height);
                    t_g.append('line').attr('stroke-width', 0.2).attr('stroke-linecap', 'round').attr('stroke', 'black').attr('x1', this.timelines_x_scale(data.current_time)).attr('y1', 0).attr('x2', this.timelines_x_scale(data.current_time)).attr('y2', this.timelines_height);
                    return t_g;
                },
                update => {
                    update.selectAll('line').transition().duration(200).attr('x1', this.timelines_x_scale(data.current_time)).attr('x2', this.timelines_x_scale(data.current_time));
                    return update;
                });
    }

    set_timeline_name(tl_name) { this.tl_name = tl_name; }

    set_state_variable_value_name(sv_val_name) { this.sv_val_name = sv_val_name; }
    set_state_variable_value_tooltip(sv_val_tooltip) { this.sv_val_tooltip = sv_val_tooltip; }

    set_reusable_resource_value_tooltip(rr_val_tooltip) { this.rr_val_tooltip = rr_val_tooltip; }

    set_agent_value_name(ag_val_name) { this.ag_val_name = ag_val_name; }
    set_agent_value_tooltip(ag_val_tooltip) { this.ag_val_tooltip = ag_val_tooltip; }
}

function timeline_name(tl) { return tl.name; }

function sv_value_fill(sv_value) {
    switch (sv_value.atoms.length) {
        case 0: return 'url(#sv-none-lg)';
        case 1: return 'url(#sv-ok-lg)';
        default: return 'url(#sv-inc-lg)';
    }
}

function sv_value_name(sv_value) {
    switch (sv_value.atoms.length) {
        case 0: return '';
        case 1: return sv_value.atoms[0].predicate;
        default: return Array.from(sv_value.atoms, atm => atm.predicate).join();
    }
}

function sv_value_tooltip(sv_value) {
    switch (sv_value.atoms.length) {
        case 0: return 'none';
        case 1: return atom_to_string(sv_value.atoms[0]);
        default: return Array.from(sv_value.atoms, atm => atom_to_string(atm)).join();
    }
}

function rr_value_tooltip(rr_value) { return rr_value.usage; }

function ag_value_name(ag_value) { return ag_value.atom.predicate; }

function ag_value_tooltip(ag_value) { return atom_to_string(ag_value.atom); }

function values_y(start, end, ends) {
    for (let i = 0; i < ends.length; i++)
        if (ends[i] <= start) {
            ends[i] = end;
            return i;
        }
    ends.push(end);
    return ends.length - 1;
}

function atom_to_string(atom) {
    let txt = '\u03C3' + atom.sigma + ' ' + atom.predicate;
    const xprs = [];
    Object.keys(atom).filter(n => n != 'sigma' && n != 'predicate').sort().forEach(xpr => xprs.push(xpr + ':' + atom[xpr]));
    return txt + '(' + xprs.join(', ') + ')';
}

function getFontSize(d) {
    const bbox = this.getBBox();
    const cbbox = this.parentNode.getBBox();
    const scale = Math.min(cbbox.width / bbox.width, cbbox.height / bbox.height, 15);
    d.scale = scale;
}