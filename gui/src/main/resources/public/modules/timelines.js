export class Timelines {

    constructor(svg, tooltip) {
        this.timelines = [];

        this.sv_ok_lg = svg.append('defs').append('linearGradient').attr('id', 'sv-ok-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        sv_ok_lg.append('stop').attr('offset', '0%').style('stop-color', 'palegreen').style('stop-opacity', 1);
        sv_ok_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        sv_ok_lg.append('stop').attr('offset', '100%').style('stop-color', 'palegreen').style('stop-opacity', 1);

        this.sv_inc_lg = svg.append('defs').append('linearGradient').attr('id', 'sv-inc-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        sv_inc_lg.append('stop').attr('offset', '0%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);
        sv_inc_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        sv_inc_lg.append('stop').attr('offset', '100%').style('stop-color', 'lightsalmon').style('stop-opacity', 1);

        this.sv_none_lg = svg.append('defs').append('linearGradient').attr('id', 'sv-none-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        sv_none_lg.append('stop').attr('offset', '0%').style('stop-color', 'lightgray').style('stop-opacity', 1);
        sv_none_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        sv_none_lg.append('stop').attr('offset', '100%').style('stop-color', 'lightgray').style('stop-opacity', 1);

        this.ag_lg = svg.append('defs').append('linearGradient').attr('id', 'ag-lg').attr('x1', '0%').attr('x2', '0%').attr('y1', '0%').attr('y2', '100%');
        ag_lg.append('stop').attr('offset', '0%').style('stop-color', 'navajowhite').style('stop-opacity', 1);
        ag_lg.append('stop').attr('offset', '20%').style('stop-color', 'ivory').style('stop-opacity', 1);
        ag_lg.append('stop').attr('offset', '100%').style('stop-color', 'navajowhite').style('stop-opacity', 1);

        this.timelines_g = svg.append('g');

        const timelines_box = svg.node().getBoundingClientRect();
        const timelines_width = timelines_box.width, timelines_height = timelines_box.height;

        this.timelines_x_scale = d3.scaleLinear().range([0, timelines_width]);
        this.timelines_y_scale = d3.scaleBand().rangeRound([0, timelines_height]).padding(0.1);

        this.timelines_axis_g = svg.append('g');
        this.timelines_x_axis = d3.axisBottom(timelines_x_scale);

        const timelines_zoom = d3.zoom().on('zoom', event => {
            timelines_axis_g.call(timelines_x_axis.scale(event.transform.rescaleX(timelines_x_scale)));
            timelines_g.attr('transform', event.transform);
        });
        svg.call(timelines_zoom);

        this.tooltip = tooltip;
    }

    reset(tls) {
        timelines.splice(0, timelines.length - tls.timelines.length);
        tls.timelines.forEach((tl, i) => {
            console.log(tl);
            timelines[i] = tl;
            timelines[i].id = i;
            timelines[i].values.forEach((v, j) => v.id = j);
            if (timelines[i].type === 'reusable-resource' && timelines[i].values.length) timelines[i].values.push({ usage: 0, from: timelines[i].values[timelines[i].values.length - 1].to, id: timelines[i].values.length });
            if (timelines[i].type === 'agent') {
                const ends = [0];
                timelines[i].values.forEach(v => v.y = values_y(v.from, v.from === v.to ? v.from + 0.1 : v.to, ends));
            }
        });
        if (timelines.length) {
            horizon = Math.max(d3.max(timelines, d => d.horizon), 1);
            timelines_x_scale.domain([0, horizon]);
            timelines_y_scale.domain(d3.range(timelines.length));
            updateTimelines();
        }
        timelines_axis_g.call(timelines_x_axis);
    }

    tick(time) {
        current_time = time;
        if (timelines.length)
            updateTime();
    }

    starting_atoms(atoms) {
        console.log('starting atoms: ' + atoms.atoms);
    }

    ending_atoms(atoms) {
        console.log('ending atoms: ' + atoms.atoms);
    }

    update() {
        timelines_g.selectAll('g.timeline').data(timelines, d => d.id).join(
            enter => {
                const tl_g = enter.append('g').attr('class', 'timeline').attr('id', d => 'tl-' + d.id);
                tl_g.append('rect').attr('x', -10).attr('y', d => timelines_y_scale(timelines.indexOf(d))).attr('width', timelines_x_scale(horizon) + 20).attr('height', timelines_y_scale.bandwidth()).style('fill', 'floralwhite');
                tl_g.append('text').attr('x', 0).attr('y', d => timelines_y_scale(timelines.indexOf(d)) + timelines_y_scale.bandwidth() * 0.08).text(d => d.name).style('text-anchor', 'start');
                return tl_g;
            },
            update => {
                update.select('rect').transition().duration(200).attr('width', timelines_x_scale(horizon) + 20);
                return update;
            });
        timelines.forEach((tl, i) => updateTimeline(tl, i));
    }

    updateTimeline(tl, i) {
        switch (tl.type) {
            case 'state-variable':
                d3.select('#tl-' + i).selectAll('g').data(tl.values, d => d.id).join(
                    enter => {
                        const tl_val_g = enter.append('g');
                        tl_val_g.append('rect').attr('x', d => timelines_x_scale(d.from)).attr('y', d => timelines_y_scale(i) + timelines_y_scale.bandwidth() * 0.1).attr('width', d => timelines_x_scale(d.to) - timelines_x_scale(d.from)).attr('height', timelines_y_scale.bandwidth() * 0.9).attr('rx', 5).attr('ry', 5).style('fill', d => sv_value_fill(d)).style('stroke', 'lightgray');
                        tl_val_g.on('mouseover', (event, d) => tooltip.html(sv_value_tooltip(d)).transition().duration(200).style('opacity', .9))
                            .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                            .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0));
                        return tl_val_g;
                    },
                    update => {
                        update.select('rect').transition().duration(200).attr('x', d => timelines_x_scale(d.from)).attr('y', d => timelines_y_scale(i) + timelines_y_scale.bandwidth() * 0.1).attr('width', d => timelines_x_scale(d.to) - timelines_x_scale(d.from)).attr('height', timelines_y_scale.bandwidth() * 0.9).style('fill', d => sv_value_fill(d));
                        return update;
                    }
                );
                break;
            case 'reusable-resource':
                const rr_max_val = (tl.values.length ? Math.max(d3.max(tl.values, d => d.usage), tl.capacity) : tl.capacity);
                const rr_y_scale = d3.scaleLinear().domain([0, rr_max_val + rr_max_val * 0.1]).range([timelines_y_scale(i) + timelines_y_scale.bandwidth(), timelines_y_scale(i)]);
                const rr_g = d3.select('#tl-' + i);
                rr_g.selectAll('path').data([tl.values]).join(
                    enter => {
                        const tl_val_g = enter.append('path').attr('fill', 'aliceblue').attr('stroke', 'lightblue')
                            .attr('d', d3.area().curve(d3.curveStepAfter).x(d => timelines_x_scale(d.from)).y0(timelines_y_scale(i) + timelines_y_scale.bandwidth()).y1(d => rr_y_scale(d.usage)));
                        tl_val_g.on('mouseover', (event, d) => tooltip.html(rr_value_tooltip(d)).transition().duration(200).style('opacity', .9))
                            .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                            .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0));
                        return tl_val_g;
                    },
                    update => {
                        update.transition().duration(200).attr('d', d3.area().curve(d3.curveStepAfter).x(d => timelines_x_scale(d.from)).y0(timelines_y_scale(i) + timelines_y_scale.bandwidth()).y1(d => rr_y_scale(d.usage)));
                        return update;
                    }
                );
                rr_g.selectAll('line').data([tl.capacity]).join(
                    enter => {
                        const line_g = enter.append('line').attr('stroke-width', 2).attr('stroke-opacity', 0.8).attr('stroke-linecap', 'round').attr('stroke', 'darkslategray');
                        line_g.attr('x1', timelines_x_scale(0)).attr('y1', d => rr_y_scale(d)).attr('x2', timelines_x_scale(horizon)).attr('y2', d => rr_y_scale(d));
                        return line_g;
                    },
                    update => {
                        update.transition().duration(200).attr('y1', d => rr_y_scale(d)).attr('x2', timelines_x_scale(horizon)).attr('y2', d => rr_y_scale(d));
                        return update;
                    }
                );
                break;
            case 'agent':
                const max_overlap = d3.max(tl.values, d => d.y) + 1;
                const agent_y_scale = d3.scaleBand().domain(d3.range(max_overlap)).rangeRound([0, timelines_y_scale.bandwidth() * 0.9]).padding(0.1);
                d3.select('#tl-' + i).selectAll('g').data(tl.values, d => d.id).join(
                    enter => {
                        const tl_val_g = enter.append('g');
                        tl_val_g.append('rect').attr('x', d => timelines_x_scale(d.from)).attr('y', d => timelines_y_scale(i) + timelines_y_scale.bandwidth() * 0.1 + agent_y_scale(max_overlap - 1 - d.y)).attr('width', d => d.from === d.to ? 1 : timelines_x_scale(d.to) - timelines_x_scale(d.from)).attr('height', agent_y_scale.bandwidth()).attr('rx', 5).attr('ry', 5).style('fill', 'url(#ag-lg)').style('stroke', 'lightgray');
                        tl_val_g.on('mouseover', (event, d) => tooltip.html(sv_value_tooltip(d)).transition().duration(200).style('opacity', .9))
                            .on('mousemove', event => tooltip.style('left', (event.pageX) + 'px').style('top', (event.pageY - 28) + 'px'))
                            .on('mouseout', event => tooltip.transition().duration(500).style('opacity', 0));
                        return tl_val_g;
                    },
                    update => {
                        update.select('rect').transition().duration(200).attr('x', d => timelines_x_scale(d.from)).attr('y', d => timelines_y_scale(i) + timelines_y_scale.bandwidth() * 0.1 + agent_y_scale(max_overlap - 1 - d.y)).attr('width', d => d.from === d.to ? 1 : timelines_x_scale(d.to) - timelines_x_scale(d.from)).attr('height', agent_y_scale.bandwidth() * 0.9);
                        return update;
                    }
                );
                break;
            default:
                break;
        }
    }

    updateTime() {
        timelines_g.selectAll('g.time').data([current_time]).join(
            enter => {
                const t_g = enter.append('g').attr('class', 'time');
                t_g.append('line').attr('stroke-width', 2).attr('stroke-linecap', 'round').attr('stroke', 'lavender').attr('stroke-opacity', 0.4).attr('x1', timelines_x_scale(current_time)).attr('y1', 0).attr('x2', timelines_x_scale(current_time)).attr('y2', timelines_height);
                return t_g;
            },
            update => {
                update.select('line').transition().duration(200).attr('x1', timelines_x_scale(current_time)).attr('x2', timelines_x_scale(current_time));
                return update;
            });
    }
}

function sv_value_tooltip(sv_value) { return sv_value.name; }

function rr_value_tooltip(rr_value) { return rr_value.usage; }

function sv_value_fill(sv_value) {
    switch (sv_value.atoms.length) {
        case 0: return 'url(#sv-none-lg)';
        case 1: return 'url(#sv-ok-lg)';
        default: return 'url(#sv-inc-lg)';
    }
}

function values_y(start, end, ends) {
    for (let i = 0; i < ends.length; i++)
        if (ends[i] <= start) {
            ends[i] = end;
            return i;
        }
    ends.push(end);
    return ends.length - 1;
}