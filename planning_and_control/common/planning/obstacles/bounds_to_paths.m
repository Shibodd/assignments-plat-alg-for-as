function [paths] = bounds_to_paths(bounds)
    choices_per_step = cellfun(@(x) size(x, 1), bounds);

    if ~all(choices_per_step > 0)
        warning("Some steps are infeasible!");
        paths = [];
        return;
    end
    assert(all(cellfun(@(x) size(x, 2) == 2, bounds)), "Bounds are badly structured!");

    % Build the NodeID -> Bound LUT (note that step info is lost)
    boundLUT = vertcat(bounds{:});
    
    node_count = size(boundLUT, 1);

    stepidx_to_nodeid = cumsum([0, choices_per_step(1:end-1)]);

    % Build the graph.
    A = sparse(node_count, node_count);
    for step_idx=1:numel(bounds) - 1
        for i=stepidx_to_nodeid(step_idx) + (1:choices_per_step(step_idx))
            for j=stepidx_to_nodeid(step_idx + 1) + (1:choices_per_step(step_idx + 1))
                A(i,j) = simple_overlap_check(boundLUT(i,:), boundLUT(j,:));
            end
        end
    end
    g = digraph(A);

    % Build paths
    paths = cellfun(@(x) boundLUT(x,:), g.allpaths(1, node_count), 'UniformOutput', false);
end

