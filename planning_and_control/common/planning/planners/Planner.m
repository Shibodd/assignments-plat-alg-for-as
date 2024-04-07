classdef Planner < handle
    methods (Abstract)
        path = get_path(obj, position, x0, sys, vx, tgt_ld)
    end
end

