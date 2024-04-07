classdef SimVizSignal < handle
    properties
        ScatterHandle
        AnimatedLineHandle
        Ts (1,:)
        Xs (1,:)
        Ys (1,:)
        N (1,1)
    end
    
    methods
        function obj = SimVizSignal(ts, xs, ys)
            N = numel(ts);
            assert(N == numel(ys) && N == numel(xs), "ts, xs and ys should have the same size.");
            
            obj.ScatterHandle = scatter(xs(1), ys(1), 50);
            hold on;
            obj.AnimatedLineHandle = animatedline(xs(1), ys(1), 'Color', obj.ScatterHandle.CData);
            
            obj.Ts = ts;
            obj.Xs = xs;
            obj.Ys = ys;
            obj.N = N;
        end
    end
end

