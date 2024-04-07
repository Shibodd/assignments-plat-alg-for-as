classdef MultiReferencePath < ReferencePath
    properties(Access=private)
        Pieces (:,1) cell
        SwapArcs (:,1) double
        GlobalSwapArcs (:,1) double
    end
    
    methods(Access=protected)
        function [i, piece_s] = piece_by_pos(obj, position)
            % Find i s.t. 0 <= piece_s=Pieces{i).arc(position) <= swap_arcs(i)
            % with minimum Pieces{i).deviation(position)

            N = numel(obj.Pieces);

            ss = vstackfi(N, @(i) obj.Pieces{i}.arc(position));
            idxs = find((ss >= 0) & (ss <= [obj.SwapArcs; Inf]));

            if numel(idxs) <= 0
                disp("ciao");
            end

            assert(numel(idxs) > 0, "FIXME");

            lds = abs(vstackfi(numel(idxs), @(i) obj.Pieces{idxs(i)}.lateral_deviation(position)));
            [~,idxs_i] = min(lds);

            i = idxs(idxs_i);
            piece_s = ss(i);
        end

        function i = piece_by_arc(obj, s)
            i = [];
            for j=1:numel(s)
                k = find(obj.GlobalSwapArcs < s(j), 1, "last") + 1;
                if isempty(k)
                    i = [i 1];
                else
                    i = [i k];
                end
            end
        end

        function x = func_at_arc(obj, s, func)
            is = obj.piece_by_arc(s);

            x = [];
            for j=1:numel(s)
                i = is(j);
                s_piece = s(j);

                i = min(numel(obj.Pieces), i);

                if i > 1
                    s_piece = s_piece - obj.GlobalSwapArcs(i-1);
                end
                x = [x, func(obj.Pieces{i}, s_piece)];
            end
        end
    end
    
    methods
        function obj = MultiReferencePath(pieces, swap_arcs)
            arguments
                pieces (1,:) cell
                swap_arcs (1,:) double
            end

            assert(numel(pieces) == (numel(swap_arcs) + 1));

            obj.Pieces = pieces;
            obj.SwapArcs = swap_arcs;
            obj.GlobalSwapArcs = cumsum(swap_arcs);
        end

        function s = arc(obj, position)
            s = [];
            for j=1:size(position,2)
                [i, piece_s] = obj.piece_by_pos(position);
                s = [s, piece_s];
                if i > 1
                    s = s + obj.GlobalSwapArcs(i - 1);
                end
            end
        end

        function z = deviation(obj, position, heading)
            [i, ~] = obj.piece_by_pos(position);
            z = obj.Pieces{i}.deviation(position, heading);
        end

        function position = position(obj, s)
            position = obj.func_at_arc(s, @position);
        end

        function k = curvature(obj, s)
            k = obj.func_at_arc(s, @curvature);
        end

        function theta = angle(obj, s)
            theta = obj.func_at_arc(s, @angle);
        end

        function normal = normal(obj, s)
            normal = obj.func_at_arc(s, @normal);
        end
    end
end

