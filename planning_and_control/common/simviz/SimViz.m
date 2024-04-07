classdef SimViz < handle
    properties (Access = private)
        Signals cell = {}
    end
    
    methods
        function obj = addSignal(obj, ts, xs, ys)
            arguments
                obj SimViz
                ts (1,:)
                xs (1,:)
                ys (1,:)
            end
            obj.Signals{end+1} = SimVizSignal(ts, xs, ys);
        end

        function hndl = getLastScatter(obj)
            hndl = obj.Signals(end).ScatterHandle;
        end

        function hndl = getLastAnimatedLine(obj)
            hndl = obj.Signals(end).AnimatedLineHandle;
        end

        function [] = run(obj, speed)
            arguments
                obj SimViz
                speed double = 1
            end
            
            % Sample index of each signal
            i = ones(numel(obj.Signals), 1);
            
            TICKS_PER_SECOND = 1e3;
            start_t = convertTo(datetime, 'epochtime', 'TicksPerSecond', TICKS_PER_SECOND);
            skip = false;
    
            fprintf("Running animated playback at %.1fx speed. Press any key while the figure is focused to skip.\n", speed);
            
            while all(i >= 1)
                % Support keypress to skip to end
                if numel(get(gcf,'CurrentCharacter')) > 0
                    skip = true;
                end
                if skip
                    t = inf;
                else
                    t = speed * double(convertTo(datetime, 'epochtime', 'TicksPerSecond', TICKS_PER_SECOND) - start_t) / TICKS_PER_SECOND;
                end
                
                should_draw = false;

                % For each signal
                for k=1:numel(obj.Signals)
                    sig = obj.Signals{k};

                    % Terminate
                    if i(k) > sig.N
                        i(k) = -1;
                        continue;
                    end

                    % Seek for samples with timestamp <= t
                    new_i = i(k);
                    
                    while new_i <= sig.N && sig.Ts(new_i) <= t
                        new_i = new_i + 1;
                    end

                    % There are new points, add them
                    if new_i > i(k)
                        last_i = min(sig.N, new_i);
                        addpoints(sig.AnimatedLineHandle, sig.Xs(i(k):last_i), sig.Ys(i(k):last_i));
                        
                        sig.ScatterHandle.XData = sig.Xs(last_i);
                        sig.ScatterHandle.YData = sig.Ys(last_i);
                        
                        should_draw = true;
                        i(k) = new_i;
                    end
                end

                if should_draw
                    drawnow;
                end
            end
        end
    end
end

