% Allows logging uniformly-sized vectors through the log method.
% The column-wise series can be retrieved with the retrieve method.

% Throttling is applied on the first vector row.
% Batch allocation is used for performance.

classdef Logger < handle
    properties(Access = private)
        N int64 = 0
        Data = []
    end
    properties
        Step double
        BatchSize int64 = 8192
    end
    
    methods
        function obj = Logger(step)
            arguments
                step = 0.01;
            end
            obj.Step = step;
        end

        function obj = log(obj, x)
            % If some data was already logged
            if obj.N > 0
                % Throttling on first row
                if x(1) <= obj.Data(1, obj.N) + obj.Step
                    return;
                end

                % Batch allocation
                if size(obj.Data, 2) <= obj.N
                   obj.Data = [obj.Data, zeros(size(obj.Data, 1), obj.BatchSize)];
                end
            end

            obj.N = obj.N + 1;

            % MATLAB stores in column-major order
            obj.Data(:, obj.N) = x;
        end

        function data = retrieve(obj)
            data = obj.Data(:, 1:obj.N);
        end
    end
end

