classdef NullPlanner < Planner
    properties
        ReferencePath
    end
    methods
        function obj = NullPlanner(path)
            obj.ReferencePath = path;
        end

        function path = get_path(obj, ~, ~, ~, ~, ~)
            path = obj.ReferencePath;
        end
    end
end