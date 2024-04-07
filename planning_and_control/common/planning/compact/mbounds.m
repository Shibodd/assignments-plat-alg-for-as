function [mbounds] = mbounds(varargin)
    [tmin, tmax] = bounds(varargin{:});
    mbounds = [tmin, tmax];
end

