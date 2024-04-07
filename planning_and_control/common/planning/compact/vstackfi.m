function [A] = vstackfi(n, fi)
    rows = cat(2, arrayfun(fi, 1:n, 'UniformOutput', false));
    A = vertcat(rows{:});
end