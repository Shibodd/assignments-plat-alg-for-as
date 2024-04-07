function [xs] = sweep_ranges(ranges, dx)
    % Sweeps a list of ranges to return a vector xs containing the sorted 
    % boundaries of the union of the ranges - meaning the values xi s.t. 
    % if xi-dx is contained in the union of all ranges, xi+dx is not, or viceversa.

    arguments
        ranges (:,2) double
        dx (1,1) double = eps
    end
    
    % Valid ranges assumption
    % assert(all(diff(ranges, [], 2) > 0), "Some ranges have negative length!");

    N = size(ranges, 1);

    % Flatten starts and ends, and mark starts and ends...
    START_MARKER = -1;
    END_MARKER = 1;
    unnorm_xs = [
        ranges(:,1), repelem(START_MARKER, N, 1);
        ranges(:,2), repelem(END_MARKER, N, 1)
    ];
    % Then sort by value.
    unnorm_xs = sortrows(unnorm_xs);

    % The first column of unnorm_xs is now a sorted vector of range
    % boundaries, and its second column is a vector of markers
    % to tell range starts and ends apart.

    xs = [];

    % Sweep unnorm_xs to remove range overlaps and obtain xs.
    started_count = 0; % count how many overlapping ranges we're inside
    for i=1:size(unnorm_xs, 1)
        x = unnorm_xs(i,1);
        marker = unnorm_xs(i,2);

        if marker == START_MARKER
            started_count = started_count + 1; % entered a range

            if started_count == 1
                % We're entering the range union. Store the start.
                xs = [xs, x];
            end
        elseif marker == END_MARKER
            started_count = started_count - 1; % left a range

            % A range cannot end before starting.
            % Otherwise, it has negative length and is invalid.
            assert(started_count >= 0, "Invalid range!");

            if started_count == 0
                % We're leaving the range union. Store the end.
                xs = [xs, x];
            end
        else
            assert(false, "wtf");
        end
    end

    assert(started_count == 0, "wtf");
    % Apply threshold dx
    xs = remove_near_bounds(xs, dx);
end

