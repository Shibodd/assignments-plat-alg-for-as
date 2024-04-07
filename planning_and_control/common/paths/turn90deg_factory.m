function rpath = turn90deg_factory(straight_len, radius)
    rpath = MultiReferencePath({
        LineReferencePath([0; 0], [1; 0]);
        CircleReferencePath([straight_len; radius], radius, -pi/2);
        LineReferencePath([radius + straight_len; radius], [0;1])
    }, [straight_len, radius * pi/2]);
end

