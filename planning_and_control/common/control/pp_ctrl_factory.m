function ctrl_fn = pp_ctrl_factory(vp, ctrl_interval)
    sys = bicycle_lat_roadalign_factory(vp, ctrl_interval, 70 / 3.6);
    % Target value is assumed to be zero vector => Kr is null
    
    % At any speed, eigenvalues are either 0 or 1. Intuitively, without 
    % control the car is stable in the sense that it keeps moving on 
    % its path, but it isn't attracted by the reference path

    poles = eig(sys.A); % 1 and 4 are the dominant eigenvalues
    poles(1) = 0.94;
    poles(4) = 0.93;
    K = place(sys.A, sys.B(:, 1), poles);

    ctrl_fn = cl_ctrl_factory(K);
end