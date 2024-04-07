function ctrl_fn = lqr_ctrl_factory(vp, Q, R, ctrl_interval)
    sys = bicycle_lat_roadalign_factory(vp, ctrl_interval, 70 / 3.6);
    K = dlqr(sys.A, sys.B(:,1), Q, R);
    ctrl_fn = cl_ctrl_factory(K);
end

