function optimizer_fn = optimizer_factory(sys, u_max, N)
    Ti = Ti_fn_factory(sys.A, sys.B, N);

    L = zeros(1, 4);
    L(1,1) = 1;
    
    LT = vstackfi(N, @(i) L * Ti(i));
    A = [
        LT;
        -LT;
    ];

    ub = repelem(u_max, N, 1);

    wdev = 0.1 / double(N);
    wact = 5 / double(N);
    wep = 1;

    % wavg = 1 / double(N);
    % window = N / 2;
    % avg_m = [-1, ones(1,window-1), zeros(1,N-window)];
    % AVG = toeplitz(avg_m, avg_m);

    C = [
        wdev* LT;
        wact* eye(N, N);
        wep*  Ti(N);
        % wavg* AVG
    ];
    assignin('base', 'C', C);

    options = optimset("display", "off");
    
    function [u, cost] = optimize(x0, tgt_l, phidotdes, lminmax)
        assert(isequal(size(x0), [4, 1]));
        assert(isequal(size(phidotdes), [1,N]));
        assert(isequal(size(lminmax), [N,2]));
        
        c = c_factory(sys.A, sys.B, x0, phidotdes);

        Lc = (L * c)';
        b = [
            lminmax(:,2) - Lc;
            Lc - lminmax(:,1)
        ];
        d = -[
            wdev* (Lc - tgt_l);
            wact* zeros(N, 1);
            wep*  c(:,N) - ([tgt_l(N);0;0;0]);
            % wavg* zeros(N, 1);
        ];

        [u,cost,~,exitflag,out] = lsqlin(C, d, A, b, [], [], -ub, ub, [], options);
    
        assert(exitflag >= 0 || exitflag == -2, out.message + " (exitflag=" + exitflag + ")");
        
        % Unfeasible problem
        if exitflag == -2
            u = missing;
            cost = NaN;
        end
    end

    optimizer_fn = @optimize;
end

