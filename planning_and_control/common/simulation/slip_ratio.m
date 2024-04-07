function slip = slip_ratio(v, wheelspeed)
    if v > wheelspeed && abs(v) > eps
        slip = wheelspeed / v - 1;
    elseif v < wheelspeed && abs(wheelspeed) > eps
        slip = 1 - v / wheelspeed;
    else
        slip = 0;
    end
end