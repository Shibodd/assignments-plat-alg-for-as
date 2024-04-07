classdef LongitudinalPIController < handle
    properties
        Kp
        Ki
        TargetVelocityFn
        LogFn
    end

    properties(Access=private)
        Integral = 0
    end

    methods
        function [u] = tick(obj, t, interval, simulation_x)
            % Assume we're measuring the velocity without noise
            tgt_vel = obj.TargetVelocityFn(t, simulation_x);
            err = tgt_vel - simulation_x(2);

            obj.Integral = obj.Integral + obj.Ki * err * interval;
            
            u = obj.Kp * err + obj.Integral;
            l = [
                t;
                err;
                tgt_vel;
                u;
                simulation_x;
                obj.Integral
           ];
           obj.LogFn(l);
        end
    end
end