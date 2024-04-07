classdef VehicleParameters
    properties
        M double = 2164 % Kg
        I double = 4373 % Kg * m^2
        Lf double = 1.3384 % m
        Lr double = 1.6456 % m
        Cf double = 1.0745 * 10^5
        Cr double = 1.9032 * 10^5

        WheelRadius double = 0.681 / 2 % [m]
        WheelInertia double = 0.8 % [kg*m^2]
        CdA double = 0.56 % [m^2] (Coeff. Drag * Area)
        Croll double = 0.0065 % (Rolling Resistance Coeff.)
        Clong double = 10000 % [N]

        MaxSteer double = deg2rad(30) % [rad]
    end
end