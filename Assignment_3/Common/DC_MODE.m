classdef DC_MODE < Simulink.IntEnumType
    % Enumeration type for rudder input selection
    enumeration
       CONTROLLER(1),
       CONSTANT(2),
       ZERO(3)
    end
end

