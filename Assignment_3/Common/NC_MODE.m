classdef NC_MODE < Simulink.IntEnumType
    % Enumeration type for shaft input selection
    enumeration
       CONTROLLER(1),
       CONSTANT(2),
       STEP(3)
    end
end

