% Bus object: DCM
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'DCMbe';
elems(1).Dimensions = [3 3];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

DCMbe = Simulink.Bus;
DCMbe.Description = '';
DCMbe.DataScope = 'Auto';
DCMbe.Alignment = -1;
DCMbe.Elements = elems;
clear elems;
assignin('base','DCMbe', DCMbe);

% Bus object: Force
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

Force = Simulink.Bus;
Force.Description = '';
Force.DataScope = 'Auto';
Force.Alignment = -1;
Force.Elements = elems;
clear elems;
assignin('base','Force', Force);

% Bus object: Quaternion
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'w';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'x';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'y';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'z';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'single';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).DocUnits = '';
elems(4).Description = '';

quat = Simulink.Bus;
quat.Description = '';
quat.DataScope = 'Auto';
quat.Alignment = -1;
quat.Elements = elems;
clear elems;
assignin('base','quat', quat);

% Bus object: Position
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

Position = Simulink.Bus;
Position.Description = '';
Position.DataScope = 'Auto';
Position.Alignment = -1;
Position.Elements = elems;
clear elems;
assignin('base','Position', Position);

% Bus object: Velocity
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

Velocity = Simulink.Bus;
Velocity.Description = '';
Velocity.DataScope = 'Auto';
Velocity.Alignment = -1;
Velocity.Elements = elems;
clear elems;
assignin('base','Velocity', Velocity);

% Bus object: Euler Angles
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'roll';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'pitch';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'yaw';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

euler = Simulink.Bus;
euler.Description = '';
euler.DataScope = 'Auto';
euler.Alignment = -1;
euler.Elements = elems;
clear elems;
assignin('base','euler', euler);

% Bus object: Body Rates
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'p';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'q';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'r';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

body_rates = Simulink.Bus;
body_rates.Description = '';
body_rates.DataScope = 'Auto';
body_rates.Alignment = -1;
body_rates.Elements = elems;
clear elems;
assignin('base','body_rates', body_rates);

% Bus object: LLA
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'lat';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'long';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'altitude';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

LLA = Simulink.Bus;
LLA.Description = '';
LLA.DataScope = 'Auto';
LLA.Alignment = -1;
LLA.Elements = elems;
clear elems;
assignin('base','LLA', LLA);

% Bus object: State Estimates
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'Xe';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Bus: Position';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'LLA';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'Bus: LLA';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Ve';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Bus: Velocity';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Vb';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'Bus: Velocity';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'euler';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'Bus: euler';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'wb';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'Bus: body_rates';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'quatbe';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'Bus: quat';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).DocUnits = '';
elems(7).Description = '';

state_estimates = Simulink.Bus;
state_estimates.Description = '';
state_estimates.DataScope = 'Auto';
state_estimates.Alignment = -1;
state_estimates.Elements = elems;
clear elems;
assignin('base','state_estimates', state_estimates);

% Bus object: Reference Signals
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'Xe';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'Bus: Position';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Ve';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'Bus: Velocity';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Vb';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'Bus: Velocity';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'euler';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'Bus: euler';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'wb';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'Bus: body_rates';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).DocUnits = '';
elems(5).Description = '';

reference_signals = Simulink.Bus;
reference_signals.Description = '';
reference_signals.DataScope = 'Auto';
reference_signals.Alignment = -1;
reference_signals.Elements = elems;
clear elems;
assignin('base','reference_signals', reference_signals);


