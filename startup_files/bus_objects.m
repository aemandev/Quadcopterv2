% Bus object: Force
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = 1;
elems(1).DataType = 'single';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).DocUnits = sprintf('N');
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y';
elems(2).Dimensions = 1;
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).DocUnits = sprintf('N');
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z';
elems(3).Dimensions = 1;
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).DocUnits = sprintf('N');
elems(3).Description = '';

Force = Simulink.Bus;
Force.Description = '';
Force.DataScope = 'Auto';
Force.Alignment = -1;
Force.Elements = elems;
clear elems;
assignin('base','Force', Force);