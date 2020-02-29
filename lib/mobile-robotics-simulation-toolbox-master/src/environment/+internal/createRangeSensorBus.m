function createRangeSensorBus
% Creates a range sensor bus definition whose maximum size is dictated by
% the range sensor with the most scans
%
% Copyright 2019 The MathWorks, Inc.

% Get the maximum number of scans
maxNumScans = internal.getMaxNumScans;
% Bus object: slbus_MultiRobotRangeSensor 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'hasLidar';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'boolean';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'ranges';
elems(2).Dimensions = maxNumScans;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'scanAngles';
elems(3).Dimensions = maxNumScans;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'sensorOffset';
elems(4).Dimensions = 2;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'numScans';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

slbus_MultiRobotRangeSensor = Simulink.Bus;
slbus_MultiRobotRangeSensor.HeaderFile = '';
slbus_MultiRobotRangeSensor.Description = '';
slbus_MultiRobotRangeSensor.DataScope = 'Auto';
slbus_MultiRobotRangeSensor.Alignment = -1;
slbus_MultiRobotRangeSensor.Elements = elems;
clear elems;
assignin('base','slbus_MultiRobotRangeSensor', slbus_MultiRobotRangeSensor);