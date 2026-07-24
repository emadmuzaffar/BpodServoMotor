function distance = getDistance(motorInstruction)
global ServoMotorStruct

speed = motorInstruction{1};
time = motorInstruction{2};
direction = motorInstruction{3};

maximumDrivenDegreesPerSecond = ServoMotorStruct.maxDrivenRPM * 6;
validateattributes(speed, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonnegative', ...
        '<=', maximumDrivenDegreesPerSecond}, ...
    mfilename, 'speed');
validateattributes(time, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonnegative', '<=', double(intmax('uint16'))}, ...
    mfilename, 'time');
validateattributes(direction, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonnegative', '<=', 1}, ...
    mfilename, 'direction');

distance = speed * (time / (1000 / ServoMotorStruct.timeMultiplier));

if direction == 1    
    distance = -distance; 
end

end
