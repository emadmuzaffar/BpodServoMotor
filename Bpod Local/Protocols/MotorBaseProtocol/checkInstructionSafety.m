
function instructedRotation = checkInstructionSafety(motorInstruction, startingRotation)

    global ServoMotorStruct

    if ~iscell(motorInstruction) || numel(motorInstruction) ~= 3
        error('motorInstruction must be a 3-element cell array: {speed, time, direction}.')
    end

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

    if nargin < 2
        startingRotation = ServoMotorStruct.instructedRotation;
        commitResult = true;
    else
        validateattributes(startingRotation, {'numeric'}, ...
            {'scalar', 'real', 'finite'}, mfilename, 'startingRotation');
        commitResult = false;
    end

    distance = speed * (time / (1000 / ServoMotorStruct.timeMultiplier));
    
    if direction == 1
        distance = -distance;
    end

    instructedRotation = startingRotation + distance;

    if instructedRotation > ServoMotorStruct.maxRotation
        error("Instructed rotation exceeded max rotation. Instructed value when thrown = " + string(instructedRotation) + ". Max = " + string(ServoMotorStruct.maxRotation) + ".")
    end
    if instructedRotation < -ServoMotorStruct.maxRotation
        error("Instructed rotation exceeded max rotation. Instructed value when thrown = " + string(instructedRotation) + ". Max = " + string(ServoMotorStruct.maxRotation) + ".")
    end

    if commitResult
        ServoMotorStruct.instructedRotation = instructedRotation;
    end

end
