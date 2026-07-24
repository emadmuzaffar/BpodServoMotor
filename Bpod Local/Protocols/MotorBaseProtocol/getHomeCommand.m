function messages = getHomeCommand(speed)

global ServoMotorStruct

validateattributes(speed, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'positive', ...
        '<=', ServoMotorStruct.maxDrivenRPM * 6}, ...
    mfilename, 'speed');

ServoMotorStruct.instructedRotation = 0;

homeMotorOpCode = uint8(200);
messages = [homeMotorOpCode, typecast(uint16(speed), 'uint8')];
end
