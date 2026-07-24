function messages = getInstructionCommand(speed, time, direction)
    instructionOpCode = uint8(1);
    instructionCount = uint8(1);

    global ServoMotorStruct

    plannedRotation = checkInstructionSafety( ...
        {speed, time, direction}, ServoMotorStruct.instructedRotation);

    speedBytes = typecast(uint16(speed), 'uint8');
    timeBytes = typecast(uint16(time), 'uint8');
    directionByte = uint8(direction);

    messages = {
        uint8([instructionOpCode, instructionCount]) ...
        uint8([speedBytes, timeBytes, directionByte])
    };

    ServoMotorStruct.instructedRotation = plannedRotation;
    
end
