function disableMotorOnEnd()
servoMotor = 'ServoMotor1';
disableMotor = uint8(250);
% Post trial state machine to disable the motor
sma = NewStateMachine();
sma = AddState(sma, 'Name', 'DisableMotor', ...
    'Timer', 0.1, ...
    'StateChangeConditions', {'Tup', 'exit'}, ...
    'OutputActions', {servoMotor, disableMotor});
SendStateMachine(sma);
RunStateMachine();
HandlePauseCondition;
if BpodSystem.Status.BeingUsed == 0
    return
end