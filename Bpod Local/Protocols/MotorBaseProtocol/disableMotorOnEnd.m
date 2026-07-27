function disableMotorOnEnd()
servoMotor = 'ServoMotor1';
% Post trial state machine to disable the motor
sma = NewStateMachine();
sma = AddState(sma, 'Name', 'DisableMotor', ...
    'Timer', 0.1, ...
    'StateChangeConditions', {'Tup', 'exit'}, ...
    'OutputActions', {servoMotor, disableMotor});
SendStateMachine(sma);
RunStateMachine();
HandlePauseCondition;
end