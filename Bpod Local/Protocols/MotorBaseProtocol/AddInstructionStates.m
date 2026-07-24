%% Function to send multiple instructions to Arduino servo motor module

% Wraps Bpod AddState() for ease of use, eliminates the need to manually
% declare a series of states to send >5 byte messages by implicitly
% creating states automatically.

% Arguments: 
% sma, the state machine
% namestr, follows Bpod AddState() conventions
% stateName, assigned to name of the 1st implicit state, transition to this
% conditionstr, follows Bpod AddState() conventions
% 


function sma_out = AddInstructionStates(sma, namestr, stateName, conditionstr, stateChangeConditions, outputstr, outputActions, ~, instructions)

global ServoMotorStruct

servoMotor = 'ServoMotor1';
receivedInstruction = 'ServoMotor1_2';
disableMotor = uint8(249);

nInstructions = numel(instructions);

if nInstructions < 1
    error('AddInstructionStates requires at least one instruction.');
end

if nInstructions > 199
    error('AddInstructionStates supports at most 199 instructions.');
end

instructionOpCode = uint8(1);
names = cell(1, nInstructions);
abortState = sprintf('%s_Abort', stateName);

% Validate the complete sequence before committing any planned rotation.
% This prevents a later invalid instruction from partially advancing the
% MATLAB-side safety state when no state machine has been sent yet.
plannedRotation = ServoMotorStruct.instructedRotation;
for i = 1:nInstructions
    plannedRotation = checkInstructionSafety(instructions{i}, plannedRotation);
end
for i = 1:nInstructions
    names{i} = sprintf('%s_Instruction%d', stateName, i);
end

% Send an explicit two-byte header so Bpod stores it as an implicit serial
outputActions = [outputActions, ...
    {servoMotor, uint8([instructionOpCode, nInstructions])}];

sma = AddState(sma, namestr, stateName, ...
    'Timer', 0.0001, ...
    'StateChangeConditions', {'Tup', names{1}}, ...
    outputstr, outputActions);

 
% Send instructions through for loop
% uint16 speed, uint16 time, uint8 direction.
for i = 1:nInstructions

    if i < nInstructions
        nextState = names{i + 1};
        instructionCompleteConditions = {receivedInstruction, nextState, ...
            'Tup', abortState};
    else
        instructionCompleteConditions = [stateChangeConditions, ...
            {'Tup', abortState}];
    end

    sma = AddState(sma, namestr, names{i}, ...
        'Timer', 1, ...
        conditionstr, instructionCompleteConditions, ...
        'OutputActions', {servoMotor, uint8([ ...
            typecast(uint16(instructions{i}{1}), 'uint8'), ...
            typecast(uint16(instructions{i}{2}), 'uint8'), ...
            uint8(instructions{i}{3}) ...
        ])});

end

% A missing parser acknowledgement disables the motor and exits instead of
% leaving Bpod stuck indefinitely in an instruction state.
sma = AddState(sma, namestr, abortState, ...
    'Timer', 0.001, ...
    'StateChangeConditions', {'Tup', 'exit'}, ...
    'OutputActions', {servoMotor, disableMotor});

ServoMotorStruct.instructedRotation = plannedRotation;
sma_out = sma;

end
