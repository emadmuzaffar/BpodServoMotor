

% Direction values: 1 = counterclockwise, 0 = clockwise. (right hand rule)
% Instruction speeds are specified as belt-driven-part degrees per second
% and times are specified in milliseconds.
% The format is {{instruction 1}, {instruction 2},...} with
% each instruction declared as {speed, time, direction}

% Potential Errors:
%
% If you command the motor to exceed its limit and error will appear in the
% Matlab Command Window, correct the instructions or increase the max
% travel and restart the protocol
%
% The Matlab side rotation limiter is not foolproof as it is designed to
% preserve trial flexibility, because of this it is possible to trigger an
% emergency stop path on the Arduino by rotating to far. To reset
% emergency stops press the RESET button on the Arduino (small beige button
% next to the USB ports)
%
% If commands are sent to the Arduino but the motor was not powered, it's
% likely that the Arduino will enter an emergency stop path, to reset
% emergency stops press the RESET button on the Arduino (small beige button
% next to the USB ports)
%
% The Arduino module must appear in the Bpod GUI module bar as 
% "ServoMotor1". If it does not, close the GUI, run clear all, press the 
% reset button on the Arduino and restart the GUI. (this is rare and due to
% the Bpod GUI not being closed correctly)
%
% Things that look/sound like problems but are not actually problems:
%
% When the motor is first powered, it will move slightly and make
% unpleasant sounds, This is document by the manufacturer and should never
% exceed +-1 deg of movement.
%
% If the motor is enabled and commanded to be still, it will occasionally
% make noises as if it is moving, this is mostly due to the lack of
% mechanical load, and will be less audible when attached to a rig.
%

function MotorBaseProtocol
global BpodSystem
ServoMotorData;
frequency = 4000;
startWavePlayer(frequency)

wavePlayer = 'WavePlayer1';
soundAction = ['P' 1 1];  % Channel 1, Waveform 1 = Sound
lightAction = ['P' 2 0];  % Channel 2, Waveform 0 = Light

%% Normal trial variables
maxTrials = 8; % Standard Bpod max trials
minInterTrialDelay = 3;
maxInterTrialDelay = 8;
minHomingDelay = 3;
maxHomingDelay = 8;
minTimeToZap = 1;
maxTimeToZap = 2;

%% Treadmill Callback variables
callbackSpeed = 50;
callbackAveragingTimePeriodMS = 1; % time over which the callback is averaged (0 == disabled)


%% ServoMotor Variables, all max out at 65500 (uint-max16)
% Bpod motion values describe the belt-driven part. The Due converts them to
% the 1080 RPM motor-side full scale configured in ClearPath MSP.
maxDrivenRPM = 120;
timeMultiplier = 1; % Multiplies the times sent in instructions, useful if desired time in ms exceeds 65500 or for more precision
tolerance = 15; % % If error in degrees exceeds this, the system will correct, corrective must be enabled for correction.
encoderPPR = 3600; % Encoder pulses per revolution; firmware uses x4 quadrature counts
    % MAX ROTATION SHOULD BE 720 FOR 2 ROTATION MAX IN EITHER DIRECTION 
maxRotation = 720; % Maximum travel from home (start position), in degrees in either directions, 
% Corrective speed of the belt-driven part in degrees/second. The Due
% receives whole driven-part RPM, so use 6 deg/s increments.
correctiveSpeedDegreesPerSecond = 108;
correctivePosErrorMultiplier = 1.056; % Should not be touched unless the mechanical properties of the setup change
correctiveEnable = false; % If true, the motor will correct for its position when there are no instructions
correctionStartDelayMs = 1000; % Delay before correction activates
homeSpeed = 15;

% Keep trial generation aligned with the configuration that will be sent to
% the Due later in the pretrial state machine.
ServoMotorStruct.maxRotation = maxRotation;
ServoMotorStruct.timeMultiplier = timeMultiplier;
ServoMotorStruct.maxDrivenRPM = maxDrivenRPM;



%% Experimental variables. Set up for the 3 trial types here, others can be made.
trialSetting = 2; % 1 = Emad test setup, Experimental setup with homing

% Settings for trial type 1
if trialSetting == 1
    trial1TypeSelector = 1; % Could be random to pick a random trial type
    if trial1TypeSelector == 0 % Random Instruction System
        s1 = randi([1, 5])*90; % Speed is a multiple of 90
        t1 = randi([1, 4])*500; % Time is a multiple of 500
        d1 = randi([0,1]); % Random Direction
        s2 = randi([1, 5])*90;
        t2 = randi([1, 4])*500;
        d2 = randi([0,1]);
        readout = [s1, t1, s2, t2]; % Print what speeds were randomly selected
        instructions = {{s1,t1,d1}, {s2,t2,d2}};
    elseif trial1TypeSelector == 1 % 360 deg/s for 1000ms ccw
        instructions = {{360, 1000, 1}};
    elseif trial1TypeSelector == 2 % 360 deg/s for 500ms ccw immediately followed by 180 deg/s for 1000ms ccw
        instructions = {{360, 500, 1}, {180, 1000, 1}};
    elseif trial1TypeSelector == 3 % 360 deg/s for 333ms ccw immediately followed by 180 deg/s for 1000ms ccw followed by 90 deg/s 666ms ccw
        instructions = {{360, 333, 1}, {180, 1000, 1}, {90, 666, 1}};
    elseif trial1TypeSelector == 4 % 360 deg/s for 333ms ccw then 180 deg/s for 500ms ccw then 90 deg/s 1000ms ccw then 30 deg/s for 2000ms ccw
        instructions = {{360, 333, 1}, {180, 500, 1} {90, 1000, 1} {30, 2000, 1}};
    end
end

% Settings for trial type 2
if trialSetting == 2
    
    trialTypes = { ...
        {{180, 1000, 0}, {90, 2000, 0}}, ... % Trial type 1
        {{90, 2000, 0}, {180, 1000, 0}}, ... % Trial type 2
        {{180, 1000, 1}, {90, 2000, 1}}, ...
        {{90, 2000, 1}, {180, 1000, 1}}, ...
        };
    trialInstructionSeries = GenerateHomingTrialSeries(maxTrials, trialTypes);
    
    zapProbabilities = {0.75, 0.50, 0.25, 0};

    soundProbalities = {0, 0.25, 0.50, 0.75};
    
end







%% !! STUFF BELOW THIS LINE IS NOT REGULARLY USEFUL, DO NOT TOUCH UNLESS YOU KNOW WHAT YOU ARE DOING !!


% Constants DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING:
servoMotor = 'ServoMotor1'; % Send message to the servoMotor using normal AddState OuputActions with {servoMotor, message}
encoderModule = 'EmadRotaryEncoder1';

% Callbacks
% (only use instructionsCompleted if you do not want to wait for correction, correction completed still works with correction off and is more robust)
instructionsCompleted = 'ServoMotor1_1'; % Callback received when all instructions sent to the servoMotor are completed 

instructionsReceived = 'ServoMotor1_2'; % Callback to spend minimum time in config or instruction states
correctionCompleted = 'ServoMotor1_3'; % Callback received when all instructions and following correction are completed (motor is completely still)

atRunningSpeed = 'EmadRotaryEncoder1_1'; % Callback when configured treadmill speed is achieved

% Enable and Disable
enableMotor = uint8(250); % Can also be sent through the Bpod GUI serial terminal 
disableMotor = uint8(249); % Can also be sent through the Bpod GUI serial terminal
resetEncoderPosition = uint8(253); % Only run when setup faces north (Center of movement capability)


% Pretrial State machine, runs once before trials start
% Sends the above configuration to the arduino using the AddConfigStates function which is wrapper of AddState
% Only one config can be sent per trial
sma = NewStateMachine();
sma = AddConfigStates(sma, 'Name', 'ConfigureMotorState', ...
    'StateChangeConditions', {instructionsReceived, 'CameraOnState'}, ...
    'OutputActions', {encoderModule, ...
        getEncoderConfig(callbackSpeed, callbackAveragingTimePeriodMS)}, ...
    'maxDrivenRPM', maxDrivenRPM, ...
    'timeMultiplier', timeMultiplier, ...
    'tolerance', tolerance, ...
    'encoderPPR', encoderPPR, ...
    'maxRotation', maxRotation, ...
    'correctiveSpeedDegreesPerSecond', correctiveSpeedDegreesPerSecond, ...
    'correctivePosErrorMultiplier', correctivePosErrorMultiplier, ...
    'correctiveEnable', correctiveEnable, ...
    'correctionStartDelayMs', correctionStartDelayMs);
sma = AddState(sma, 'Name', 'CameraOnState', ...
    'Timer', 0.1, ...
    'StateChangeCondtitions', {'Tup', 'EnableMotorState'}, ...
    'OutputActions', {'BNC1', 1, 'BNC2', 1});
sma = AddState(sma, 'Name', 'EnableMotorState', ...
    'Timer', 0.01, ...
    'StateChangeConditions', {'Tup', 'exit'}, ...
    'OutputActions', {servoMotor, enableMotor, ...
    'BNC1', 1, 'BNC2', 1});
SendStateMachine(sma);
RunStateMachine();
HandlePauseCondition;
if BpodSystem.Status.BeingUsed == 0
    return
end


% Very simple protocol   
if trialSetting == 1   
    BpodSystem.Data = struct;
    for currentTrial = 1:maxTrials
        
        trialInterval = randi([minInterTrialDelay, maxInterTrialDelay]);

        sma = NewStateMachine();
    
        % Sends an instruction or a series of instructions to the motor using the AddConfigStates function which is wrapper of AddState
        % Can be used multiple times in the same trial
    
        % Even one instruction must use AddInstructionStates because the
        % command is transmitted as separate header and payload messages.
    
        sma = AddInstructionStates(sma, 'Name', 'SendInstructionsState', ... % Use AddInstructions to send a series of instructions
            'StateChangeConditions', {instructionsReceived, 'WaitForCompletion'}, ... % Transition states upon message send completion
            'OutputActions', {}, ... % Blank for no additional actions other than sending instructions
            'Instructions', instructions); % Series of instructions, {{driven-part speed in deg/s, time in ms, direction 1=ccw and 0=cw}, {speed, time, direction}}
            % In this trial the instruction cell array is declared under trial variables, it could also be declared inline here
        
       % Waits for previously sent instructions to be completed
       sma = AddState(sma, 'Name', 'WaitForCompletion', ...
           'Timer', 1, ...
           'StateChangeConditions', {instructionsCompleted, 'WaitState'}, ... % Uses correction completed callback from servoMotor 
           'OutputActions', {});
       
       % Wait 1 second before disabling the motor after instruction completion
       sma = AddState(sma, 'Name', 'WaitState', ...
           'Timer', trialInterval, ...
           'StateChangeConditions', {'Tup', 'exit'}, ...
           'OutputActions', {});
    

       %Bpod Stuff:
       SendStateMachine(sma);
       RunStateMachine();
       HandlePauseCondition;
       if BpodSystem.Status.BeingUsed == 0
           sma = NewStateMachine();
           sma = AddState(sma, 'Name', 'DisableMotor', ...
               'Timer', 0.1, ...
               'StateChangeConditions', {'Tup', 'exit'}, ...
               'OutputActions', {servoMotor, disableMotor});
           SendStateMachine(sma);
           RunStateMachine();
           return
       end
    
    end
end


% Normal experimental setup that homes after every trial
if trialSetting == 2   
    BpodSystem.Data = struct;
    for currentTrial = 1:maxTrials
        
        trialInterval = randi([minInterTrialDelay, maxInterTrialDelay]);
        delayToHome = randi([minHomingDelay, maxHomingDelay]);
        timeToZap = randi([minTimeToZap, maxTimeToZap]);
        trialTypInt = getTrialInt(trialTypes, trialInstructionSeries{currentTrial});

        sma = NewStateMachine();
           
        if rand < soundProbalities{trialTypInt}
            sma = AddState(sma, 'Name', 'soundState', ...
                'Timer', 1, ...
                'StateChangeConditions', {'Tup', 'SendInstructionsState'}, ...
                'OutputActions', {wavePlayer, soundAction});
        end
        
        sma = AddInstructionStates(sma, 'Name', 'SendInstructionsState', ... % Use AddInstructions to send a series of instructions
            'StateChangeConditions', {instructionsReceived, 'LogicState'}, ... % Transition states upon message send completion
            'OutputActions', {}, ... % Blank for no additional actions other than sending instructions
            'Instructions', trialInstructionSeries{currentTrial}); % Series of instructions, {{driven-part speed in deg/s, time in ms, direction 1=ccw and 0=cw}, {speed, time, direction}}
   
        sma = AddState(sma, 'Name', 'LogicState', ...
            'Timer', timeToZap, ...
            'StateChangeConditions', {atRunningSpeed, 'pWaitState1', instructionsCompleted, 'WaitState1', 'Tup', 'ZapThenLogicState'}, ...
            'OutputActions', {});
        
        if rand < zapProbabilities{trialTypInt} 
            sma = AddState(sma, 'Name', 'ZapThenLogicState', ...
                'Timer', 1, ...
                'StateChangeConditions', {atRunningSpeed, 'pWaitState1', instructionsCompleted, 'WaitState1'}, ...
                'OutputActions', {wavePlayer, lightAction}); % Figure out zap logic
            trialTypInt
            "zapped"
        else
            sma = AddState(sma, 'Name', 'ZapThenLogicState', ...
                'Timer', 1, ...
                'StateChangeConditions', {atRunningSpeed, 'pWaitState1', instructionsCompleted, 'WaitState1'}, ...
                'OutputActions', {}); 
            trialTypInt
            "nozapped"
        end

        sma = AddInstructionStates(sma, 'Name', 'pWaitState1', ...
            'StateChangeConditions', {instructionsReceived, 'WaitState1'}, ...
            'OutputActions', {}, ...
            'Instructions', {{0, maxHomingDelay * 1000, 0}});
            
        sma = AddState(sma, 'Name', 'WaitState1', ...
            'Timer', delayToHome, ...
            'StateChangeConditions', {'Tup', 'SendHomeInstruction'}, ...
            'OutputActions', {});

        sma = AddState(sma, 'Name', 'SendHomeInstruction', ...
            'Timer', 1, ...
            'StateChangeConditions', {instructionsCompleted, 'WaitState2'}, ...
            'OutputActions', {servoMotor, getHomeCommand(homeSpeed)});

        sma = AddState(sma, 'Name', 'WaitState2', ...
            'Timer', trialInterval, ...
            'StateChangeConditions', {'Tup', 'exit'}, ...
            'OutputActions', {});
        
        SendStateMachine(sma);
        RunStateMachine();
        HandlePauseCondition;
        if BpodSystem.Status.BeingUsed == 0
            sma = NewStateMachine();
            sma = AddState(sma, 'Name', 'DisableMotor', ...
                'Timer', 0.1, ...
                'StateChangeConditions', {'Tup', 'exit'}, ...
                'OutputActions', {servoMotor, disableMotor});
            SendStateMachine(sma);
            RunStateMachine();
            return
        end
        currentTrial
    end
end

% ignore
if trialSetting == 3
    error("not yet implemented")
end

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
"finished"
end
