function BlinkModuleSerial2
% Sends a byte on Bpod serial port 2 so BlinkModule can show received commands.
%
% Copy or symlink this folder into your Bpod Protocols folder if needed:
%   BpodProtocols/BlinkModuleSerial2/BlinkModuleSerial2.m

global BpodSystem

serialPort = 'Serial2';

S = BpodSystem.ProtocolSettings;
if isempty(fieldnames(S))
    S.GUI.MaxTrials = 20;
    S.GUI.BlinkByte = 3;
    S.GUI.PostCommandWait = 1;
    S.GUI.ITI = 1;
end

BpodParameterGUI('init', S);

for currentTrial = 1:S.GUI.MaxTrials
    S = BpodParameterGUI('sync', S);

    blinkByte = uint8(max(0, min(254, round(S.GUI.BlinkByte))));
    LoadSerialMessages(serialPort, {blinkByte});

    sma = NewStateMachine();

    sma = AddState(sma, 'Name', 'SendBlinkCommand', ...
        'Timer', S.GUI.PostCommandWait, ...
        'StateChangeConditions', {'Tup', 'ITI'}, ...
        'OutputActions', {serialPort, 1});

    sma = AddState(sma, 'Name', 'ITI', ...
        'Timer', S.GUI.ITI, ...
        'StateChangeConditions', {'Tup', 'exit'}, ...
        'OutputActions', {});

    SendStateMachine(sma);
    RawEvents = RunStateMachine();

    if ~isempty(fieldnames(RawEvents))
        BpodSystem.Data = AddTrialEvents(BpodSystem.Data, RawEvents);
        BpodSystem.Data.TrialSettings(currentTrial) = S;
        BpodSystem.Data.BlinkByte(currentTrial) = blinkByte;
        SaveBpodProtocolData;
    end

    HandlePauseCondition;
    if BpodSystem.Status.BeingUsed == 0
        return
    end
end
end
