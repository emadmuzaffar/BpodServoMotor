function validateProtocolTrialTypes(trialTypes)
global ServoMotorStruct

nTrialTypes = numel(trialTypes);
if nTrialTypes < 1
    error('At least one trial type required');
end

for n = 1:nTrialTypes
    validateTrialType(trialTypes{n});
end

backupAvailable = false;
for n = 1:nTrialTypes
    if abs(getTrialDistance(trialTypes{n})) <= ServoMotorStruct.maxRotation
        backupAvailable = true;
    end
end
if backupAvailable == false
    error("At least one trial type must move less than the max rotation")
end

end
