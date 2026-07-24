function validateHomingProtocolTrialTypes(trialTypes)
global ServoMotorStruct

nTrials = numel(trialTypes);
if nTrials < 1
    error('At least one homing trial type is required.')
end

for i = 1:nTrials
    validateTrialType(trialTypes{i});
    trialDistance = getTrialDistance(trialTypes{i});
    if abs(trialDistance) > ServoMotorStruct.maxRotation
        error(['Homing trial type %d ends at %g degrees, beyond the ' ...
            'configured maximum rotation of %g degrees.'], ...
            i, trialDistance, ServoMotorStruct.maxRotation)
    end
end

end
