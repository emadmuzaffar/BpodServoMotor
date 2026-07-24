function validateTrialType(trialInstructions) 
global ServoMotorStruct

maxTrialDistance = ServoMotorStruct.maxRotation * 2;

trialDistance = getTrialDistance(trialInstructions);

if abs(trialDistance) > maxTrialDistance
    error(['Trial net rotation magnitude of %g degrees exceeds the maximum ' ...
        'allowed trial movement of %g degrees.'], ...
        abs(trialDistance), maxTrialDistance)
end 

end
