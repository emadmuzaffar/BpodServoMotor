function trialDistance = getTrialDistance(trialInstructions)

nInstructions = numel(trialInstructions);
trialDistance = 0;

if nInstructions < 1
    error('Trial must have at least one instruction.');
end
if nInstructions > 199
    error('Trial must have at most 199 instructions.');
end

for i = 1:nInstructions
    trialDistance = trialDistance + getDistance(trialInstructions{i});
end

end
