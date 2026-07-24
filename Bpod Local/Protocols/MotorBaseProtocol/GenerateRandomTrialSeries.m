
%%
function protocolInstructions = GenerateRandomTrialSeries(trials, trialTypes)
global ServoMotorStruct

validateattributes(trials, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonnegative'}, ...
    mfilename, 'trials');

protocolInstructions = cell(1, trials);
accumulatedRotation = 0;
nTrialTypes = numel(trialTypes);

validateProtocolTrialTypes(trialTypes);
trialDistances = zeros(1, nTrialTypes);
for trialTypeIndex = 1:nTrialTypes
    trialDistances(trialTypeIndex) = ...
        getTrialDistance(trialTypes{trialTypeIndex});
end

for trialIndex = 1:trials
    validTrialTypeIndices = find( ...
        abs(accumulatedRotation + trialDistances) <= ...
        ServoMotorStruct.maxRotation);
    if isempty(validTrialTypeIndices)
        error(['Randomization failed at trial %d: no trial type stays ' ...
            'within the configured maximum rotation.'], trialIndex)
    end

    selectedIndex = validTrialTypeIndices( ...
        randi(numel(validTrialTypeIndices)));
    protocolInstructions{trialIndex} = trialTypes{selectedIndex};
    accumulatedRotation = ...
        accumulatedRotation + trialDistances(selectedIndex);
end

end




