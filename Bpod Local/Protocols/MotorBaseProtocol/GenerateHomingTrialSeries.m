function instructions = GenerateHomingTrialSeries(trials, trialTypes)

validateattributes(trials, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonnegative'}, ...
    mfilename, 'trials');
nTrialTypes = numel(trialTypes);
instructions = cell(1, trials);

validateHomingProtocolTrialTypes(trialTypes);

for n = 1:trials
    trialTypeIndex = randi(nTrialTypes);
    instructions{n} = trialTypes{trialTypeIndex};
end

end
