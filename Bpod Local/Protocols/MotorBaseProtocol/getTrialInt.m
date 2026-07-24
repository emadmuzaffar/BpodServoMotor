function i = getTrialInt(trialTypes, trialInstructions)

nTrials = numel(trialTypes);

for n = 1:nTrials
    if trialInstructions == trialTypes{n}
        i = n;
        return
    end
end

error("No matches found")
end