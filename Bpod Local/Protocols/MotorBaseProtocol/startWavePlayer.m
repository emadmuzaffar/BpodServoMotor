function startWavePlayer(frequency)

global BpodSystem
S = BpodSystem.ProtocolSettings;
BpodSystem.assertModule('WavePlayer', 1);
W = BpodWavePlayer(BpodSystem.ModuleUSB.WavePlayer1);

% Non-GUI Parameters
SoundDuration = 2;          % seconds
SoundAmplitude = 2;         % Volume control
UCSDuration = 2;            % LED stimulation duration
UCSVoltage = 0.2;             % LED voltage          % Block 4: sound-only
% Mixed block proportions
pctPaired = 0.75;           % sound+light (CS+US)
pctLightOnly = 1 - pctPaired; % light-only within mixed block
%% Generate and Load Waveforms
W.SamplingRate = 192000;
UCSWave = ones(1, W.SamplingRate * UCSDuration);
CSWave  = GenerateSineWave(W.SamplingRate, frequency, SoundDuration);
% Wave IDs: 1=UCS (LED), 2=CS (Sound)
W.loadWaveform(1, UCSWave * UCSVoltage);
W.loadWaveform(2, CSWave  * SoundAmplitude);
"pulsesLoded"

end