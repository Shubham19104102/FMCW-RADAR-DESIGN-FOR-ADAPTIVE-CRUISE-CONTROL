%% FMCW RADAR FOR ADAPTIVE CRUISE CONTROL
% Complete simulation based on the project report

clc;
clear;
close all;

%% ============================================================
% 1. BASIC RADAR PARAMETERS
% =============================================================

fc = 77e9;                 % Carrier frequency = 77 GHz
c = 3e8;                   % Speed of light
lambda = c/fc;             % Wavelength

range_max = 200;           % Maximum detection range (m)
range_res = 1;             % Required range resolution (m)

v_max = 230*1000/3600;     % Maximum vehicle speed (m/s)

%% ============================================================
% 2. FMCW WAVEFORM PARAMETERS
% =============================================================

% Round trip time corresponding to maximum range
td_max = 2*range_max/c;

% Sweep time = 5.5 times maximum round-trip time
tm = 5.5*td_max;

% Sweep bandwidth for required range resolution
bw = c/(2*range_res);

% FMCW sweep slope
sweep_slope = bw/tm;

fprintf('\n========== RADAR PARAMETERS ==========\n');
fprintf('Carrier frequency       = %.2f GHz\n',fc/1e9);
fprintf('Maximum range           = %.2f m\n',range_max);
fprintf('Range resolution        = %.2f m\n',range_res);
fprintf('Sweep time              = %.3f us\n',tm*1e6);
fprintf('Sweep bandwidth         = %.2f MHz\n',bw/1e6);
fprintf('Sweep slope             = %.3e Hz/s\n',sweep_slope);

%% ============================================================
% 3. MAXIMUM BEAT FREQUENCY AND SAMPLE RATE
% =============================================================

% Maximum range beat frequency
fr_max = 2*sweep_slope*range_max/c;

% Maximum Doppler frequency
fd_max = 2*v_max/lambda;

% Maximum beat frequency
fb_max = fr_max + fd_max;

% Sampling frequency
fs = max(2*fb_max,bw);

fprintf('Maximum range beat freq = %.2f MHz\n',fr_max/1e6);
fprintf('Maximum Doppler freq    = %.2f MHz\n',fd_max/1e6);
fprintf('Maximum beat freq       = %.2f MHz\n',fb_max/1e6);
fprintf('Sampling frequency      = %.2f MHz\n',fs/1e6);

%% ============================================================
% 4. FMCW WAVEFORM GENERATION
% =============================================================

waveform = phased.FMCWWaveform( ...
    'SweepTime',tm, ...
    'SweepBandwidth',bw, ...
    'SampleRate',fs);

sig = waveform();

N = length(sig);
t = (0:N-1)/fs;

%% ============================================================
% 5. PLOT FMCW SIGNAL
% =============================================================

figure;

subplot(2,1,1);
plot(t*1e6,real(sig));
xlabel('Time (\mus)');
ylabel('Amplitude');
title('FMCW Transmitted Signal');
grid on;
axis tight;

subplot(2,1,2);
spectrogram(sig,32,16,32,fs,'yaxis');
title('FMCW Signal Spectrogram');

%% ============================================================
% 6. TARGET MODEL
% =============================================================

% Target initial range
car_dist = 43;             % Target is 43 m away

% Target velocity
car_speed = 96*1000/3600; % 96 km/h = 26.67 m/s

% Radar velocity
radar_speed = 100*1000/3600;

% Relative velocity
relative_speed = car_speed - radar_speed;

fprintf('\n========== TARGET ==========\n');
fprintf('Target distance = %.2f m\n',car_dist);
fprintf('Target speed    = %.2f m/s\n',car_speed);
fprintf('Radar speed     = %.2f m/s\n',radar_speed);
fprintf('Relative speed  = %.2f m/s\n',relative_speed);

%% ============================================================
% 7. RADAR CROSS SECTION OF TARGET
% =============================================================

car_rcs = db2pow(min(10*log10(car_dist)+5,20));

cartarget = phased.RadarTarget( ...
    'MeanRCS',car_rcs, ...
    'PropagationSpeed',c, ...
    'OperatingFrequency',fc);

%% ============================================================
% 8. TARGET MOTION
% =============================================================

carmotion = phased.Platform( ...
    'InitialPosition',[car_dist;0;0.5], ...
    'Velocity',[car_speed;0;0]);

%% ============================================================
% 9. FREE SPACE PROPAGATION CHANNEL
% =============================================================

channel = phased.FreeSpace( ...
    'PropagationSpeed',c, ...
    'OperatingFrequency',fc, ...
    'SampleRate',fs, ...
    'TwoWayPropagation',true);

%% ============================================================
% 10. ANTENNA AND RADAR PARAMETERS
% =============================================================

ant_aperture = 6.06e-4;

ant_gain = aperture2gain(ant_aperture,lambda);

tx_ppower = db2pow(5)*1e-3;

tx_gain = 9 + ant_gain;

rx_gain = 15 + ant_gain;

rx_nf = 4.5;

%% ============================================================
% 11. TRANSMITTER
% =============================================================

transmitter = phased.Transmitter( ...
    'PeakPower',tx_ppower, ...
    'Gain',tx_gain);

%% ============================================================
% 12. RECEIVER
% =============================================================

receiver = phased.ReceiverPreamp( ...
    'Gain',rx_gain, ...
    'NoiseFigure',rx_nf, ...
    'SampleRate',fs);

%% ============================================================
% 13. RADAR PLATFORM
% =============================================================

radarmotion = phased.Platform( ...
    'InitialPosition',[0;0;0.5], ...
    'Velocity',[radar_speed;0;0]);

%% ============================================================
% 14. SPECTRUM ANALYZER
% =============================================================

specanalyzer = spectrumAnalyzer( ...
    'SampleRate',fs, ...
    'Method','welch', ...
    'AveragingMethod','running', ...
    'PlotAsTwoSidedSpectrum',true, ...
    'FrequencyResolutionMethod','rbw', ...
    'Title','Received and Dechirped Signal Spectrum', ...
    'ShowLegend',true);

%% ============================================================
% 15. FMCW SIGNAL SIMULATION
% =============================================================

Nsweep = 64;

Nsamples = round(fs*tm);

xr = complex(zeros(Nsamples,Nsweep));

fprintf('\n========== SIMULATION ==========\n');
fprintf('Number of sweeps = %d\n',Nsweep);
fprintf('Samples/sweep    = %d\n',Nsamples);

for m = 1:Nsweep

    % ---------------------------------------------------------
    % Update radar and target positions
    % ---------------------------------------------------------

    [radar_pos,radar_vel] = radarmotion(tm);

    [tgt_pos,tgt_vel] = carmotion(tm);

    % ---------------------------------------------------------
    % Generate FMCW waveform
    % ---------------------------------------------------------

    sig = waveform();

    % ---------------------------------------------------------
    % Transmit signal
    % ---------------------------------------------------------

    txsig = transmitter(sig);

    % ---------------------------------------------------------
    % Propagate signal to target and back
    % ---------------------------------------------------------

    txsig = channel( ...
        txsig, ...
        radar_pos, ...
        tgt_pos, ...
        radar_vel, ...
        tgt_vel);

    % ---------------------------------------------------------
    % Reflection from target
    % ---------------------------------------------------------

    txsig = cartarget(txsig);

    % ---------------------------------------------------------
    % Receiver
    % ---------------------------------------------------------

    txsig = receiver(txsig);

    % ---------------------------------------------------------
    % Dechirping
    % ---------------------------------------------------------

    dechirpsig = dechirp(txsig,sig);

    % ---------------------------------------------------------
    % Store dechirped signal
    % ---------------------------------------------------------

    xr(:,m) = dechirpsig;

end

fprintf('Simulation completed.\n');

%% ============================================================
% 16. RANGE-DOPPLER RESPONSE
% =============================================================

rngdopresp = phased.RangeDopplerResponse( ...
    'PropagationSpeed',c, ...
    'DopplerOutput','Speed', ...
    'OperatingFrequency',fc, ...
    'SampleRate',fs, ...
    'RangeMethod','FFT', ...
    'SweepSlope',sweep_slope, ...
    'RangeFFTLengthSource','Property', ...
    'RangeFFTLength',2048, ...
    'DopplerFFTLengthSource','Property', ...
    'DopplerFFTLength',256);

%% ============================================================
% 17. RANGE-DOPPLER MAP
% =============================================================

figure;

plotResponse(rngdopresp,xr);

axis([-v_max v_max 0 range_max]);

title('FMCW Radar Range-Doppler Response');

%% ============================================================
% 18. RANGE ESTIMATION USING FFT
% =============================================================

% Average/coherently integrate all sweeps
range_signal = mean(xr,2);

% FFT
Nfft = 4096;

range_fft = fft(range_signal,Nfft);

range_fft = abs(range_fft);

% Only positive frequency
range_fft = range_fft(1:Nfft/2);

% Frequency axis
f_range = (0:Nfft/2-1)*(fs/Nfft);

% Convert beat frequency to range
range_axis = c*f_range/(2*sweep_slope);

%% ============================================================
% 19. PLOT RANGE FFT
% =============================================================

figure;

plot(range_axis,20*log10(range_fft/max(range_fft)));

xlabel('Range (m)');
ylabel('Magnitude (dB)');
title('Range Spectrum');
grid on;

xlim([0 range_max]);

%% ============================================================
% 20. ESTIMATE TARGET RANGE
% =============================================================

% Ignore DC region
search_region = range_axis > 1 & range_axis <= range_max;

[~,idx] = max(range_fft(search_region));

temp_range_axis = range_axis(search_region);

range_est = temp_range_axis(idx);

fprintf('\n========== RANGE ESTIMATION ==========\n');
fprintf('Estimated target range = %.3f m\n',range_est);
fprintf('Actual target range    = %.3f m\n',car_dist);

%% ============================================================
% 21. DOPPLER ESTIMATION
% =============================================================

% Find range bin nearest to estimated range
[~,range_bin] = min(abs(range_axis-range_est));

% Extract signal corresponding to target range
slow_time_signal = xr(range_bin,:);

% Doppler FFT
Ndoppler = 1024;

doppler_fft = fftshift(fft(slow_time_signal,Ndoppler));

doppler_mag = abs(doppler_fft);

% Doppler frequency axis
fd_axis = (-Ndoppler/2:Ndoppler/2-1)/(Ndoppler*tm);

% Convert Doppler frequency to velocity
velocity_axis = fd_axis*lambda/2;

%% ============================================================
% 22. PLOT DOPPLER SPECTRUM
% =============================================================

figure;

plot(velocity_axis,doppler_mag/max(doppler_mag));

xlabel('Velocity (m/s)');
ylabel('Normalized Magnitude');
title('Doppler Spectrum');
grid on;

%% ============================================================
% 23. ESTIMATE VELOCITY
% =============================================================

[~,velocity_idx] = max(doppler_mag);

velocity_est = velocity_axis(velocity_idx);

fprintf('\n========== VELOCITY ESTIMATION ==========\n');
fprintf('Estimated relative velocity = %.3f m/s\n',velocity_est);
fprintf('Actual relative velocity    = %.3f m/s\n',relative_speed);

%% ============================================================
% 24. RANGE-DOPPLER COUPLING
% =============================================================

% Doppler frequency corresponding to relative velocity
fd = 2*relative_speed/lambda;

% Range error due to Doppler coupling
deltaR = c*fd/(2*sweep_slope);

fprintf('\n========== RANGE-DOPPLER COUPLING ==========\n');
fprintf('Doppler frequency = %.3f Hz\n',fd);
fprintf('Range error        = %.6f m\n',deltaR);

%% ============================================================
% 25. LONGER SWEEP TIME
% =============================================================

tm_long = 2e-3;

sweep_slope_long = bw/tm_long;

deltaR_long = c*fd/(2*sweep_slope_long);

fprintf('\n========== LONG SWEEP ==========\n');
fprintf('Sweep time = %.3f ms\n',tm_long*1e3);
fprintf('Range-Doppler coupling error = %.3f m\n',deltaR_long);

%% ============================================================
% 26. MAXIMUM UNAMBIGUOUS VELOCITY
% =============================================================

v_unambiguous = lambda/(4*tm_long);

fprintf('Maximum unambiguous velocity = %.3f m/s\n', ...
    v_unambiguous);

%% ============================================================
% 27. TRIANGULAR FMCW SWEEP
% =============================================================

waveform_tr = phased.FMCWWaveform( ...
    'SweepTime',tm_long, ...
    'SweepBandwidth',bw, ...
    'SampleRate',fs, ...
    'SweepDirection','Triangle');

%% ============================================================
% 28. TRIANGULAR SWEEP SIGNAL
% =============================================================

sig_tri = waveform_tr();

figure;

spectrogram(sig_tri,64,32,128,fs,'yaxis');

title('Triangular FMCW Sweep Spectrogram');

%% ============================================================
% 29. TRIANGULAR SWEEP SIMULATION
% =============================================================

Nsweep_tri = 16;

Nsamples_tri = round(fs*tm_long);

xr_tri = complex(zeros(Nsamples_tri,Nsweep_tri));

for m = 1:Nsweep_tri

    [radar_pos,radar_vel] = radarmotion(tm_long);

    [tgt_pos,tgt_vel] = carmotion(tm_long);

    sig = waveform_tr();

    txsig = transmitter(sig);

    txsig = channel( ...
        txsig, ...
        radar_pos, ...
        tgt_pos, ...
        radar_vel, ...
        tgt_vel);

    txsig = cartarget(txsig);

    txsig = receiver(txsig);

    dechirpsig = dechirp(txsig,sig);

    xr_tri(:,m) = dechirpsig;

end

fprintf('\nTriangular sweep simulation completed.\n');

%% ============================================================
% 30. SEPARATE UP AND DOWN SWEEPS
% =============================================================

up_sweeps = xr_tri(:,1:2:end);

down_sweeps = xr_tri(:,2:2:end);

%% ============================================================
% 31. RANGE FFT FOR UP SWEEP
% =============================================================

up_signal = mean(up_sweeps,2);

Nfft_tri = 4096;

up_fft = abs(fft(up_signal,Nfft_tri));

up_fft = up_fft(1:Nfft_tri/2);

f_up = (0:Nfft_tri/2-1)*(fs/Nfft_tri);

range_up = c*f_up/(2*sweep_slope_long);

%% ============================================================
% 32. RANGE FFT FOR DOWN SWEEP
% =============================================================

down_signal = mean(down_sweeps,2);

down_fft = abs(fft(down_signal,Nfft_tri));

down_fft = down_fft(1:Nfft_tri/2);

f_down = (0:Nfft_tri/2-1)*(fs/Nfft_tri);

range_down = c*f_down/(2*sweep_slope_long);

%% ============================================================
% 33. PLOT UP AND DOWN SWEEP
% =============================================================

figure;

plot(range_up, ...
    20*log10(up_fft/max(up_fft)), ...
    'LineWidth',1.2);

hold on;

plot(range_down, ...
    20*log10(down_fft/max(down_fft)), ...
    'LineWidth',1.2);

xlabel('Range (m)');
ylabel('Magnitude (dB)');
title('Up-Sweep and Down-Sweep Range Spectrum');

legend('Up Sweep','Down Sweep');

grid on;

xlim([0 range_max]);

%% ============================================================
% 34. FINAL RESULTS
% =============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('             FINAL RESULTS\n');
fprintf('============================================\n');

fprintf('Actual Target Range       = %.2f m\n',car_dist);
fprintf('Estimated Target Range    = %.2f m\n',range_est);

fprintf('Actual Relative Velocity  = %.2f m/s\n',relative_speed);
fprintf('Estimated Velocity         = %.2f m/s\n',velocity_est);

fprintf('Range Resolution           = %.2f m\n',range_res);
fprintf('Maximum Detection Range   = %.2f m\n',range_max);

fprintf('============================================\n');