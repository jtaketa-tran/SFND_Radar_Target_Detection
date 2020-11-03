% Constant False Alarm Rate
clear all; close all;

N = 1000; % Number of Samples
signal = abs(randn(N,1)); % Generate Random Noise

% Set Target Locations:
% Assign Targets to Bins 100, 200, 300 and 700
% Assign Target Amplitudes: 8, 9, 4, 11
signal([100 200 300 700]) = [8 9 4 11];
%signal([100 200 350 700]) = [8 15 7 13];

figure; plot(signal);

T = 12; % Number of Training Cells
G = 4; % Number of Guard Cells
tnr = 3.5; % define threshold-to-noise ratio
threshold_cfar = []; % define vector for threshold values
signal_cfar = []; % define vector for thresholded signal result

% Sliding window filter: make sure we give enough room at the end for the
% number of guard cells and training cells
for winIdx = 1:(N-(G+T))
    
    % Estimate the average noise across the training cells
    noise = sum(signal(winIdx:winIdx+T-1))/T; 
    
    % Compute threshold
    threshold = noise*tnr;
    threshold_cfar = [threshold_cfar,{threshold}];
    
    % Measure signal within the cell under test (CUT)
    CUTsig = signal(winIdx+T+G); % define as T+G cells away from the first training cell
    
    % If the signal level at CUT is below the threshold
    if CUTsig < threshold
        % then assign it a 0 value
        CUTsig = 0;
    end
    
    signal_cfar = [signal_cfar, {CUTsig}];
end

% Plot the filtered signal
plot(cell2mat(signal_cfar),'g--');

figure; plot(signal);
hold on; plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2);
hold on; plot(cell2mat(circshift(signal_cfar,(T+G))),'g.','MarkerSize',14);
legend('Signal','CFAR Threshold','Detection');

% 2-D CFAR on Range-Doppler Map
% 1. Determine the number of Training Cells (Tr & Td) and the number of
% Guard Cells (Gr & Gd) for each dimension 
% 2. Slide the Cell Under Test (CUT) across the complete cell matrix
% 3. Select the grid that includes the training, guard and test cells. Grid
% Size = (2Tr+2Gr+1)(2Td+2Gd+1) 
% 4. The total number of cells in the guard region and cell under test.
% (2Gr+1)(2Gd+1) 
% 5. This gives the Training Cells : (2Tr+2Gr+1)(2Td+2Gd+1) - (2Gr+1)(2Gd+1)
% 6. Measure and average the noise across all the training cells. This
% gives the threshold 
% 7. Add the offset (if in signal strength in dB) to the threshold to keep
% the false alarm to the minimum 
% 8. Determine the signal level at the Cell Under Test 
% 9. If the CUT signal level is greater than the Threshold, assign a value
% of 1, else equate it to zero 
% 10. Since the cell under test are not located at the edges, due to the
% training cells occupying the edges, we suppress the edges to zero. Any
% cell value that is neither 1 nor a 0, assign it a zero  
% 
% *************************************************************************
% 2-D FFT

% Create and plot 2-D data with repeated blocks
P = peaks(20);
X = repmat(P,[5 10]);
imagesc(X);

% Compute the 2-D Fourier Transform of the data
X_fft = fft2(X);

% Shift the zero-frequency component to the center of the output
X_fft = abs(fftshift(X_fft));

% Plot the resulting 100-by-200 matrix, which is the same size as X
figure; imagesc(log(X_fft));

% *************************************************************************
% Find the frequency components of a signal buried in noise

% Define FFT parameters
fs = 1.0e3; % sampling frequency
T = 1/fs; % sampling period
L = 1500; % length of signal (time duration) in ms
%N = 1024; % number of samples

% Define a signal and add noise
t = (0:L-1)*T; % time vector
f = [77 43]; % frequency of two signals in Hz
A = [0.7 2.0]; % signal amplitudes
%S = 0.7*sin(2*pi*77*t) + 2*sin(2*pi*43*t); % note: class notes suggested cos
signal = sum(A'.*sin(2*pi*f'.*t));
noisedSignal = signal; % + 2*randn(size(t));

figure; plot(1000*t(1:50),noisedSignal(1:50));
title('Signal Plus Zero-Mean Random Noise');
xlabel('time [ms]');
ylabel('noised signal');

% Compute the FFT of the noised signal
signal_fft = fft(noisedSignal); %,N);

% Compute the amplitude of the normalized signal
%signal_fft = abs(signal_fft);  % take the magnitude of the fft
signal_fft = abs(signal_fft/L); % take the amplitude of the fft

% Compute the single-sided spectrum based on the even-valued signal length L
%signal_fft = signal_fft(1:L/2-1); % discard negative half of fft (mirrored image)
signal_fft = signal_fft(1:L/2+1); % discard negative half of fft (mirrored image)

freq = fs*(0:(L/2))/L;
figure; plot(freq,signal_fft);
title('Single-Sided Amplitude Spectrum of Noisy Signal');
xlabel('frequency [Hz]');
ylabel('amplitude');

% *************************************************************************
% Doppler Velocity Calculation
% Calculate the velocity in m/s of four targets with the following Doppler
% frequency shifts: [3 KHz, -4.5 KHz, 11 KHz, -3 KHz]

% Radar Operating Frequency (Hz)
fc = 77.0e9;

% Speed of Light (meters/sec)
c = 3*10^8;

% Wavelength
lambda = c/fc;

% Doppler Shifts in Hz
fd = [3.0e3, -4.5e3, 11.0e3, -3e3];

% Velocity of the targets
%fd = (2*vr)/lambda;
vr = (fd*lambda)/2;

disp(vr)

% velocity = distance/time

% *************************************************************************
% Given the radar maximum range of 300m and range resolution of 1m,
% calculate the range of four targets with the following measured beat
% frequencies: 0MHz, 1.1MHz, 13MHz, 24MHz

% Speed of Light (meters/sec)
c = 3*10^8;

% Maximum Detectable Range in Meters
Rmax = 300;

% Range Resolution in Meters
deltaR = 1;

% Bsweep of Chirp
Bsweep = c/2*deltaR;

% The sweep/chirp time can be computed based on the time needed for the signal to
% travel the maximum range. In general, for an FMCW radar system, the sweep
% time should be at least 5 to 6 times the round trip time. This example
% uses a factor of 5.5:
Tchirp = 5.5*(Rmax*2/c); % 5.5 times the trip time at maximum range

% Beat Frequencies (Hz) of four targets:
fb = [0e6,1.1e6,13e6,24e6];

% Range for every target (meters)
calculated_range = c*Tchirp*fb/(2*Bsweep); 

disp(calculated_range);

% *************************************************************************
% % Operating Frequency (Hz)
% fc = 77.0e9;
% % Wavelength
% lambda = c/fc;
% % Transmitted Power (W)
% Pt = 3e-3;
% 
% % Antenna Gain (linear)
% G = 10000;
% 
% % Minimum Detectable Power
% Ps = 1e-10;
% 
% % RCS of a car
% RCS = 100;
% 
% Range = (((10*log10(Pt*1000))*(G^2)*(lambda^2)*RCS)/(Ps*((4*pi)^3)))^(1/4);
% disp(Range);
