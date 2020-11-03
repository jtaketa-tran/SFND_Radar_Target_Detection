% +---------------------------------------+
% |  Radar Target Generation & Detection  |                                 
% |  Jasmine Taketa-Tran, 5 October 2020  |
% +---------------------------------------+
clear all; close all;

%% Define Radar Specifications
% Configure the FMCW Waveform based on the System Requirements

fc= 77.0e9; % Operating Carrier Frequency (Hz)
c = 3.0e8; % Speed of Light (meters/sec)
lambda = c/fc; % Wavelength
Rmax = 200; % Maximum Detectable Range (meters)
Vmax = 70; % Maximum Velocity (meters/sec)
deltaR = 1.0; % Range Resolution in (meters)
deltaV = 3.0; % Velocity Resolution (meters/sec)

%% Compute FMCW Waveform Parameters
% Calculate the FMCW waveform parameters given the requirements specs

% Bandwidth of each chirp at the given range resolution:
Bsweep = c/(2*deltaR);

% Sweep Time for each chirp defined at 5.5x round trip time at max range:
Tsweep = 5.5*(Rmax*2/c);

% Slope of the chirp
Slope = Bsweep/Tsweep;

% The number of chirps in one sequence
Nd = 128; % #of doppler cells OR #of sent periods

% The number of samples on each chirp
Nr = 1024; % for length of time OR # of range cells


%% Model Signal Propagation for the Moving Target Simulation

% Set initial range and velocity of the target
targetRange = 110; %100; % Initial Distance to the Target (<= 200 m)
targetVelocity = 20; %-40; % Closing Velocity of Target (-70 to +70 m/s)

% Initialize vectors to store variable time histories
time = linspace(0,Nd*Tsweep,Nr*Nd); % time vector
Tx = zeros(1,length(time)); % transmit signal
Rx = zeros(1,length(time)); % receive signal
beatSig = zeros(1,length(time)); % beat frequency
range2Tgt = zeros(1,length(time)); % range to target
tau = zeros(1,length(time)); % time delay (trip time for the signal)

for tIdx = 1:length(time)
    
    % Displace the target based on the assumption of constant velocity
    range2Tgt(tIdx) = targetRange + (targetVelocity * time(tIdx));
    
    % Ensure that the targetRange does not exceed the maximum detectable
    % range of the radar. If it does, then flag it as undetectable.
    if range2Tgt(tIdx) > Rmax
        range2Tgt(tIdx) = NaN;
    end
    
    % Update the trip/delay time for the received signal
    tau(tIdx) = (range2Tgt(tIdx)*2)/c;
    
    % For each time stamp, update the transmit and receive signals
    % Receive signal is the time-delayed version of the transmit signal
    Tx(tIdx) = cos(2*pi*((fc*time(tIdx)) + ((Slope*(time(tIdx)^2))/2)));
    Rx(tIdx) = cos(2*pi*((fc*(time(tIdx)-tau(tIdx))) +...
        ((Slope*((time(tIdx)-tau(tIdx))^2))/2)));
end

% % Add 20% random noise to the signals
% Tx = Tx.*(1 + 0.2*randn(size(time)));
% Rx = Rx.*(1 + 0.2*randn(size(time)));

% For each displacement, determine the beat signal (frequency shift)
beatSig = Tx.*Rx; 

figure; subplot(3,1,1); plot(time(1:(Nr*2)),Tx(1:(Nr*2))); ylabel('transmit signal'); axis tight; grid on; grid minor; set(gca,'FontName','Cambria');
subplot(3,1,2); plot(time(1:(Nr*2)),Rx(1:(Nr*2))); ylabel('receive signal'); axis tight; grid on; grid minor; set(gca,'FontName','Cambria');
subplot(3,1,3); plot(time(1:(Nr*2)),beatSig(1:(Nr*2))); ylabel('beat signal'); axis tight; xlabel('time [sec]'); grid on; grid minor; set(gca,'FontName','Cambria');
subplot(3,1,1); hold on; title(sprintf('un-noised signal across 2 chirps on a target at initial range of %dm & %d m/s closing velocity',targetRange,targetVelocity));


%% Estimate the Range to Target
% Perform 1D FFT on the beat signal to estimate the range to target

% Reshape the mix signal into number of range samples (Nr) 
% and number of doppler samples (Nd) to form an Nr-by-Nd array
% Nr and Nd will also be used for the FFT sizes
beatMat = reshape(beatSig,[Nr Nd]);
range2TgtMat = reshape(range2Tgt,[Nr Nd]);

% Compute the normalized FFT of the beat signal along the range dimension
beat_fft = fft(beatMat,Nr)/Nr;

% Compute the amplitude of the normalized signal
beat_fft = abs(beat_fft);

% Compute the single-sided spectrum based on the even-valued signal length
beat_fft = beat_fft(1:Nr/2+1,:); % discard negative half of fft

figure('Name','Range from First FFT'); plot(beat_fft);
title('Amplitude Spectrum of the Beat Signal');
xlabel('range [m]'); ylabel('amplitude'); grid on; grid minor;
set(gca,'FontName','Cambria'); axis tight;
xticks(1:50:513); xticklabels(0:50:512);


%% Generate the Range Doppler Map (RDM)
% Perform a 2D FFT on the beat signal to generate a Range Doppler Map
% Perform CFAR processing on the output of the 2nd FFT to find the target
beatMat = reshape(beatSig,[Nr Nd]);

% 2D FFT using the FFT size for both dimensions
sig_fft2 = fft2(beatMat,Nr,Nd);

% Take just one side of signal in the range dimension
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift(sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM);

% Plot the output of 2D FFT as a function of range and velocity
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);

figure; surf(doppler_axis,range_axis,RDM);
xlabel('doppler'); ylabel('range'); 
title('Range Doppler Map');
set(gca,'FontName','Cambria');
grid on; grid minor; axis tight;

figure; contourf(doppler_axis,range_axis,RDM);
xlabel('doppler'); ylabel('range'); 
title('Range Doppler Map');
set(gca,'FontName','Cambria'); colorbar;
grid on; grid minor;


%% Perform CFAR Thresholding on the RDM to Detect the Target
% Note: For a Target at 100m, Gr = 6, Gd = 22 & tnr = 9
%       For a Target at 110m, Gr = 5, Gd = 23 & tnr = 10

% Select the Number of Training Cells
Tr = 20; % Number of Training Cells in the Range Dimension
Td = 10; % Number of Training Cells in the Doppler Dimension

% Select the Number of Guard Cells
Gr = 5;  % Number of Guard Cells in the Range Dimension
Gd = 23; % Number of Guard Cells in the Doppler Dimension

% Offset the threshold by SNR value in dB
tnr = 10; % Define the Threshold-to-Noise Ratio

% Slide the window filter across the Range Doppler Map
% Allow an outer image buffer for Guard & Training cells
for i = Tr+Gr+1:(Nr/2)-(Gr+Tr)
    for j = Td+Gd+1:Nd-(Gd+Td)
        
        % Step through the bins & grid surrounding the Cell Under Test
        noise_level = zeros(1,1);
        for p = i-(Tr+Gr):i+Tr+Gr
            for q = j-(Td+Gd):j+Td+Gd
                
                % Exclude the Guard Cells and the Cell Under Test
                if (abs(i-p)>Gr) || (abs(j-q)>Gd)
                    
                    % Convert the logarithmic dB value to linear power
                    % Then sum the noise across the Training Cells
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        
        % Compute threshold based on the noise and offset (tnr)
        numGridCells = ((2*Tr)+(2*Gr)+1)*((2*Td)+(2*Gd)+1);
        numGuardCells = Gr*Gd;
        numTrainCells = numGridCells - numGuardCells - 1;
        
        % Estimate the average noise across the Training Cells
        % Convert back to logarithmic dB after obtaining the average
        threshold = pow2db(noise_level/numTrainCells);
        threshold = threshold + tnr; % multiplying in linear space
        
        % Compare the Cell Under Test to the Detection Threshold
        CUT = RDM(i,j); % get signal within the Cell Under Test
        if CUT < threshold % if it is below threshold
            RDM(i,j) = 0; % then assign it a value of 0
%       else % if it is above threshold
%           RDM(i,j) = 1; % then assign it a value of 1
        end    
    end
end

% Suppress edges of the Range Doppler Map to account for the presence 
% of Training and Guard Cells surrounding the Cell Under Test 
RDM_thresholded = zeros(size(RDM));
RDM_thresholded(Tr+Gr+1:(Nr/2)-(Gr+Tr),Td+Gd+1:Nd-(Gd+Td)) = ...
    RDM(Tr+Gr+1:(Nr/2)-(Gr+Tr),Td+Gd+1:Nd-(Gd+Td));

% Plot the filtered RDM as a function of range and velocity
figure; surf(doppler_axis,range_axis,RDM_thresholded);
title('Range Doppler Map after CFAR Thresholding');
xlabel('doppler'); ylabel('range'); axis tight;
set(gca,'FontName','Cambria'); grid on; grid minor;

figure; contourf(doppler_axis,range_axis,RDM_thresholded);
title('Range Doppler Map after CFAR Thresholding');
xlabel('doppler'); ylabel('range'); axis tight;
set(gca,'FontName','Cambria'); colorbar;
grid on; grid minor;
