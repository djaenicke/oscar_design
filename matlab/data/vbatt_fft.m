num_samples = length(v_batt);

% Control loop is run at 25ms rate
fs = 1/0.025; 

% Perform the FFT
ycoef = fft(v_batt - mean(v_batt));

% Normalize all components by N
ycoef = ycoef/num_samples;

% Multiply everything except the DC component by 2 to normalize by N/2
ycoef(2:num_samples) = 2*ycoef(2:num_samples);

% Only the first N/2 coefficients are significant 
%(the rest are mirror images)
spectrum = ycoef(1:num_samples/2+1);

% Construct a frequency vector with delta f spacing
% The largest measurable frequency is (num_samples/2)-1
freq = fs*(0:num_samples/2)/num_samples;

% Plot the frequency versus magnitude
% Note: abs is used to get the magnitude
plot(freq,abs(spectrum));
title('single sided amplitude spectrum')
xlabel('f (Hz)')
ylabel('Magnitude');

% Create a new spectrum vector ignoring the DC value
ac_spectrum=abs(spectrum(2:length(spectrum)));

% Create a new frequency vector ignoring the DC value
ac_freq = freq(2:length(freq));

% Find the largest magnitude in the spectrum
max_value = max(ac_spectrum);

% Find the largest values coresponding index
max_index = find(ac_spectrum == max_value);

% The frequency where the maximum occurs
max_noise_f = ac_freq(max_index);
