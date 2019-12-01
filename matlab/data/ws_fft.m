% Perform the FFT
ycoef = fft(sq_wav);

% Normalize all components by N
ycoef = ycoef/N;

% Multiple everything except the DC component by 2 to normalize by N/2
ycoef(2:N) = 2*ycoef(2:N);

% Only the first N/2 coefficients are significant 
%(the rest are mirror images)
spectrum = ycoef(1:N/2+1);

% Construct a frequency vector with delta f spacing
% The largest measurable frequency is (num_samples/2)-1
freq = fs*(0:N/2)/N;

% Plot the frequency versus magnitude
% Note: abs is used to get the magnitude
plot(freq,abs(spectrum));
title('single sided amplitude spectrum')
xlabel('f (Hz)')
ylabel('Magnitude');

% Create a new spectrum vector ignoring the DC value
ac_spectrum=abs(spectrum(min_idx:length(spectrum)));

% Create a new frequency vector ignoring the DC value
ac_freq = freq(min_idx:max_idx);

% Find the largest magnitude in the spectrum
max_value = max(ac_spectrum);

% Find the largest values coresponding index
max_index = find(ac_spectrum == max_value);

% The wheel speed is the frequency where the maximum occurs
wheel_speed = (ac_freq(max_index)/192)*2*pi
