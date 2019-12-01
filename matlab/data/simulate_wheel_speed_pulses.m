%% Tunable Parameters
pulses_per_rev = 192; % Hall effect pulses per wheel revolution
control_loop_rate = 0.025;

fs = 2500; %Hz
N = 512;

min_wheel_speed = 1; %(rad/s)
max_wheel_speed = 35;  %(rad/s)

%% Input
wheel_speed = 25; %(rad/s)

%% Computations
max_meas_speed = fs/2/192*2*pi;   %(rad/s)
bin_width = fs/N/192*2*pi;         %(rad/s)

freq = wheel_speed*pulses_per_rev; %(rad/s)
offset = 0.5;
amp = 0.5;
duty = 50;

t=0:1/fs:control_loop_rate-(1/fs);

sq_wav = offset + (amp*square(freq.*t,duty));
plot(t, sq_wav)

if N > length(sq_wav)
    sq_wav = [sq_wav zeros(1,N-length(sq_wav))];
    min_idx = round(min_wheel_speed/bin_width);
    max_idx = round(max_wheel_speed/bin_width);
else
    min_idx = 2;
    max_idx = N/2;
end

ws_fft
