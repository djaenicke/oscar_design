%% Tunable Parameters
pulses_per_rev = 192; % Hall effect pulses per wheel revolution
control_loop_rate = 0.025;

fs = 2500; %Hz
N = 4096;

%% Input
wheel_speed = 15; %(rad/s)

%% Computations
max_wheel_speed = fs/2/192*2*pi;   %(rad/s)
bin_width = fs/N/192*2*pi;         %(rad/s)

freq = wheel_speed*pulses_per_rev; %(rad/s)
offset = 1;
amp = 1;
duty = 50;

t=0:1/fs:control_loop_rate-(1/fs);

sq_wav = offset + (amp*square(freq.*t,duty));
plot(t, sq_wav)

if N > length(sq_wav)
    sq_wav = [sq_wav zeros(1,N-length(sq_wav))];
    min_f = 32; % Hz
    min_idx = round(min_f/(fs/N));
else
    min_idx = 2;
end

ws_fft
