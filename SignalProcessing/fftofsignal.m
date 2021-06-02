x=A(66:225,7);
Fs = 99;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = length(x);             % Length of signal
t = (0:L-1)*T;        % Time vector
Y = fft(x);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
