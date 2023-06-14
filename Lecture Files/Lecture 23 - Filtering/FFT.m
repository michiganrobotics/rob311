function [a, f, ao, fo] = FFT(Fs, X, plotting)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Single sided amplitude spectrum function file
% Usage: [a, f] = FFT(Fs, X)
% Inputs: Fs - Sample Rate, X - Signal
% Outputs:  f - frequency axis (Hz), a - Magnitude
%           ao - original a, fo - original f
%
% E Rouse 10-1-09
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plotting = 0; % Toggle to plot

L = length(X);
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(X,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2);
a = 2*abs(Y(1:NFFT/2))*(1/Fs);
ao = Y;
fo = Fs/2*linspace(0,1,NFFT/2);


if plotting == 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot single-sided amplitude spectrum.
figure
plot(f,2*abs(Y(1:NFFT/2))) 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end