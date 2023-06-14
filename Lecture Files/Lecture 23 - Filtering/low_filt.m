function filt_data = low_filt(Fs,N,Fc,data)

%%%%%%%%%%%%%%%%%%%%%%
%This function low-passfilters the EMG data to reduce the motion artifact
%Usage: filt_data = low_filt(Fs,N,Fc,data)
%Fs - sampling frequency
%N - Filter order
%Fc - cutoff frequency
%data - data to be filtered
%
%
%%%%%%%%%%%%%%%%%%%%%%%

[B,A] = butter(N, Fc/(Fs/2),'low');                                         % Butterworth filter design

for i=1:size(data,2)
%    filt_data(:,i) = filtfilt(B,A, data(:,i))                              % For non-causal / bidirectional 0-phase filtering 
    filt_data(:,i) = filter(B,A, data(:,i));                                % For causal filtering
end