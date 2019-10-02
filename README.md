# UdacityRadar
Udacity Nanodegree Self Driving Car/ Radar proj


# Project Overview
![ProjectOverview](https://user-images.githubusercontent.com/51704629/66046436-afc42800-e560-11e9-8b32-caa1f222c5f6.png)

* Configure the FMCW waveform based on the system requirements.
* Define the range and velocity of target and simulate its displacement.
* For the same simulation loop process the transmit and receive signal to determine the beat signal
* Perform Range FFT on the received signal to determine the Range
* Towards the end, perform the CFAR processing on the output of 2nd FFT to display the target.

## 1. Radar Specifications
* Frequency of operation = 77 GHz
* Max Range = 200 m
* Range Resolution = 1 m
* Max Velocity = 70 m/s

```matlab
fc= 77e9;
range_max_m = 200;
d_res_m = 1;
velocity_max_ms = 70;
```

## 2. user Defined Range and Velocity of target
* define the target's initial posotion and velocity
* Note: Velocity remains constant

```matlab
R = 50; % Traget Initial Range
v = -30; % Target Velocity
```

## 3. FMCW Waveform Generation
* Design the FMCW waveform by giving the specs of each of its parameters.
* Calculate the Bandwidth (Bsweep), Chirp Time (Tchirp) and Slope (slope) of the FMCW chirp using the requirements above.
* Operating carrier frequency of Radar

```matlab
Tchirp = 5.5 * (range_max_m * 2)/c_ms;
Bsweep = c_ms/(2*d_res_m);
slope = Bsweep/Tchirp;
```

* The number of chirps in one sequence.
* Its ideal to have 2^value for the ease of running the FFT for Doppler Estimation.

```matlab
Nd=128; 
```

* The number of samples on each chirp.

```matlab
Nr=1024;
```

* Timestamp for running the displacement scenario for every sample on each chirp

```matlab
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples
```

* Creating the vectors for Tx, Rx and mix based on the total samples input.

```matlab
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal
```

* Similar vectors for range_covered and time delay.

```matlab
r_t=zeros(1,length(t));
td=zeros(1,length(t));
```

## 4. Signal generation and Moving Target simulation

```matlab
for i=1:length(t)    
    
    % For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = R + v * t(i);
    td(i) = 2 * r_t(i) / c_ms;
    
    % For each time sample we need update the transmitted and Received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + (slope*t(i)^2/2)));
    Rx(i) = cos(2*pi*(fc * (t(i) - td(i)) + 0.5 * slope * (t(i) - td(i))^2));
    
    % Now by mixing the Transmit and Receive generate the beat signal This is done by element
    % wise matrix multiplication of Transmit and Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end
```

