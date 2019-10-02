# UdacityRadar
Udacity Nanodegree Self Driving Car/ Radar proj


# Project Overview
![ProjectOverview](https://user-images.githubusercontent.com/51704629/66046436-afc42800-e560-11e9-8b32-caa1f222c5f6.png)

* Configure the FMCW waveform based on the system requirements.
* Define the range and velocity of target and simulate its displacement.
* For the same simulation loop process the transmit and receive signal to determine the beat signal
* Perform Range FFT on the received signal to determine the Range
* Towards the end, perform the CFAR processing on the output of 2nd FFT to display the target.

1. Radar Specifications
* Frequency of operation = 77 GHz
* Max Range = 200 m
* Range Resolution = 1 m
* Max Velocity = 70 m/s

```
matlab
fc= 77e9;
range_max_m = 200;
d_res_m = 1;
velocity_max_ms = 70;
```
