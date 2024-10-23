# R909-VFO-GPS
GPS corrected Si5351a VFO
There is a function of correction for Si5351a frequency error which is named F-COR.
I would like to elliminate handling for F-COR to rotate, to check the frequncy by the frequency counter, and get F-COR value.
We shall use PPS signal as a gate to count up Si5351a 25MHz signal. The difference between counted value and 25MHz will be a crrected value.

I adopted E108 GN02D GPS module and connected to R909-VFO.

I will introduce the sketch of the trial connection with GPS and R909-VFO refered "https://github.com/W3PM/GPS-Si5351-VFO-QEX-JUL-AUG-2015/Si5351_vfo_v5_3d.ino"
