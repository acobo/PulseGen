# PulseGen
STM32+arduino-based pulse generator for LIBS experiments, with precise delays using DMA.

It creates a vector of values for port GPIOA  (A0..A15) and the pins are updated by a DMA transfer at a precise and repetitive rate (97ns steps on a bluepill board). Due to RAM constraints (20KB), the maximum number of values is 2000, that allows a burst of pulses up to 190 microseconds. I guess this can be expanded in other STM32s with more RAM available. This burst is repeated at a chosen rate (10 Hz)

This is inteded to drive a double-pulse Nd:YAG laser and an spectrometer for Laser-Induced Breakdown Spectroscopy (LIBS) experiments, which require a precise and programmable delay between the pulse of the lamp (A0) and the q-switch triggers (A1 & A2) and the spectrometer (A3). Besides, the spectrometer pulse can be programmed with different alternate delays (for every other pulse) so the plasma emission for consecutive laser shots is captured with a differential delay. This allows to substract both spectra and get a spectrum with an effective shorter time window (gated-like capture with nongated espectrometers).

The actual jitter has been checked with an oscilloscope and it is about +/-50ns , although I believe is due to noise in the edges of pulses affecting the trigger of the oscilloscope.

code for DMA transfer from:  https://www.stm32duino.com/viewtopic.php?t=1042
