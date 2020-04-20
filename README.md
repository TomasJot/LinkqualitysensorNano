# LinkqualitysensorNano
Link quality sensor for Frsky on Arduino Nano based on RealTadango LQ sensor. Custom name 5105. SBUS out connected through inverter to RX0, S-port - to D3.
Lost frame detection based on tringle wave generation on one  or 2 channels.
Failsafe duration is displayed on the same 5105 custom sensor every 10s for 1s in 1/10 of a second.

@define's : 
DISPLAYFS - maxim number of detected failsafes to display
LOWCHAN - channel with triangle wave generation (one in 8ch mode, lower (1-8) channel in 16 channels mode.
HIGHCHAN - upper channel (9-16) in 16 channels mode
CH16 - enable 16 channnel mode

Triangle generation in OpenTX:
L08 a<x 0
Mix: CH8 L08 Weight(+100) Delay (u1:d1) Slow (u2:d2:)
