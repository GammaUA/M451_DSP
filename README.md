The demo project of speech communicator on Nuvoton Cortex M4F M451LG6 microcontroller.

Realtime duplex analog audio recording and playing use 12 bit ADC and DAC with DMA support. 
AMR codec work in 4750 bps mode and convert 20 mS audio frame (160 short samles) to 12 bytes data block.
Voice active detector  selects only speech blocks for transmitting over UART.
The receiver provides alignment to the block boundary in speech pauses.
Ring buffer is used to compensate transport jitter.  The fill level of the buffer  is kept  by setting the palyback rate while the speech is continuous and changes the length of speech pauses.  To reduce the overall latency  the fill level is adjusted to optimum value for a given communicatin channel based on the measurement of transport jitter.
The code is compiled in Keil µVision 5.20 with installed  Nuvoton support package Nuvoton.NuMicro_DFP.1.2.0.pack and uses M451 Series_BSP_CMSIS_V3.01.001 drivers library. ANSI-C code for the floating-point Adaptive Multi-Rate  speech codec was adated for M4F platform mostly change all double types and math functions to float one.  
Program Size is: Code=73008 RO-data=113720 RW-data=420 ZI-data=26292  Perfomance is: encoding speech frame - 13 mS, silency - 4 mS, decoding - 2 mS.

FAE: Sergey Gayevsky, "Gamma UA", info@nuvoton.com.ua
