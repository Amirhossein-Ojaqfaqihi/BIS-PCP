# STM32 Bioimpedance & EMG Measurement PCB

This repository documents the design, testing, and validation of a mixed-signal PCB developed for biomedical signal acquisition.  
The system supports **bioimpedance spectroscopy (BIS)** and **electromyography (EMG)** measurements using an **STM32-based embedded platform**.

The project integrates signal generation, analog signal conditioning, synchronized data acquisition, and digital signal processing to analyze biological signals in real time.

---

# System Overview

The measurement system is built around an **STM32F3 microcontroller** that coordinates signal generation, acquisition, and processing.

The platform is designed to:

- Generate excitation signals for impedance measurements
- Acquire voltage and current signals simultaneously
- Capture multi-channel EMG signals
- Perform real-time signal processing
- Validate measurement accuracy using electrical models

The architecture combines embedded firmware with a dedicated **analog front-end PCB** for signal conditioning and amplification.

---

# Key Features

• Multisine signal generation using **STM32 DAC**  
• Analog front-end for current injection and voltage measurement  
• Simultaneous voltage/current acquisition using **ADC channels**  
• **4-channel EMG signal acquisition**  
• Adjustable gain control for EMG amplifiers  
• Real-time signal visualization via **UART serial output**  
• FFT-based impedance analysis using **CMSIS-DSP library**  
• Oscilloscope-based hardware debugging and signal tracing  

---

# Hardware Architecture

The PCB contains several functional blocks:

1. **Signal Generation Block**
   - Multisine waveform generated using STM32 DAC
   - DMA used for continuous waveform output

2. **Current Injection Circuit**
   - Injects excitation current into the measurement load

3. **Voltage Measurement Circuit**
   - Measures voltage response across the device under test

4. **Current Measurement Circuit**
   - Current calculated using shunt resistor measurements

5. **Analog Front-End Amplification**
   - Instrumentation amplifiers used to amplify biosignals

6. **EMG Sensor Channels**
   - Four independent EMG acquisition channels

7. **ADC Acquisition Interface**
   - ADC channels capture analog signals for processing

---

# Firmware Interaction

The firmware running on the STM32 microcontroller performs:

- DAC signal generation using DMA
- ADC sampling of voltage and current signals
- EMG signal acquisition
- UART transmission for real-time monitoring
- Digital signal processing using **CMSIS-DSP**

The **Fast Fourier Transform (FFT)** is used to convert time-domain signals to the frequency domain for impedance analysis.

---

# Hardware Testing and Validation

Several testing procedures were performed to verify system functionality.

### Signal Tracing

Signal propagation was traced across the PCB using an oscilloscope to verify:

- DAC waveform integrity
- amplifier stage functionality
- correct signal routing through the analog chain

### EMG Channel Testing

All four EMG channels were tested under two conditions:

- Idle muscle state
- Active muscle contraction

Each channel successfully detected muscle activity and produced measurable signal variations.

### Firmware Optimization

An inefficient firmware loop caused delays in signal visualization.  
After removing the redundant loop, real-time UART signal display became stable and synchronized with actual measurements.

### Hardware Debugging

During testing, the adjustable gain control circuit did not respond to software settings.

Signal tracing localized the issue to the **analog front-end pre-amplification stage**.

Further investigation revealed a failure in the **−2.5 V supply path used by the amplifier circuitry**.  
After restoring the correct supply voltage externally, the gain control function operated correctly.

---

# Bioimpedance Spectroscopy Testing

The BIS measurement system was evaluated using:

- known resistors
- capacitors
- electrical Cole impedance models

Measurements were compared against a reference impedance analyzer.

Results showed:

- **<3% error for resistive components**
- **<5% error for several Cole models**
- larger error for highly capacitive elements

These results demonstrate the system's capability to measure moderate impedance networks accurately.

---


