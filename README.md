## Background
This project is a final project for the laboratory section of a microprocessor systems course. In this course, the details of hardware-software interplay, memory management, input/output interfacing, and internal/external communication are discussed. The laboratory section endeavours to offer students the opportunity to apply concepts to the design and testing of an advanced embedded system. In the first laboratory demo, interrupts, timers, and General-Purpose Input-Output (GPIO) ports were explored to build a software-based frequency detector for a square wave input. This detector used an Interrupt Service Routine (ISR) triggered on the rising-edge of a general input pin. The ISR uses a timer to count the time elapsed between triggers, and compares this to the timer clock frequency to determine the input square wave frequency. This final project builds upon that to use an analog interface and a LM555N timer to produce the square wave input, rather than using a function generator.
## Problem Description
The objectives of this project can be broken down into the following subsystems:
1.	Measure the resistance of a potentiometer using an STM32F0 microcontroller 
1.	Use the STM32F0 microcontroller to output an analog voltage
1.	Build a circuit using an LM555 timer to output a square wave
1.	Build a circuit using a 4N35 Optocoupler to enable dynamic frequency adjustment of the timer circuit’s output signal, given the voltage in (2) as input
1.	Measure the frequency of the timer’s signal
1.	Output the signal frequency and potentiometer’s resistance on the display of the PBMCUSLK Project board.
