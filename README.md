# FMCW Radar Design for Adaptive Cruise Control

## Introduction

This project is based on the design and simulation of an **FMCW (Frequency Modulated Continuous Wave) Radar** for **Adaptive Cruise Control (ACC)** applications. The main purpose of the project is to detect a vehicle in front of the radar and estimate its distance and relative velocity.

FMCW radar is commonly used in automotive applications because it can provide both range and velocity information. In this project, an FMCW chirp signal is generated and transmitted towards a target. The reflected signal is received back at the radar after a certain delay. If the target is moving, the received signal also contains a Doppler shift.

The transmitted and received signals are compared to obtain the beat signal. The beat signal is then processed using FFT to obtain the required frequency components. These frequency components are used to estimate the target range and relative velocity.

## Objectives

* To generate an FMCW radar chirp signal.
* To simulate the transmitted and received radar signals.
* To calculate the beat signal.
* To estimate the range of the target.
* To estimate the relative velocity of the target.
* To obtain the Range-Doppler Map.
* To understand the use of FMCW radar in Adaptive Cruise Control.

## FMCW Radar

In FMCW radar, the frequency of the transmitted signal changes continuously with time. A linear frequency-modulated chirp is used in the simulation.

When the signal is reflected from a target, the received signal is delayed because of the distance between the radar and the target. The movement of the target also produces a Doppler frequency shift.

The transmitted and received signals are mixed to obtain the beat signal. The beat frequency is related to the target range, while the Doppler frequency is related to the relative velocity.

## Methodology

The project is carried out in the following steps:

1. Define the radar and target parameters.
2. Generate the FMCW chirp signal.
3. Generate the received signal considering target range and velocity.
4. Mix the transmitted and received signals to obtain the beat signal.
5. Apply FFT for range estimation.
6. Perform Doppler processing for velocity estimation.
7. Generate the Range-Doppler Map.
8. Analyze the detected target parameters.

## Parameters

The simulation can be performed by defining parameters such as:

* Carrier frequency
* Bandwidth
* Chirp duration
* Sampling frequency
* Target distance
* Target velocity
* Number of chirps

These parameters can be changed according to the required simulation scenario.

## Range Estimation

The target range is estimated from the beat frequency obtained after mixing the transmitted and received signals. FFT is used to find the dominant frequency component, which is then converted into the corresponding target distance.

## Velocity Estimation

For a moving target, the received signal contains a Doppler frequency shift. Multiple chirps are used for Doppler processing, and the Doppler frequency is used to estimate the relative velocity of the target.

## Range-Doppler Map

The Range-Doppler Map gives a two-dimensional representation of the detected target. It shows the relationship between target range and relative velocity and helps in identifying the location and movement of the target.

## Adaptive Cruise Control

The estimated range and relative velocity can be used in an Adaptive Cruise Control system. The radar continuously observes the vehicle ahead and provides information about its distance and relative motion.

Based on this information, an ACC controller can adjust the speed of the host vehicle to maintain a suitable distance from the vehicle ahead.

## Software Used

* MATLAB
* Signal Processing
* FFT
* FMCW Radar Simulation

## Results

The simulation provides results such as:

* FMCW transmitted signal
* Received signal
* Beat signal
* Range estimation
* Doppler/velocity estimation
* Range-Doppler Map

The generated plots and results are included in the `Results` folder.

## Future Scope

The project can be further extended by including:

* Multiple target detection
* CFAR detection
* Noise and clutter
* Target tracking
* MIMO radar
* Real-time implementation
* Complete ACC controller

## Author
Shubham
Electronics and Communication Engineering
