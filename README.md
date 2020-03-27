## Always-On Low-Power AFE Readout Subsystem


### Introduction

As a joint effort between the Analog and Mixed Signal and Digital Design Groups at IIS we
are exploring deep integration of analog precision sensing circuits with powerful digital processing capabilities. 
The combined workforce and expertise converge on the VivoSoC project, a mixed-signal multi-sensor parallel processing 
System-on-Chip (SoC) for miniaturized medical instrumentation. 
Over the recent years, several prototypes have been developed toward higher integration and better energy efficiency. 
Layout views and micrographs of the latest version can be found in the [IIS chip gallery](http://asic.ethz.ch/2019/Vivosoc3.142.html).

A detailed description of both an earlier version of the VivoSoC ASIC itself as well as
the surrounding mobile health platform alongside application examples is available in [1, 2].


The focus of this project is on the μDMA [4], the digital I/O subsystem of the chip which itself
is a subsystem of the parallel processing subsystem of VivoSoC, an adapted and optimized
implementation of the [Parallel Ultra-Low Power Processor (PULP) System](http://pulp.ethz.ch). PULP has been
used for a multitude of ASICs in a broad range of technology nodes and application fields; a
detailed description of a recent PULP chip can be found in [3].

### Context

Even if the μDMA [4] can free the cores of the system from tedious low-level control and management tasks such that 
they can either be utilized for more suitable workloads or clock gated, its core, the global interconnect and at 
least part of the L2 memory still needs to be active. In *VivoSoC* however, the data rates from the AFEs 
can be as low as tens or hundreds of samples per second, resulting in long idle times for the whole PULP system 
before enough data have been collected for a processing or even management task to become necessary.
Since samples can be generated at any times, the whole MCU system must still be powered at all times, resulting in both wasted 
active and static power that can dominate the overall power consumption of *VivoSoC* in low-bandwidth operations.

### Project

To overcome the aformentioned limits, this project aims to design and implement a digital subsystem for the PULP processing system 
of VivoSoC that can independantly handle the storage of incoming AFE data (up to a certain capcity) while the rest of the PULP 
system can be in a deep sleep or power gated.

The readout subsystem must be able to handle data streams from multiple AFEs concurrently and - when required - arbitrate between 
them to not require area- and power intensive multiported memory. Furthermore, each AFE can have multiple channels that can create 
data at arbitrary instants and with no fixed timely relation to each other. In the internal storage of the readout subsystem,
the data of each channel however should be placed as a single block of data to greatly ease further processing.

### Implementation Notes

This IP requires a package that configures the various parameters, starting from the number of AFEs all the way to defining
where in the data meta-information such as the channel id or flags reside. For your reference, an example package (`afe_parameters_pkg_example.sv`)
is provided. You have to adapt it to your needs and system and add it to your compilation flow.

For buffering the AFE samples, the IP is tailored towards the usage of SRAM. The module `sram_buffer` instantiates an `sram_wrapper_32b`, which is
technology-dependent and in turn instantiates the memory-cuts that are available in a specific technology. Therefore, you have to provide the module `sram_wrapper_32b` appropriately.

### References

* [1] P. Schönle et al. “A Multi-Sensor and Parallel Processing SoC for Miniaturized Medical
Instrumentation”, IEEE Journal of Solid-State Circuits (JSSC), Jul 2018.
* [2] F. Glaser et al. “Towards a Mobile Health Platform with Parallel Processing and Multi-Sensor
Capabilities”, Euromicro Conference on Digital System Design (DSD), Aug 2017.
* [3] A. Pullini et al. “Mr.Wolf: An Energy-Precision Scalable Parallel Ultra Low Power SoC for
IoT Edge Processing”, IEEE Journal of Solid-State Circuits (JSSC), Jul 2019.
* [4] A. Pullini et al. “μDMA: An Autonomous I/O Subsystem For IoT End-Nodes”, International
Workshop on Power and Timing Modeling, Optimization and Simulation (PATMOS), Sep 2017.
