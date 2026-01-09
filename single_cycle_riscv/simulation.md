# Single-Cycle RISC-V Processor Design

This project implements a **Single-Cycle RISC-V processor** using **Verilog HDL**.  
The processor executes one instruction completely in a single clock cycle and follows the basic RISC-V integer instruction set architecture.

---

## Tools Used

- Cadence Xcelium – RTL simulation
- GTKWave – Waveform analysis
- Verilog HDL
- Linux Environment

---
## Simulation and Verification

The design was simulated using **Cadence Xcelium**.  
A Verilog testbench was used to generate clock and input signals.

### Using Cadence Xcelium
First source the cadence tool
```ver
csh
source /home/install/cshrc
xrun *.v -access +rwc -gui &
```
![alt](https://github.com/nilophertaj/risc-v_CPU/blob/776e569c2a0cc38d27c430984ef4f6e736230cf5/single_cycle_riscv/pictures/sim1.png)
![alt](https://github.com/nilophertaj/risc-v_CPU/blob/776e569c2a0cc38d27c430984ef4f6e736230cf5/single_cycle_riscv/pictures/sim2.png)

# waveform analysis
The following were verified using waveform analysis:
- Instruction fetch
- Register read and write
- ALU operations
- Memory access
- Write-back stage

---

## Waveform Results

The waveform confirms correct functionality of the processor.
![alt](https://github.com/nilophertaj/risc-v_CPU/blob/776e569c2a0cc38d27c430984ef4f6e736230cf5/single_cycle_riscv/pictures/wave.png)

Add these signals to check the process of single cycle risc-v

🔹 From top module riscv_single_cycle
- pc
- instr
- pc_next
🔹 From register file
- rd1
- rd2
- regwrite
🔹 From ALU
- alu_out
- zero
🔹 From memory
- memwrite
- memread
- mem_out
