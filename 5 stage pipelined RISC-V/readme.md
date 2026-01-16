# 5-Stage Pipelined RISC-V Processor (Baseline Design)

## Overview
This project implements a **baseline 5-stage pipelined RISC-V processor** using Verilog HDL.  
The design follows the standard RISC-V pipeline stages: **Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), and Write Back (WB)**.  
The objective of this project is to understand the **instruction flow, pipeline registers, and control signal propagation** in a pipelined CPU architecture.

This is a **baseline design**, meaning no forwarding, stalling, or branch hazard handling is implemented.

---

## Pipeline Stages

### 1. Instruction Fetch (IF)
- Program Counter (PC) generates instruction address
- Instruction memory fetches the instruction
- PC is incremented by 4

### 2. Instruction Decode (ID)
- Instruction is decoded
- Register file reads source registers
- Immediate value is generated
- Control signals are generated

### 3. Execute (EX)
- ALU performs arithmetic or logical operations
- Operand selection using control signals

### 4. Memory Access (MEM)
- Data memory read/write operations
- ALU result used as memory address

### 5. Write Back (WB)
- Final result written back to register file
- Controlled by RegWrite signal

---

## Design Type
- **Architecture**: 5-stage pipelined RISC-V
- **Pipeline Type**: Baseline (no hazard handling)
- **Instruction Flow**: One instruction per cycle after pipeline fill
- **Optimization**: None (focused on clarity and learning)

---

## Tools Used
- **HDL**: Verilog
- **Simulator**: Cadence Xcelium / Icarus Verilog
- **Waveform Viewer**: GTKWave

---

## Project Structure
```verilog
riscv_5stage_pipeline/
├── rtl/
│ ├── pc.v
│ ├── instr_mem.v
│ ├── reg_file.v
│ ├── imm_gen.v
│ ├── alu.v
│ ├── control_unit.v
│ ├── if_id.v
│ ├── id_ex.v
│ ├── ex_mem.v
│ ├── mem_wb.v
│ └── riscv_pipeline.v
├── tb/
│ └── tb_pipeline.v
└── README.md
```
