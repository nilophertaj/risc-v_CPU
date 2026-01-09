# Single-Cycle RISC-V Processor (RV32I)

## Overview
This project implements a **32-bit single-cycle RISC-V (RV32I) processor** using **Verilog HDL**.  
All stages of instruction execution—fetch, decode, execute, memory access, and write-back—are completed within a **single clock cycle**.

The design focuses on understanding **CPU datapath fundamentals** and serves as a base for future upgrades to **pipelined architectures**.

---

## Objectives
- Design a complete **single-cycle CPU datapath**
- Implement control logic for **RV32I instructions**
- Understand instruction flow and register operations
- Verify functionality using RTL simulation

---

## Architecture
The processor follows a classic **single-cycle datapath**, where each instruction completes in one clock period.

### Instruction Flow
1. **Instruction Fetch (IF)**  
   The Program Counter (PC) fetches the instruction from Instruction Memory.

2. **Instruction Decode (ID)**  
   The Control Unit decodes the instruction and the Register File reads source registers.

3. **Execute (EX)**  
   The ALU performs arithmetic or logical operations and evaluates branch conditions.

4. **Memory Access (MEM)**  
   Data Memory is accessed for load and store instructions.

5. **Write Back (WB)**  
   The final result is written back to the Register File.

---

## Supported Instructions (Partial RV32I)
- **R-Type**: `add`, `sub`, `and`, `or`
- **I-Type**: `addi`, `lw`
- **S-Type**: `sw`
- **Branch**: `beq` (basic support)

---

## Hardware Modules

| Module | Description |
|------|------------|
| Program Counter | Holds the address of the current instruction |
| Instruction Memory | Stores program instructions |
| Control Unit | Generates control signals |
| Register File | 32 general-purpose registers |
| Immediate Generator | Extracts immediate values |
| ALU | Performs arithmetic and logic operations |
| Data Memory | Supports load and store operations |
| Write-Back MUX | Selects ALU or memory output |

---

## Project Structure
single_cycle_riscv/
├── rtl/
│   ├── pc.v
│   ├── instr_mem.v
│   ├── regfile.v
│   ├── control.v
│   ├── imm_gen.v
│   ├── alu.v
│   ├── alu_control.v
│   ├── data_mem.v
│   └── top.v
├── tb/
    └── tb_top.v


## Results
- Correct instruction execution observed
- Proper register write-back verified
- Program Counter updates every clock cycle
- Load and store operations function as expected

## Limitations
- Single-cycle design limits maximum clock frequency
- No pipeline hazard handling
- Performance not optimized for complex programs

## Future Enhancements
- Upgrade to a 5-stage pipelined RISC-V processor
- Implement forwarding and hazard detection
- Support full RV32I instruction set
- Execute compiled C programs
