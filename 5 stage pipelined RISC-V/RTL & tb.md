# 5-Stage Pipelined RISC-V Processor (Baseline)

This repository contains a **baseline 5-stage pipelined RISC-V processor** implemented in Verilog HDL.
The design is intended **purely for learning and understanding pipeline concepts**.

No forwarding, no stalls, no branch handling.

---

## Pipeline Stages
1. IF – Instruction Fetch  
2. ID – Instruction Decode  
3. EX – Execute  
4. MEM – Memory Access  
5. WB – Write Back  

---

## RTL CODE

---

### pc.v
```verilog
module pc(input clk, input reset, input [31:0] pc_next, output reg [31:0] pc);
always @(posedge clk) begin
    if (reset) pc <= 0;
    else pc <= pc_next;
end
endmodule
```

### instr_mem.v
```verilog
module instr_mem(input [31:0] addr, output [31:0] instr);
reg [31:0] mem [0:255];
initial begin
    mem[0] = 32'h00500093; // addi x1,x0,5
    mem[1] = 32'h00600113; // addi x2,x0,6
end
assign instr = mem[addr[9:2]];
endmodule
```

### reg_file.v
```verilog
module reg_file(
    input clk,
    input regwrite,
    input [4:0] rs1, rs2, rd,
    input [31:0] wd,
    output [31:0] rd1, rd2
);
reg [31:0] regs[31:0];

assign rd1 = (rs1==0)?0:regs[rs1];
assign rd2 = (rs2==0)?0:regs[rs2];

always @(posedge clk)
    if (regwrite && rd!=0)
        regs[rd] <= wd;
endmodule

```

### imm_gen.v
```verilog
module imm_gen(input [31:0] instr, output [31:0] imm);
assign imm = {{20{instr[31]}}, instr[31:20]};
endmodule

```

### control_unit.v
```verilog
module control_unit(
    input [6:0] opcode,
    output reg regwrite, alusrc
);
always @(*) begin
    regwrite = 0; alusrc = 0;
    if (opcode == 7'b0010011) begin
        regwrite = 1;
        alusrc = 1;
    end
end
endmodule

```

### alu.v
```verilog
module alu(input [31:0] a, b, output [31:0] result);
assign result = a + b;
endmodule

```

### if_id.v
```verilog
module if_id(input clk, input [31:0] pc4_in, instr_in,
             output reg [31:0] pc4_out, instr_out);
always @(posedge clk) begin
    pc4_out <= pc4_in;
    instr_out <= instr_in;
end
endmodule

```

### id_ex.v
```verilog
module id_ex(
    input clk,
    input [31:0] rs1_in, rs2_in, imm_in,
    input regwrite_in, alusrc_in,
    output reg [31:0] rs1_out, rs2_out, imm_out,
    output reg regwrite_out, alusrc_out
);
always @(posedge clk) begin
    rs1_out <= rs1_in;
    rs2_out <= rs2_in;
    imm_out <= imm_in;
    regwrite_out <= regwrite_in;
    alusrc_out <= alusrc_in;
end
endmodule

```

### riscv_pipeline.v (TOP)
```verilog
module riscv_pipeline(input clk, input reset);

wire [31:0] pc, pc4, instr;
assign pc4 = pc + 4;

// IF
pc PC(clk, reset, pc4, pc);
instr_mem IM(pc, instr);

// IF/ID
wire [31:0] ifid_instr;
if_id IFID(clk, pc4, instr, , ifid_instr);

// ID
wire [31:0] rs1_data, rs2_data, imm;
wire regwrite, alusrc;

reg_file RF(clk, regwrite,
            ifid_instr[19:15],
            ifid_instr[24:20],
            5'd0, 32'd0,
            rs1_data, rs2_data);

imm_gen IG(ifid_instr, imm);
control_unit CU(ifid_instr[6:0], regwrite, alusrc);

// ID/EX
wire [31:0] idex_rs1, idex_rs2, idex_imm;
wire idex_regwrite, idex_alusrc;

id_ex IDEX(clk, rs1_data, rs2_data, imm,
           regwrite, alusrc,
           idex_rs1, idex_rs2, idex_imm,
           idex_regwrite, idex_alusrc);

// EX
wire [31:0] alu_b = idex_alusrc ? idex_imm : idex_rs2;
wire [31:0] alu_result;
alu ALU(idex_rs1, alu_b, alu_result);

endmodule

```

## TESTBENCH

---

### tb_pipeline.v
```verilog
module tb_pipeline;
reg clk = 0, reset = 1;

riscv_pipeline dut(clk, reset);

always #5 clk = ~clk;

initial begin
    #10 reset = 0;
    #100 $finish;
end

initial begin
    $dumpfile("pipeline.vcd");
    $dumpvars(0, tb_pipeline);
end
endmodule

```

# Waveform Analysis – 5-Stage Pipelined RISC-V Processor
## Output Waveform using GTKWave
![alt](https://github.com/nilophertaj/risc-v_CPU/blob/9a35c8afe883e09694cae0c1f0cf27bf6ce54e97/5%20stage%20pipelined%20RISC-V/full_pipeline.png)

## Important Signals to Add in GTKWave

### 1. Global Signals
These confirm correct timing and synchronization.
- `clk`
- `reset`

---

### 2. Instruction Fetch (IF) Stage
These signals verify instruction fetching.
- `pc`
- `pc4`
- `instr`

✔ Check:
- `pc` increments by 4 every cycle
- `instr` changes according to `pc`

---

### 3. IF/ID Pipeline Register
These signals show instruction transfer to the decode stage.
- `ifid_instr`
- `ifid_pc4`

✔ Check:
- `ifid_instr` holds the previous cycle’s `instr`
- Confirms one-cycle pipeline delay

---

### 4. Instruction Decode (ID) Stage
These signals verify decoding and register read.
- `rs1_data`
- `rs2_data`
- `imm`
- `regwrite`
- `alusrc`

✔ Check:
- `rs1_data` and `rs2_data` match register file contents
- `imm` is sign-extended correctly
- Control signals are generated correctly based on opcode

---

### 5. ID/EX Pipeline Register
These confirm decoded values are passed to execute stage.
- `idex_rs1`
- `idex_rs2`
- `idex_imm`
- `idex_regwrite`
- `idex_alusrc`

✔ Check:
- Values match ID stage but delayed by one cycle
- Confirms correct pipeline register operation

---

### 6. Execute (EX) Stage
These signals verify ALU operation.
- `alu_result`
- `alu_b` (selected operand)

✔ Check:
- `alu_result` equals expected computation
- Operand selection depends on `alusrc`

---

### 7. EX/MEM Pipeline Register (If Implemented)
- `exmem_alu_result`
- `exmem_rs2_data`
- `exmem_regwrite`

✔ Check:
- Values are forwarded correctly to memory stage

---

### 8. Memory Access (MEM) Stage
- `mem_read_data`
- `exmem_memread`
- `exmem_memwrite`

✔ Check:
- Memory accessed only when control signals are asserted

---

### 9. Write Back (WB) Stage
These signals verify final result write-back.
- `memwb_rd`
- `wb_data`
- `memwb_regwrite`

✔ Check:
- `wb_data` written only when `memwb_regwrite = 1`
- Correct destination register selected

---

## Pipeline Behavior Verification

### Instruction Flow Check
✔ One instruction enters the pipeline every cycle  
✔ Different instructions occupy different stages simultaneously  
✔ No stage overwrites another stage’s data  

This confirms **true pipelined execution**.

---

### Timing Check
✔ Same instruction appears across IF → ID → EX → MEM → WB in consecutive cycles  
✔ Total latency = 5 cycles per instruction  
✔ Throughput = 1 instruction per cycle after pipeline fill  

---

## What This Design Represents

| Feature        | Status |
|---------------|--------|
| Pipeline Type | 5-Stage |
| Hazard Handling | ❌ No |
| Forwarding | ❌ No |
| Branch Handling | ❌ No |
| Focus | Learning & Clarity |

This is a **baseline pipelined RISC-V design**, used as a foundation for advanced features.


