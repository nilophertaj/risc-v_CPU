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

## Output Waveform using GTKWave
