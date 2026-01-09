## The design contains :

- Program Counter (PC)
- Instruction Memory
- Register File
- Immediate Generator
- Control Unit
- ALU Control
- ALU
- Data Memory
- Top Processor Module
- Testbench

  # 1️⃣ PROGRAM COUNTER
 ```verilog
module pc (
    input clk,
    input reset,
    input [31:0] pc_next,
    output reg [31:0] pc
);

always @(posedge clk) begin
    if (reset)
        pc <= 32'b0;
    else
        pc <= pc_next;
end

endmodule
```
# 2️⃣ INSTRUCTION MEMORY (ROM)
```verilog
module instr_mem (
    input [31:0] addr,
    output reg [31:0] instr
);

always @(*) begin
    case (addr)
        32'h0: instr = 32'h00500093; // ADDI x1, x0, 5
        32'h4: instr = 32'h00600113; // ADDI x2, x0, 6
        32'h8: instr = 32'h002081b3; // ADD x3, x1, x2
        32'hC: instr = 32'h00302023; // SW x3, 0(x0)
        32'h10: instr = 32'h00002283; // LW x5, 0(x0)
        default: instr = 32'b0;
    endcase
end

endmodule

```
# 3️⃣ REGISTER FILE
```verilog
module reg_file (
    input clk,
    input regwrite,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] rd,
    input [31:0] wd,
    output [31:0] rd1,
    output [31:0] rd2
);

reg [31:0] regs [31:0];

assign rd1 = (rs1 == 0) ? 0 : regs[rs1];
assign rd2 = (rs2 == 0) ? 0 : regs[rs2];

always @(posedge clk) begin
    if (regwrite && rd != 0)
        regs[rd] <= wd;
end

endmodule

```
# 4️⃣ IMMEDIATE GENERATOR
```verilog
module imm_gen (
    input [31:0] instr,
    output reg [31:0] imm
);

always @(*) begin
    case (instr[6:0])
        7'b0010011, 7'b0000011: // ADDI, LW
            imm = {{20{instr[31]}}, instr[31:20]};
        7'b0100011: // SW
            imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
        7'b1100011: // BEQ
            imm = {{19{instr[31]}}, instr[31], instr[7],
                   instr[30:25], instr[11:8], 1'b0};
        default:
            imm = 0;
    endcase
end

endmodule

```
# 5️⃣ CONTROL UNIT
```verilog
module control_unit (
    input [6:0] opcode,
    output reg regwrite,
    output reg memread,
    output reg memwrite,
    output reg alusrc,
    output reg memtoreg,
    output reg branch,
    output reg [1:0] aluop
);

always @(*) begin
    case (opcode)
        7'b0110011: begin // R-type
            regwrite=1; alusrc=0; aluop=2'b10;
            memread=0; memwrite=0; memtoreg=0; branch=0;
        end
        7'b0010011: begin // ADDI
            regwrite=1; alusrc=1; aluop=2'b00;
            memread=0; memwrite=0; memtoreg=0; branch=0;
        end
        7'b0000011: begin // LW
            regwrite=1; alusrc=1; aluop=2'b00;
            memread=1; memwrite=0; memtoreg=1; branch=0;
        end
        7'b0100011: begin // SW
            regwrite=0; alusrc=1; aluop=2'b00;
            memread=0; memwrite=1; memtoreg=0; branch=0;
        end
        7'b1100011: begin // BEQ
            regwrite=0; alusrc=0; aluop=2'b01;
            memread=0; memwrite=0; memtoreg=0; branch=1;
        end
        default: begin
            regwrite=0; memread=0; memwrite=0;
            alusrc=0; memtoreg=0; branch=0; aluop=0;
        end
    endcase
end

endmodule

```
# 6️⃣ ALU CONTROL
```verilog
module alu_control (
    input [1:0] aluop,
    input [2:0] funct3,
    input funct7,
    output reg [3:0] alu_ctrl
);

always @(*) begin
    case (aluop)
        2'b00: alu_ctrl = 4'b0000; // ADD
        2'b01: alu_ctrl = 4'b0001; // SUB
        2'b10: begin
            if (funct3 == 3'b000 && funct7 == 0)
                alu_ctrl = 4'b0000; // ADD
            else
                alu_ctrl = 4'b0001; // SUB
        end
        default: alu_ctrl = 4'b0000;
    endcase
end

endmodule

```
# 7️⃣ ALU
```verilog
module alu (
    input [31:0] a,
    input [31:0] b,
    input [3:0] alu_ctrl,
    output reg [31:0] result,
    output zero
);

always @(*) begin
    case (alu_ctrl)
        4'b0000: result = a + b;
        4'b0001: result = a - b;
        default: result = 0;
    endcase
end

assign zero = (result == 0);

endmodule

```
# 8️⃣ DATA MEMORY
```verilog
module data_mem (
    input clk,
    input memread,
    input memwrite,
    input [31:0] addr,
    input [31:0] wd,
    output [31:0] rd
);

reg [31:0] mem [255:0];

assign rd = memread ? mem[addr[7:0]] : 0;

always @(posedge clk) begin
    if (memwrite)
        mem[addr[7:0]] <= wd;
end

endmodule

```
# 9️⃣ TOP MODULE (PROCESSOR)
This connects everything
```verilog
module riscv_single_cycle (
    input clk,
    input reset
);

wire [31:0] pc, pc_next, instr;
wire [31:0] rd1, rd2, imm, alu_out, mem_out, wb;
wire regwrite, memread, memwrite, alusrc, memtoreg, branch;
wire [1:0] aluop;
wire [3:0] alu_ctrl;
wire zero;

pc PC(clk, reset, pc_next, pc);
instr_mem IM(pc, instr);

control_unit CU(instr[6:0], regwrite, memread, memwrite,
                alusrc, memtoreg, branch, aluop);

reg_file RF(clk, regwrite,
             instr[19:15], instr[24:20], instr[11:7],
             wb, rd1, rd2);

imm_gen IG(instr, imm);

alu_control AC(aluop, instr[14:12], instr[30], alu_ctrl);

alu ALU(rd1, alusrc ? imm : rd2, alu_ctrl, alu_out, zero);

data_mem DM(clk, memread, memwrite, alu_out, rd2, mem_out);

assign wb = memtoreg ? mem_out : alu_out;
assign pc_next = (branch && zero) ? pc + imm : pc + 4;

endmodule

```
# 🔟 TESTBENCH
```verilog
module tb;

reg clk = 0;
reg reset = 1;

riscv_single_cycle DUT(clk, reset);

always #5 clk = ~clk;

initial begin
    #10 reset = 0;
    #200 $finish;
end

endmodule

```
