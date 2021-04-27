// Your SingleCycle RISC-V code
`include "memory.v"

module CHIP(clk,
            rst_n,
            // for mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // for mem_I
            mem_addr_I,
            mem_rdata_I
    );

    input         clk, rst_n ;
    // for mem_D
    output        mem_wen_D  ;  // mem_wen_D is high, CHIP writes data to D-mem; else, CHIP reads data from D-mem
    output [31:0] mem_addr_D ;  // the specific address to fetch/store data 
    output [31:0] mem_wdata_D;  // data writing to D-mem 
    input  [31:0] mem_rdata_D;  // data reading from D-mem
    // for mem_I
    output [31:0] mem_addr_I ;  // the fetching address of next instruction
    input  [31:0] mem_rdata_I;  // instruction reading from I-mem
    wire ctrlSignal [0:8];
    // 0: Jalr, 1: Jal, 2: Branch, 3: MemRead, 4:MemtoReg, 5: MemWrite
    // 6: ALUSrc 7: RegWrite, 8: Jin
    wire [1:0] ALUOp; 
    wire [31:0] rData1,rData2,Imm,ALUData2,MemAddr,wData,wDataFin;
    wire [31:0] AddrNext,AddrJalPre,AddrJalrPre,AddrFin;
    wire [3:0] alu_ctrl;
    wire zero,BandZ,BandZorJ;
    reg [31:0] PC;
    reg [31:0] PC_next;
    // assign
    assign ALUData2 = (ctrlSignal[6])?Imm:rData2;
    assign wData = (ctrlSignal[8])?{AddrNext}:wDataFin;
    assign mem_wen_D = ctrlSignal[5];
    assign mem_addr_D = MemAddr;
    assign mem_wdata_D = {rData2[7:0],rData2[15:8],rData2[23:16],rData2[31:24]};
    assign wDataFin = (ctrlSignal[4])?{mem_rdata_D[7:0],mem_rdata_D[15:8],mem_rdata_D[23:16],mem_rdata_D[31:24]}:MemAddr;
    assign mem_addr_I = PC;
    assign AddrNext = PC[31:0] + 32'd4;       // can modify to 30 bits if necessary
    assign AddrJalPre = PC + Imm;
    // and bandz(BandZ,zero,ctrlSignal[2]);
    // or bandzorj(BandZorJ,BandZ,ctrlSignal[1]);
    // assign AddrSecStage = (BandZorJ == 1)?AddrJalPre:AddrNext;
    assign AddrJalrPre = Imm + rData1;
    // assign AddrFin = (ctrlSignal[0] == 1)?AddrJalrPre:AddrSecStage;
    assign AddrFin = (ctrlSignal[0])?AddrJalrPre:((zero&ctrlSignal[2])|ctrlSignal[1])?AddrJalPre:AddrNext;
    // combinatial
    always @(*) begin
        PC_next = AddrFin;
    end
    // sequential
    always @(posedge clk) begin
        PC <= PC_next;
        if (!rst_n) PC <= 32'b0;
    end
    
    // module     
    
    Reg_File RF(.rs1({{mem_rdata_I[11:8]},mem_rdata_I[23]}),
                .rs2({mem_rdata_I[0],{mem_rdata_I[15:12]}}),
                .wData(wData),
                .rd({{mem_rdata_I[19:16]},mem_rdata_I[31]}),
                .rData1(rData1),
                .rData2(rData2),
                .Reg_write(ctrlSignal[7]),
                .clk(clk),
                .rst_n(rst_n)
                );

    ALU alu(.rd1(rData1),
            .rd2(ALUData2),
            .out1(MemAddr),
            .ALU_Ctrl(alu_ctrl),
            .Zero(zero)
            );
    
    ALU_Ctrl aluCtrl(   .ALUOp(ALUOp),
                        .ins({{mem_rdata_I[6]},{mem_rdata_I[22:20]}}),
                        .ctrl(alu_ctrl)
                    );
    
    ImmGen immGen(  .input_reg({{mem_rdata_I[7:0]},{mem_rdata_I[15:8]},{mem_rdata_I[23:16]},{mem_rdata_I[31:24]}}),
                    .output_reg(Imm)
                    );

    CTRL ctrl(  .ins({mem_rdata_I[30:24]}),
                .Jalr(ctrlSignal[0]),
                .Jal(ctrlSignal[1]),
                .Branch(ctrlSignal[2]),
                .MemRead(ctrlSignal[3]),
                .MemtoReg(ctrlSignal[4]),
                .ALUOp(ALUOp),
                .MemWrite(ctrlSignal[5]),
                .ALUSrc(ctrlSignal[6]),
                .RegWrite(ctrlSignal[7]),
                .Jin(ctrlSignal[8])
                );
endmodule

// complete, clk is needed
// wData: data write in
module Reg_File( rs1,
                rs2,
                wData,
                rd,
                rData1,
                rData2,
                Reg_write,
                clk,
                rst_n
    );

    input [4:0] rs1,rs2,rd;
    input [31:0] wData;
    input Reg_write,clk,rst_n;
    output [31:0] rData1,rData2;
    reg [31:0] Rreg [0:31];
    reg [31:0] Wreg [0:31];
    integer i ;
    assign rData1 = Rreg[rs1];
    assign rData2 = Rreg[rs2];
    always @(*) begin
      for(i=0;i<32;i=i+1)begin
        Wreg[i] = Rreg[i];  
      end
      if (Reg_write) Wreg[rd] = wData;
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            for (i=0;i<32;i=i+1) begin
                Rreg[i] <= 32'b0;
            end
        end
        Rreg[0] <=32'b0;
        for(i=1;i<32;i=i+1)begin
            Rreg[i] <= Wreg[i];
        end
    end

endmodule

// maybe is complete
module ALU( rd1,
            rd2,
            out1,
            ALU_Ctrl,
            Zero
    );

    input [31:0] rd1,rd2;
    input [3:0] ALU_Ctrl;
    output Zero;
    output reg [31:0] out1;
    wire [31:0] andWire, orWire, addWire, subWire, sltWire;
    assign andWire = rd1 & rd2;
    assign orWire = rd1 | rd2;
    assign addWire = $signed(rd1) + $signed(rd2);
    assign subWire = $signed(rd1) - $signed(rd2);
    assign sltWire = (subWire[31]|Zero)?32'b1:32'b0; 
    assign Zero = ($signed(rd1) == $signed(rd2))?1'b1:1'b0;
    always @(*) begin
        case(ALU_Ctrl)
            4'b0000: out1 = andWire;
            4'b0001: out1 = orWire;
            4'b0010: out1 = addWire;
            4'b0110: out1 = subWire;
            4'b0111: out1 = sltWire;
            default: out1 = 32'b0;
        endcase
    end

endmodule

// need to check ALUOp
// need to modify ALUOP
// here read in opcode named as ins\
// need to modify Jal|Jalr
// ALUSrc??
module CTRL(ins,
            Jalr,
            Jal,
            Branch,
            MemRead,
            MemtoReg,
            ALUOp,
            MemWrite,
            ALUSrc,
            RegWrite,
            Jin
    );

    input [6:0] ins;
    output reg Jin,Jal,Jalr,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
    output reg [1:0] ALUOp;
    
    always @(*)begin
        {Jal,Jalr,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jin} = 9'b0;
        ALUOp = 2'b0;
        case(ins) 
            7'b0110011: begin
                        ALUOp = ins[4:3];
                        RegWrite = 1'b1;
                        end
            7'b0100011: begin
                        MemWrite = 1'b1;     // StoreWord
                        ALUSrc = 1'b1;
                        end
            7'b0000011: begin
                        MemRead = 1'b1;      // LoadWord
                        MemtoReg = 1'b1;
                        ALUSrc = 1'b1;
                        RegWrite = 1'b1;
                        end
            7'b1101111: begin
                        Jal = 1'b1;
                        Jin = 1'b1;
                        RegWrite = 1'b1;
                        ALUSrc = 1'b1;
                        end
            7'b1100111: begin
                        Jalr = 1'b1;
                        Jin = 1'b1;
                        RegWrite = 1'b1;
                        end
            7'b1100011: begin               // beq
                        ALUOp = ins[4:3];
                        Branch = 1'b1;
                        end
        endcase
    end 
endmodule

// only B type and J type for this case
module ImmGen(  input_reg,
                output_reg
    );

    input [31:0] input_reg;
    output reg [31:0] output_reg;
    always @(*) begin
        output_reg = 32'b0;
        case(input_reg[6:0])        // opcode
            7'b0000011: begin       // I type
                        output_reg = {{20{input_reg[31]}},{input_reg[31:20]}};
                        end
            7'b1100111: begin       // I type
                        output_reg = {{20{input_reg[31]}},{input_reg[31:20]}};
                        end
            7'b0100011: begin       // S type
                        output_reg = {{20{input_reg[31]}},{input_reg[31:25]},{input_reg[11:7]}};
                        end
            7'b1100011: begin       // B type
                        output_reg = {{20{input_reg[31]}},{input_reg[7]},{input_reg[30:25]},{input_reg[11:8]},1'b0};
                        end
            7'b1101111: begin       // J type
                        output_reg = {{20{input_reg[31]}},{input_reg[19:12]},{input_reg[20]},{input_reg[30:21]},1'b0};
                        end
        endcase
    end
endmodule

// need to check ALUOp
module ALU_Ctrl(ALUOp,
                ins,
                ctrl
);
    input [3:0] ins;
    input [1:0] ALUOp;
    output reg [3:0] ctrl;
    always @(*) begin
        case({ins,ALUOp})
            6'b000010: ctrl = 4'b0010; // add
            6'b100010: ctrl = 4'b0110; // sub
            6'b011110: ctrl = 4'b0000; // and
            6'b011010: ctrl = 4'b0001; // or
            6'b001010: ctrl = 4'b0111; // slt
            default: ctrl = 4'b0010;
        endcase
    end
endmodule