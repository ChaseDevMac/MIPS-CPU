`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/14/2020 02:05:46 PM
// Design Name: 
// Module Name: testbench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module testbench();

    reg clk_tb;
        
        
    //Stage 1 wires
    wire [31:0] pc4_tb, npc_tb, pc_tb, instruction_tb;
    
    
    //Stage 2 wires
    wire [5:0] op_tb;
    wire [4:0] rs_tb, rt_tb, rd_tb, shamt_tb;
    wire [5:0] func_tb;
    wire [15:0] imm_tb;
    wire [25:0] addr_tb;
    wire wpcir_tb, wreg_tb, m2reg_tb, wmem_tb, jal_tb, aluimm_tb, shift_tb, regrt_tb, rsrtequ_tb, sext_tb;
    wire [1:0] pcsrc_tb, fwda_tb, fwdb_tb;
    wire [3:0] aluc_tb;
    wire [31:0] jpc_tb, dpc4, branchshift_tb, bpc_tb, qa_tb, da_tb, qb_tb, db_tb, dimm_tb;
    wire [4:0] drn_tb;
    
    
    //Stage 3 wires
    wire ewreg_tb, em2reg_tb, ewmem_tb, ejal_tb, ealuimm_tb, eshift_tb;
    wire [3:0] ealuc_tb;
    wire [4:0] eshamt_tb;
    wire [31:0] epc4_tb, epc8_tb, ea_tb, eb_tb, eimm_tb, ALUinputA_tb, ALUinputB_tb, ALUresult_tb, ealu_tb;
    wire [4:0] ern0_tb, ern_tb;

    
    //Stage 4 wires
    wire mwreg_tb, mm2reg_tb, mwmem_tb;
    wire [4:0] mrn_tb;
    wire [31:0] malu_tb, di_tb, do_tb;
    
    //Stage 5 wires
    wire wwreg_tb, wm2reg_tb;
    wire [4:0] wrn_tb;
    wire [31:0] walu_tb, wdo_tb, wdi_tb;
    
    
    
    //Instruction Fetch Modules
    ProgramCounter PC(clk_tb, wpcir_tb, npc_tb, pc_tb);
    adder incrementer(pc_tb, pc4_tb);
    PCmux muxPC(pcsrc_tb, pc4_tb, bpc_tb, da_tb, jpc_tb, npc_tb);
    instruction_memory IM(pc_tb, instruction_tb);
    
    //Decode Modules
    IFID IF_ID(clk_tb, wpcir_tb, pc4_tb, instruction_tb, dpc4, op_tb, rs_tb, rt_tb, rd_tb, shamt_tb, func_tb, imm_tb, addr_tb);
    ControlUnit CU(pcsrc_tb, wpcir_tb, op_tb, func_tb, rs_tb, rt_tb, mrn_tb, mm2reg_tb, mwreg_tb, 
                    ern_tb, em2reg_tb, ewreg_tb, wreg_tb, m2reg_tb, wmem_tb, jal_tb, aluc_tb, aluimm_tb, shift_tb, regrt_tb, rsrtequ_tb, sext_tb, fwdb_tb, fwda_tb);
    JumpPC Jump(pc4_tb,addr_tb, jpc_tb);
    BranchPCShift BranchShift(imm_tb, branchshift_tb);
    BranchPC BranchPC(dpc4, branchshift_tb, bpc_tb);
    Regfile RF(clk_tb, wwreg_tb, rs_tb, rt_tb, wrn_tb, wdi_tb, qa_tb, qb_tb);
    ForwardAMux ForwardA(fwda_tb, qa_tb, ealu_tb, malu_tb, do_tb, da_tb);
    ForwardBMux ForwardB(fwdb_tb, qb_tb, ealu_tb, malu_tb, do_tb, db_tb);
    rsrtEqualCheck rsrtEqual(da_tb, db_tb, rsrtequ_tb);
    SignExt SignExtender(sext_tb, imm_tb, dimm_tb);
    rsrtMux rsrtMux(regrt_tb, rt_tb, rd_tb, drn_tb);
    
    //Execution Modules
    IDEXE ID_EXE(clk_tb, wreg_tb, m2reg_tb, wmem_tb, jal_tb, aluc_tb, aluimm_tb, shift_tb, dpc4, shamt_tb, da_tb, db_tb, dimm_tb, drn_tb,
                        ewreg_tb, em2reg_tb, ewmem_tb, ejal_tb, ealuc_tb, ealuimm_tb, eshift_tb, epc4_tb, eshamt_tb, ea_tb, eb_tb, eimm_tb, ern0_tb);
    epc4Adder epc4Adder(epc4_tb, epc8_tb);                 
    ALUmuxA ALUmuxA(eshift_tb, ea_tb, eimm_tb, ALUinputA_tb);
    ALUmuxB ALUmuxB(ealuimm_tb, eb_tb, eimm_tb, ALUinputB_tb);
    ALU ALU(ealuc_tb, ALUinputA_tb, ALUinputB_tb, ALUresult_tb);
    PostALUMux PostALUMux(ejal_tb, epc8_tb, ALUresult_tb, ealu_tb);
    f u(ejal_tb, ern0_tb, ern_tb);

    //Memory Modules   
    EXEMEM EXE_MEM(clk_tb, ewreg_tb, em2reg_tb, ewmem_tb, ealu_tb, eb_tb, ern_tb,
                            mwreg_tb, mm2reg_tb, mwmem_tb, malu_tb, di_tb, mrn_tb);
    data_memory DM(clk_tb, mwmem_tb, malu_tb, di_tb, do_tb);
    
    //Write Back Module
    MEMWB MEM_WB(clk_tb, mwreg_tb, mm2reg_tb, malu_tb, do_tb, mrn_tb, 
                        wwreg_tb, wm2reg_tb, walu_tb, wdo_tb, wrn_tb);
    WBmux muxWB(wm2reg_tb, walu_tb, wdo_tb, wdi_tb);
    
    
    
    
//    wire [31:0] npc_tb, pc_tb, instruction_tb, instrsetout_tb, signextout_tb, qa_tb, qb_tb, eqa_tb, eqb_tb, outsignext, exemuxout, ALUresult, memALUresult, di_tb, do_tb, wbALUresult, wbdo, wbmux_output;
//    wire [4:0] muxout, emux, memmuxout, wbmuxout;
//    wire [3:0] aluc, ealuc;
//    wire regwrite,memtoreg, memwrite, aluimm, regrt;
//    wire eregwrite, ememtoreg, ememwrite, ealuimm;
//    wire mwreg, mm2reg, mwmem;
//    wire wwreg, wm2reg;
    
    
//    ProgramCounter PC(clk_tb, npc_tb, pc_tb);
//    adder incrementer(pc_tb, npc_tb);
//    InstrMem IM(pc_tb, instruction_tb);
//    IFID IF_ID(clk_tb, instruction_tb, instrsetout_tb);
//    ControlUnit CU(instrsetout_tb[31:26], instrsetout_tb[5:0], regwrite, memtoreg, memwrite, aluc, aluimm, regrt);
//    mux muxID(regrt, instrsetout_tb[15:11], instrsetout_tb[20:16], muxout);
//    Regfile RegisterFile(clk_tb, wwreg, instrsetout_tb[25:21], instrsetout_tb[20:16], wbmuxout, wbmux_output, qa_tb, qb_tb);
//    SignExt signextenter(instrsetout_tb[15:0], signextout_tb);
//    IDEXE ID_EXE(clk_tb, regwrite, memtoreg, memwrite, aluc, aluimm, muxout, qa_tb, qb_tb, signextout_tb, eregwrite, ememtoreg, ememwrite, ealuc, ealuimm, emux, eqa_tb, eqb_tb, outsignext);
//    ALUmux muxEXE(ealuimm, eqb_tb, outsignext, exemuxout);
//    ALU ALUEXE(ealuc, eqa_tb, exemuxout, ALUresult);
//    EXEMEM EXE_MEM(clk_tb, eregwrite, ememtoreg, ememwrite, emux, ALUresult, eqb_tb, mwreg, mm2reg, mwmem, memmuxout,memALUresult, di_tb);
//    DataMem DM(mwmem, memALUresult, di_tb, do_tb);
//    MEMWB MEM_WB(clk_tb, mwreg, mm2reg, memmuxout, memALUresult, do_tb, wwreg, wm2reg, wbmuxout, wbALUresult, wbdo);
//    WBmux muxWB(wm2reg, wbALUresult, wbdo, wbmux_output);
    
   
    initial begin
        clk_tb = 1'b1;
    end
    
    always @(clk_tb) begin
        #5
        clk_tb <= ~clk_tb;
    end
endmodule
