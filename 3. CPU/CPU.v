/*------------------------------------------------------------------------
						Testbench for The CPU
-------------------------------------------------------------------------*/
module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
	reg [7:0] instr_mem [1023:0];
    
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
	
	//instruction from the instr_mem array is read with 2 time units delays
	assign #2 INSTRUCTION [7:0] = instr_mem [PC];
	assign #2 INSTRUCTION [15:8] = instr_mem [PC+1];
	assign #2 INSTRUCTION [23:16] = instr_mem [PC+2];
	assign #2 INSTRUCTION [31:24] = instr_mem [PC+3];
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("programmer/instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);		//instantiating the CPU module

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        #3 RESET = 1'b1;
		
		#5 RESET = 1'b0;
        // finish simulation after some time
        #100
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule

/*------------------------------------------------------------------------
								The CPU
-------------------------------------------------------------------------*/
//----------CPU module design----------
module cpu(PC, INSTRUCTION, CLK, RESET);

	//cpu module input and output ports initializing
	input [31:0] INSTRUCTION;
	input CLK, RESET;
	output [31:0] PC;
	
	//initializing the PC register
	wire [31:0] PC_NEXT;
	
	//control signals initializing
	wire IMMEDIATE, COMPLEMENT;
	wire [2:0] ALUOP;
	
	//register file ports initializing
	reg WRITEENABLE = 1'b1;		//WRITEENABLE is set high forever
	wire [2:0] READREG1, READREG2, WRITEREG;
	wire [7:0] REGOUT1, REGOUT2;
	
	//ALU ports initializing
	reg [7:0] DATA2;
	wire [7:0] ALURESULT;
	
	//complement MUX output net initializing
	reg [7:0] COMP_MUX_RESULT;
	
	//REGOUT2 complent value holds the comp_result net
	wire [7:0] comp_result;
	
	//palace to hold the immediate values for *loadi*, *j* and *beq* instructions
	wire [7:0] IMMEDIATE_DATA;
	
	pc_update pc_up (RESET, CLK, PC_NEXT, PC);		//instantiating *pc_update* module to update the pc register with one time unit delay
	pc_addr pc_add (PC, PC_NEXT);	//instantiating pc_addr module to increment the PC value by 4
	
	//wire connecting to register file ports
	assign WRITEREG = INSTRUCTION[18:16];
	assign READREG1 = INSTRUCTION[10:8];
	assign READREG2 = INSTRUCTION[2:0]; 	
	assign IMMEDIATE_DATA = INSTRUCTION[7:0];
	
	//control signals unit instantiating
	control_signal ctrl_signals (INSTRUCTION [31:24], IMMEDIATE, COMPLEMENT, ALUOP);
	
	//register file instantiating
	reg_file registers (ALURESULT, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
	
	//complement_module instantiating
	complement_module comp (REGOUT2, comp_result);
	
	alu my_alu (REGOUT1, DATA2, ALURESULT, ALUOP);
	
	//negative selection mux implementation
	always @(REGOUT2, comp_result, COMPLEMENT)
	begin
		case (COMPLEMENT)
			1'b0 : COMP_MUX_RESULT = REGOUT2;		//select non complemented value as the output
			1'b1 : COMP_MUX_RESULT = comp_result;		//select 2's complemented value as the output
		endcase
	end
	
	//immediate value selection mux implementation
	always @(IMMEDIATE_DATA, COMP_MUX_RESULT, IMMEDIATE)
	begin
		case (IMMEDIATE)
			1'b0 : DATA2 = COMP_MUX_RESULT;		//select value from a register as the output
			1'b1 : DATA2 = IMMEDIATE_DATA;		//select immediate value for the output
		endcase
	end
	

endmodule


//----------Control Signal Unit module design----------
module control_signal (OPCODE, IMMEDIATE, COMPLEMENT, ALUOP);
	
	//get OPCODE as input
	input [7:0] OPCODE;
	
	//Defininig Control Signals as output
	output reg IMMEDIATE, COMPLEMENT;
	output reg [2:0] ALUOP;
	
	//decoding triggered with changes of OPCODE
	always @(OPCODE)
	#1 begin		//set delay of one time unit for instruction decoding
		
		//checking for immediate values
		if (OPCODE == 8'd5 | OPCODE == 8'd6 | OPCODE == 8'd7) begin
			IMMEDIATE = 1'b1;
		end else begin
			IMMEDIATE = 1'b0;
		end
	
		//checking for negative(complemnting)
		if (OPCODE == 8'd1) begin
			COMPLEMENT = 1'b1;
		end else begin
			COMPLEMENT = 1'b0;
		end
		
		//set ALUOP signal according to instructions opcodes
		case (OPCODE)
			8'd0 : ALUOP = 3'b001;	//*add* instruction (function - add)
			8'd1 : ALUOP = 3'b001;	//*sub* instruction (function - add)
			8'd2 : ALUOP = 3'b010;	//*and* instruction (function - and)
			8'd3 : ALUOP = 3'b011;	//*or* instruction (function - or)
			8'd4 : ALUOP = 3'b000;	//*mov* instruction (function - forward)
			8'd5 : ALUOP = 3'b000;	//*loadi* instruction (function - forward)
		endcase
		
	end
	
endmodule


//----------2's Complement module design----------
module complement_module (IN, OUT);

	//defining IN as input
	input [7:0] IN;
	
	//definnig OUT as output
	output reg [7:0] OUT;
	
	//take the complement value of IN and then add 1 to it(2's complement operation)
	always @(IN)
	begin
		OUT = #1 (~IN + 8'd1);	//set delay of 1 time unit to get the negative value of IN
	end

endmodule


//----------PC Adder module----------
module pc_addr (CURRENT_PC, NEXT_PC);
	
	//current pc as the input
	input [31:0] CURRENT_PC;

	//next_pc as the output (holds next pc value)
	output [31:0] NEXT_PC;
	
	//adding $ to the current pc to make the next_pc value
	assign #1 NEXT_PC = CURRENT_PC + 32'd4;
	
endmodule


//----------PC Adder module----------
module pc_update (RESET, CLK, NEXT_PC, CURRENT_PC);

	//RESET, CLOCK and NEXT_PC value asthe input for the module
	input RESET, CLK;
	input [31:0] NEXT_PC;
	
	//PC_REG as the output for the module
	output reg [31:0] CURRENT_PC;
	
	always @(posedge CLK)	//triggered at only the positive edge of the clock
	begin
		if(RESET == 1'b1)begin		//if reset is high reset pc to zero
			CURRENT_PC = #1 32'b0;
		end else begin
			CURRENT_PC = #1 NEXT_PC;	//otherwise update the pc value
		end
	end
	
endmodule


/*------------------------------------------------------------------------
								REGISTER FILE
-------------------------------------------------------------------------*/
//The reg_file module implementation
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS, WRITE, CLK, RESET);

	//defining input port list
	input [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;  
	input [7:0] IN;
	input CLK, RESET, WRITE;
	
	output [7:0] OUT1, OUT2; //definning output port list
	
	reg [7:0] regs [7:0];	//defininng 8-bit register array of 8
	
	integer count;		//to keep track on the index of the register file array
	
	initial
	begin
		#5;
		$display("\n\t\t\t==================================================");
		$display("\t\t\t Change of register Content Starting from Time #5");
		$display("\t\t\t==================================================\n");
		$display("\t\ttime\treg0\treg1\treg2\treg3\treg4\treg5\treg6\treg7");
		$display("\t\t---------------------------------------------------------------------");
		$monitor($time , "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", regs[0], regs[2], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
		
	end
	
	//code to see the wave forms of the regs array
	initial
    begin
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, reg_file);
        for(count = 0; count < 8; count++)begin
            $dumpvars(1, regs[count]);
		end
    end
	
	//reading data
	assign #2 OUT1 = regs[OUT1ADDRESS];
    assign #2 OUT2 = regs[OUT2ADDRESS];
	
	//Writing Data
	always @(posedge CLK)	//triggered in positive edge of clock, IN and INADDRESS
	begin
		if (WRITE == 1'b1  & RESET == 1'b0)		//if WRITE is high and do the writing
			#1 begin
				regs [INADDRESS] = IN;		//writing according to INADDRESS
			end
	end

	//Reset the register values
	always @ (posedge CLK)		//triggered in positive edge of the clock
	begin
		if (RESET == 1'b1) begin		//if RESET is high write 0 to all the registers
			#1 for (count = 0; count < 8; count++) begin
				regs [count] = 8'd0;
			end
		end
	end
	
endmodule


/*------------------------------------------------------------------------
								The ALU
-------------------------------------------------------------------------*/
//The ALU module implementation
module alu (DATA1, DATA2, RESULT, SELECT);

	//definning input ports
	input [7:0] DATA1, DATA2;
	input [2:0] SELECT;

	//definning output ports
	output reg [7:0] RESULT;
	
	//definning temparary wires to carry the result generated by each functional modules
	wire [7:0] RESULT_FW, RESULT_AD, RESULT_AND, RESULT_OR;
	
	//instantiating the each functional modules
	FORWARD fw (DATA2, RESULT_FW);
	ADD ad (DATA1, DATA2, RESULT_AD);
	AND an (DATA1, DATA2, RESULT_AND);
	OR orr (DATA1, DATA2, RESULT_OR);
	
	//The MUX implementation
	always @(RESULT_FW, RESULT_AD, RESULT_AND, RESULT_OR, SELECT)	//sensirive ports for the mux
	begin
		//definning case structure to handle the output of the mux according to the select signal
		case (SELECT)	
			3'b000 :	RESULT = RESULT_FW;		//if SELECT = 0; RESULT will get the output of FORWARD functional unit
			3'b001 :	RESULT = RESULT_AD;		//if SELECT = 1; RESULT will get the output of ADD functional unit
			3'b010 :	RESULT = RESULT_AND;	//if SELECT = 2; RESULT will get the output of AND functional unit
			3'b011 :	RESULT = RESULT_OR;		//if SELECT = 3; RESULT will get the output of OR functional unit
			default :	RESULT = 8'b0000_0000;	//if SELECT > 3; RESULT is set to the zero value
		endcase
	end
	
endmodule

//FORWARD function implementation
module FORWARD (DATA2, RESULT);

	input [7:0] DATA2;		//definning input ports
	output reg [7:0] RESULT;	//definning output ports
	
	//Forward operation happens and result is stored in RESULT net
	always @(DATA2)
	#1 begin
		RESULT = DATA2;		//unit delay assign to 1 time unit
	end

endmodule

//ADD function implementation
module ADD (DATA1, DATA2, RESULT);
	
	input [7:0] DATA1, DATA2;	//definning input ports
	output reg [7:0] RESULT;	//definning output ports
	
	//ADD operation happens and result is stored in RESULT net
	always @(DATA1, DATA2)
	#2 begin
		RESULT = (DATA1 + DATA2);		//unit time delay assign to 2 time unit
	end
	
endmodule

//AND function implementation
module AND(DATA1, DATA2, RESULT);
	
	input [7:0] DATA1, DATA2;	//definning input ports
	output reg [7:0] RESULT;	//definning output ports
	
	//AND operation happens and result is stored in RESULT net
	always @(DATA1, DATA2)
	#1 begin
		RESULT = (DATA1 & DATA2);		//unit time delay assign to 1 time unit
	end
	
endmodule

//OR function implementation
module OR (DATA1, DATA2, RESULT);

	input [7:0] DATA1, DATA2;	//definning input ports
	output reg [7:0] RESULT;	//definning output ports
	
	//OR operation happen result is stored in RESULT net
	always @(DATA1, DATA2)
	#1 begin
		RESULT = (DATA1 | DATA2);		//unit time delay assign to 1 time unit
	end

endmodule
