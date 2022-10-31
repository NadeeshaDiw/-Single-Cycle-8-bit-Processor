//testbench to test reg_file module
module testbed;

	//definning signals that are needed to instantiate the reg_file module
	reg [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;
	reg [7:0] IN;
	reg CLK, RESET, WRITE;
	wire [7:0] OUT1, OUT2;
	
	//set to monitor the values in the signals
	initial begin
		$monitor ($time, " | in_data- %d -- in_ad- %d | out1- %d -- out_ad1- %d | out2- %d -- out_ad2- %d | ( w-%d | c-%d | r-%d )", IN, INADDRESS, OUT1, OUT1ADDRESS, OUT2, OUT2ADDRESS, WRITE, CLK, RESET);
	end
	
	//instantiate the reg_file module
	reg_file register (IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS, WRITE, CLK, RESET);
	
	//creating vcd file to observe the wave forms
	initial begin
		$dumpfile("my.vcd"); 
		$dumpvars(0, testbed);	
	end

	//set values to the signals
	initial
	begin
		WRITE = 1'b1; RESET= 1'b0; //WRITE is high and RESET is low at the begining
		#2 IN = 8'b0010_1010; INADDRESS = 3'b001; //after 2 time units IN = 42, and INADDRESS = 1
		#5 OUT1ADDRESS = 3'b001; OUT2ADDRESS = 3'b010; //after 5 time unit OUT1ADDRESS = 1, OUT2ADDRESS = 2
		#5 RESET = 1'b1; //after 5 time units RESET is high
		#5 OUT1ADDRESS = 3'b001; OUT2ADDRESS = 3'b010; //after 5 time unit OUT1ADDRESS = 1, OUT2ADDRESS = 2
		#5 RESET = 1'b0; WRITE = 1'b0; //after 5 time units RESET is low and WRITE is low
		#5 IN = 8'b0000_1110; INADDRESS = 3'b011; //after 5 time units IN = 14, and INADDRESS = 3
		#5 OUT1ADDRESS = 3'b011; OUT2ADDRESS = 3'b001;	//after 5 time unit OUT1ADDRESS = 3, OUT2ADDRESS = 1
		#5 WRITE = 1'b1; //after 5 time units WRITE is high
		#5 IN = 8'b0010_1110; INADDRESS = 3'b101; //after 5 time units IN = 46, and INADDRESS = 5
		#5 OUT1ADDRESS = 3'b011; OUT2ADDRESS = 3'b101;	//after 5 time unit OUT1ADDRESS = 3, OUT2ADDRESS = 5
		#5 IN = 8'b0011_1110; INADDRESS = 3'b111;	//after 5 time units IN = 62, and INADDRESS = 7
		#5 OUT1ADDRESS = 3'b111; OUT2ADDRESS = 3'b101;	//after 5 time unit OUT1ADDRESS = 7, OUT2ADDRESS = 5
		#5 RESET = 1'b1;	//after 5 time units RESET is high
		#5 OUT1ADDRESS = 3'b111; OUT2ADDRESS = 3'b101;
		
	end
	
	//set the clock
	initial
	begin
		CLK = 1'b0;
		forever #1 CLK = ~CLK;
	end
	
	//defining the simulation life
	initial
	begin
		#75 $finish;
	end
	
endmodule

//The reg_file module implementation
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS, WRITE, CLK, RESET);

	//defining input port list
	input [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;  
	input [7:0] IN;
	input CLK, RESET, WRITE;
	
	output [7:0] OUT1, OUT2; //definning output port list
	
	reg [7:0] regs [7:0];	//defininng 8-bit register array of 8
	
	//Reading the data from register file
	assign #2 OUT1 = regs[OUT1ADDRESS];		// assign OUT1 according to OUT1ADDRESS
	assign #2 OUT2 = regs[OUT2ADDRESS];		// assign OUT2 according to OUT2ADDRESS
	
	//Writing Data
	always @(posedge CLK)	//triggered in positive edge of clock, IN and INADDRESS
	begin
		if (WRITE == 1'b1 & RESET == 1'b0)		//if WRITE is high and RESET is low do the writing
			begin
				regs [INADDRESS] <= #1 IN;		//writing according to INADDRESS
			end
	end 
	
	//Reset the register values
	always @(posedge CLK)		//triggered in positive edge of the clock
	begin
		if (RESET == 1'b1)		//if RESET is high write 0 to all the registers
			begin
				regs [0] <=  #1 8'b0000_0000;
				regs [1] <=  #1 8'b0000_0000;
				regs [2] <=  #1 8'b0000_0000;
				regs [3] <=  #1 8'b0000_0000;
				regs [4] <=  #1 8'b0000_0000;
				regs [5] <=  #1 8'b0000_0000;
				regs [6] <=  #1 8'b0000_0000;
				regs [7] <=  #1 8'b0000_0000;
			end
	end
	
endmodule