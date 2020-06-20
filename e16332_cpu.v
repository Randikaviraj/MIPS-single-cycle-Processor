`timescale 1ns/100ps
`include "dcache.v"
`include "icache.v"

module test_tb;
    
    reg CLK, RESET; 
    reg [7:0] MEMORY[0:1023];
    
    wire read,write,i_busywait,d_busywait;
    wire [7:0] readdata,writedata,address;
    wire [31:0] PC,INS_MEMORY;
    //wire [31:0] INS_MEMORY;
    
    dcache my_dcache(CLK,RESET,read,write,address,writedata,readdata,d_busywait);
    cpu mycpu(RESET,CLK,PC,INS_MEMORY,read,write,address,writedata,readdata,d_busywait,i_busywait); //give Instruction to the cpu module depend on the pc
  
    icache myicache(CLK,RESET,PC[9:0],INS_MEMORY,i_busywait);
       
    initial
    begin
        CLK = 1'b1; //give the instructions as a array
         
        
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, test_tb);
        
        // assign values with time to input signals to see output 
        RESET = 1'b0;
        #1 RESET= 1'b1;
        #1 RESET= 1'b0;
        
        

        
        #2040
        $finish;
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule






module cpu(RESET,CLK,PC,INS_FROM_MEMORY,read,write,address,writedata,readdata,d_busywait,i_busywait);

input RESET,CLK,d_busywait,i_busywait,busywait;
input [7:0] readdata;
input [31:0] INS_FROM_MEMORY;

output write,read;
output [7:0] address,writedata;
output reg [31:0] PC;

wire [31:0]OUTPUT_PC;


control_unit my_control_unit(RESET,CLK,PC,INS_FROM_MEMORY,OUTPUT_PC,read,write,address,writedata,readdata,d_busywait);
or(busywait,d_busywait,i_busywait);

always @(RESET) begin //set the pc value depend on the RESET to start the programme
    PC=-4;
end

always @(posedge CLK) begin //update the pc value depend on the positive clock edge
        #1 
        if(busywait==1'b0) PC=OUTPUT_PC; //update the pc when only busywait is zero 
        
end  
endmodule








module control_unit(RESET,CLK,PC,INSTRUCTION,OUTPUT_PC,read,write,address,writedata,readdata,busywait);

input RESET,CLK,busywait;
input [7:0] readdata;
input [31:0] INSTRUCTION,PC;

output read,write;
output [7:0] writedata,address;
output [31:0] OUTPUT_PC;


wire [2:0] WRITEREG, READREG1, READREG2;
wire [5:0] OPCODE;
wire [7:0] REGOUT1, REGOUT2,ALURESULT,reg_write_data;
wire ZERO,beq_signal,bnq_signal,write_enable;
wire [31:0] IMMIDIATE,OUTPUT_ADDED_PC,OUT_MUX3,PC_INCREMENT;

reg [7:0] DATA2;
reg [2:0] ALUOP;
reg MUX1,MUX2,WRITEENABLE,MUX3,MUX4,MUX5,b_signal,MEMORY_WRITE,MEMORY_READ,REG_WRITE_IN;

assign   #1 PC_INCREMENT=PC+4; //increment the pc value by 4 by adder
assign   WRITEREG=INSTRUCTION[18:16]; //decode the instruction and get Writereg
assign   READREG1=INSTRUCTION[10:8];  //decode the instruction and get readreg1
assign   READREG2=INSTRUCTION[2:0];   //decode the instruction and get readreg2
assign   OPCODE=INSTRUCTION[28:24];   //decode the instruction and get opcode

assign read=MEMORY_READ; //assign the signal of read memory to wire goes to memory
assign write=MEMORY_WRITE; //assign the signal of  write memory to wire goes to memory
assign address=ALURESULT;//assign the alu result for the address wire goes to memory
assign writedata=REGOUT1; //assign the REGOUT1 for the writedata wire goes to memory




and(beq_signal,MUX4,ZERO);//make and operation between zero signal and MUx4 to check whether equal the two registers of beq
and(bnq_signal,MUX4,~ZERO); //nd operation between not of zero signal and MUx4 to check whether equal the two registers of beq
and(write_enable,WRITEENABLE,1);//control the write enable signal to write for rigiste when we want to stall the pc



adder_pc_to_immidiate mypcadder(IMMIDIATE,PC_INCREMENT,OUTPUT_ADDED_PC); //extended imidiate value in distination field is being added to pc
signExtention mysignExtention(INSTRUCTION[23:16],IMMIDIATE);//pass the imidate value in distination field to sign extension
reg_file File(reg_write_data, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, write_enable, CLK, RESET); //reg file control 
alu mainalu(REGOUT1, DATA2,ALURESULT, ALUOP,ZERO); //alu control
mux_2_to_1 mymux3(OUTPUT_ADDED_PC,PC_INCREMENT,MUX3,OUT_MUX3);//check wheather jump or not when jump immidiate added is taken
mux_2_to_1 mymux4(OUTPUT_ADDED_PC,OUT_MUX3,b_signal,OUTPUT_PC);//check wheather beq and output ZERO,or not 



mux_2_to_1_8bit mymux5(ALURESULT,readdata,REG_WRITE_IN,reg_write_data);//choose the register write data whether Alu result or memory read


always @(MUX5,bnq_signal,beq_signal) begin
    if(MUX5==1'b1) b_signal=bnq_signal;//chceck weather is it a bnq instruction
    else b_signal=beq_signal; //if it is not bnq and it may be beq or normal instruction
end





always @(RESET) //when reset zero make write enable to zero for avoiding written garbage to register
begin
  WRITEENABLE=1'b0 ; 
  MEMORY_WRITE=1'b0;
  MEMORY_READ=1'b0;
end

always @(INSTRUCTION) begin
    if (busywait==1'b0) begin
        MEMORY_WRITE=1'b0;
        MEMORY_READ=1'b0; 
    end       
end



always @(INSTRUCTION)  //depend on the opcode generate the control signals 
begin
    #1
   case(OPCODE)
        5'b00010 :begin       //add
                WRITEENABLE=1'b1; //write enable signal
                MUX1=1'b1; //mux1 signal
                MUX2=1'b0; //mux2 signal
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b001; //aluop code
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end      
        5'b00011 :begin     //sub
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b001;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end          
        5'b00101 :begin     //or
                WRITEENABLE=1'b1;
                MUX1=1'b1;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b011;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end               
        5'b00100 :begin    //and
                WRITEENABLE=1'b1;
                MUX1=1'b1;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b010;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end               
        5'b00001 :begin      //mov
                WRITEENABLE=1'b1;
                MUX1=1'b1;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b000;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end              
        5'b00000 :begin     //lodi
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b000;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end
        5'b00110 :begin     //J instruction
                WRITEENABLE=1'b0;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b1;
                MUX4=1'b0;
                MUX5=1'b0;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                //ALUOP=3'b000;
                end       

        5'b00111 :begin     //beq instruction
                WRITEENABLE=1'b0;
                MUX1=1'b0;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b1;
                MUX5=1'b0;
                MUX5=1'b0;
                ALUOP=3'b001;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                end

        5'b11000 :begin     //left logical shift instruction
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b110;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end   

        5'b11100 :begin     //right logical shift instruction
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b100;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end                                   

        5'b11110 :begin     //right arithmatic shift instruction
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b101;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end       

        5'b11001 :begin     //bne instruction
                WRITEENABLE=1'b0;
                MUX1=1'b0;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b1;
                MUX5=1'b1;
                ALUOP=3'b001;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                end  

        5'b11011 :begin     //rotate instruction
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b111;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b1;
                end         

         5'b01000 :begin     //lwd (register direct addressing) memory read
                WRITEENABLE=1'b1;
                MUX1=1'b1;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b000;
                MEMORY_READ=1'b1;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b0;
                end

         5'b01001 :begin     //lwi (immediate addressing) memory read
                WRITEENABLE=1'b1;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b000;
                MEMORY_READ=1'b1;
                MEMORY_WRITE=1'b0;
                REG_WRITE_IN=1'b0;
                end 

         5'b01010 :begin     //swd (register direct addressing) memory write
                WRITEENABLE=1'b0;
                MUX1=1'b1;
                MUX2=1'b0;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b000;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b1;
                end   

         5'b01011 :begin     //swi (immediate addressing) memory write
                WRITEENABLE=1'b0;
                MUX1=1'b0;
                MUX2=1'b1;
                MUX3=1'b0;
                MUX4=1'b0;
                MUX5=1'b0;
                ALUOP=3'b000;
                MEMORY_READ=1'b0;
                MEMORY_WRITE=1'b1;
                end                                                               
    endcase
   
end





always @(MUX2 or MUX1 or REGOUT2 or INSTRUCTION) //depend on mux1,mux2 give the operand2{DATA2} to the alu
begin
    if(MUX2==1'b1) begin
            DATA2=INSTRUCTION[7:0];
            
        end
    else if (MUX1==1'b1) begin
        DATA2=REGOUT2;
    end else begin
        #1 DATA2=~REGOUT2+1;
    end
end


endmodule




module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
	input [2:0] INADDRESS, OUT1ADDRESS,OUT2ADDRESS; //declare inputs
	input [7:0] IN; //declare input In
	input WRITE,CLK,RESET;
	output  [7:0] OUT1,OUT2; //declare outputs
    reg [7:0] register[0:7]; //register file with 8 registers
	integer i; //declare two integers
    integer j;

   

    always @(OUT1ADDRESS or OUT2ADDRESS) //define always block for select OUT1,OUT2 registers
    begin                                //depend on the OUT1ADDRESS and OUT2ADDRESS
        case(OUT1ADDRESS) //select integer i on OUT1ADDRESS
                3'b000 :i=0;
                3'b001 :i=1;
                3'b010 :i=2;
                3'b011 :i=3;
                3'b100 :i=4;
                3'b101 :i=5;
                3'b110 :i=6;
                3'b111 :i=7;
                
        endcase
                
        case(OUT2ADDRESS)  //select integer i on OUT2ADDRESS
                3'b000 :j=0;
                3'b001 :j=1;
                3'b010 :j=2;
                3'b011 :j=3;
                3'b100 :j=4;
                3'b101 :j=5;
                3'b110 :j=6;
                3'b111 :j=7;
                
        endcase

    end


	always @(posedge CLK or  RESET) //define always block for write and reset the register 
	begin                           //on the clock edge and reset level
        if(RESET==1'b1) //if reset is 1 then every register become zero 
            #2  //delay for reset the register file
            begin
            register[0]=8'b00000000;
            register[1]=8'b00000000;
            register[2]=8'b00000000;
            register[3]=8'b00000000;
            register[4]=8'b00000000;
            register[5]=8'b00000000;
            register[6]=8'b00000000;
            register[7]=8'b00000000;
            end
        else if(WRITE==1'b1) //if write is enable regiter is writting depend on INADDRESS
            #1               //on the clock edge,#1 for give delay for writting
            begin
                case(INADDRESS) //write register on INADDRESS
                    3'b000 :register[0]=IN;
                    3'b001 :register[1]=IN;
                    3'b010 :register[2]=IN;
                    3'b011 :register[3]=IN;
                    3'b100 :register[4]=IN;
                    3'b101 :register[5]=IN;
                    3'b110 :register[6]=IN;
                    3'b111 :register[7]=IN;
                
                endcase
            end

        
	end
    assign #2 OUT1=register[i];  //Set OUT1 asynchronsly 
    assign #2 OUT2=register[j];  //Set OUT2 asynchronsly ,OUT1,OUT2 are changing when  assign value is changing
	
endmodule




module  alu(DATA1, DATA2, RESULT, SELECT,ZERO); //module deleration
	input [7:0] DATA1,DATA2;  //define input  two 8 bit buses for oprands
	input [2:0] SELECT; //define input 3 bit buse for alu op selector
	
    output reg [7:0] RESULT; //define 8 bit out bus as a reg type
    output ZERO;

    wire [7:0] shiftwire,signshiftwire,rotatewire;
    wire wire1,wire2,wire3;

    or(wire1,RESULT[0],RESULT[1],RESULT[2]);//getting the zero signal when result is zero
    or(wire2,RESULT[3],RESULT[4],RESULT[5]);
    or(wire3,RESULT[5],RESULT[6]);
    nor(ZERO,wire1,wire2,wire3);

    rotate myrotate(DATA1,DATA2[2:0],rotatewire); //roatate the reg value
    shift myshift(DATA1,DATA2[2:0],SELECT[1],shiftwire); //shift the reg value when select[1]==1 then it is left one
    signshift mysignshift(DATA1,DATA2[2:0],signshiftwire);//ign shift shift amount is taken on DATA2[2:0]

always @(DATA1 or DATA2 or SELECT)  //declare always block wich sensitive for DATA1,DATA2,SELECT
	begin
		#1
		case (SELECT) //case block for handle the selector 
			3'b000 : RESULT=DATA2;  //FORWARD instruction output
					
			3'b001 : #1 RESULT=DATA1+DATA2; //ADD instruction output
                        
			3'b010 :RESULT=DATA1&DATA2; // bitwise and for AND instruction output
					
			3'b011 :RESULT=DATA1|DATA2; // bitwise or for OR instruction output
					 
			3'b100 :RESULT=shiftwire; // logical right shift

            3'b110 :RESULT=shiftwire; // logical left shift

            3'b101 :RESULT=signshiftwire; // arithmatic right shift

            3'b111 :RESULT=rotatewire; // rotate shift
					
		endcase 
	end 
endmodule





module signExtention(
    Immidiate,ExtendedImmidiate
);

integer i;

input [7:0] Immidiate;
output reg [31:0] ExtendedImmidiate;


always @(Immidiate) begin
    ExtendedImmidiate[1:0]=2'b00;
    ExtendedImmidiate[9:2]=Immidiate;
    for (i =10; i < 32; i = i + 1) begin
      ExtendedImmidiate[i]=Immidiate[7];
    end
end

endmodule // signExtention for extend the word imidiate to bit address



module adder_pc_to_immidiate( //adder for add pc and immidiate
    IMMIDIATE,PC,PC_OUT
);
input [31:0] IMMIDIATE,PC;
output reg [31:0] PC_OUT;

always @(IMMIDIATE or PC) begin
    #2 PC_OUT=PC+IMMIDIATE;
end

endmodule // adder




module mux_2_to_1( //mux for get 32 bit two inputs and output on select
    IN1,IN2,sel,out
);

input [31:0] IN1,IN2;
input sel;
output reg [31:0] out;

always @(sel,IN1,IN2) begin
    if (sel==1'b1) out =IN1;
    else out =IN2;
end

endmodule // 2X1_Mux




module mux_2_to_1_8bit( //mux for get 32 bit two inputs and output on select
    IN1,IN2,sel,out
);

input [7:0] IN1,IN2;
input sel;
output reg [7:0] out;

always @(sel,IN1,IN2) begin
    if (sel==1'b1) out =IN1;
    else out =IN2;
end

endmodule // 2X1_Mux






module shift( // this is ueed to logical shift 
    regvalue,sel,type_shift,out
);
input [7:0] regvalue;
input [2:0] sel;
input type_shift;
output reg [7:0]out;

always @(*) 
begin
    if(type_shift==1'b1)begin //check for left shifts
        case (sel)
            3'b000: out=regvalue; //depend on the select bit shifting is done
            3'b001: out={regvalue[6:0],1'b0};
            3'b010: out={regvalue[5:0],2'b00};
            3'b011: out={regvalue[4:0],3'b000};
            3'b100: out={regvalue[3:0],4'b0000};
            3'b101: out={regvalue[2:0],5'b00000};
            3'b110: out={regvalue[1:0],6'b000000};
            3'b111:  out={regvalue[0],7'b0000000};
            default:out=8'b00000000;
        endcase
    end
    else begin
        case (sel) //check in right shift
            3'b000: out=regvalue;
            3'b001: out={1'b0,regvalue[7:1]};
            3'b010: out={2'b00,regvalue[7:2]};
            3'b011: out={3'b000,regvalue[7:3]};
            3'b100: out={4'b0000,regvalue[7:4]};
            3'b101: out={5'b00000,regvalue[7:5]};
            3'b110: out={6'b000000,regvalue[7:6]};
            3'b111:  out={7'b0000000,regvalue[7]};
            default:out=8'b00000000;
        endcase
    end
end

endmodule //  



module signshift( //this is used to sign right shift
    regvalue,sel,out
);
input [7:0] regvalue;
input [2:0] sel;
output reg [7:0]out;

always @(*)begin
       case (sel) //depend on the select bit shifting is done
            3'b000: out=regvalue;
            3'b001: out={regvalue[7],regvalue[7:1]};
            3'b010: out={regvalue[7],regvalue[7],regvalue[7:2]};
            3'b011: out={regvalue[7],regvalue[7],regvalue[7],regvalue[7:3]};
            3'b100: out={regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7:4]};
            3'b101: out={regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7:5]};
            3'b110: out={regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7:6]};
            default: out={regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7],regvalue[7]};
            
        endcase
end

endmodule 



module rotate( //this is use to ratate the 8bit value with on sel 
    regvalue,sel,out
);
input [7:0] regvalue;
input [2:0] sel;
output reg [7:0]out;

always @(*)begin
       case (sel)
            3'b000: out=regvalue;
            3'b001: out={regvalue[6:0],regvalue[7]};
            3'b010: out={regvalue[5:0],regvalue[7:6]};
            3'b011: out={regvalue[4:0],regvalue[7:5]};
            3'b100: out={regvalue[3:0],regvalue[7:4]};
            3'b101: out={regvalue[2:0],regvalue[7:3]};
            3'b110: out={regvalue[1:0],regvalue[7:2]};
            3'b111:out={regvalue[0],regvalue[7:1]};
        endcase
end

endmodule // rotate




module bitand(
    word,bit,outword
);
input [7:0] word;
input bit;
output [7:0] outword;

and(outword[0],bit,word[0]);
and(outword[1],bit,word[1]);
and(outword[2],bit,word[2]);
and(outword[3],bit,word[3]);
and(outword[4],bit,word[4]);
and(outword[5],bit,word[5]);
and(outword[6],bit,word[6]);
and(outword[7],bit,word[7]);
endmodule // bitand


module adder_8bit(
    in1,in2,out
);
input [7:0] in1,in2;
output [7:0]out;

assign #1 out=in1+in2;
endmodule // adder_8bit




// module data_memory(
// 	clock,
//     reset,
//     read,
//     write,
//     address,
//     writedata,
//     readdata,
//     busywait
// );
// input           clock;
// input           reset;
// input           read;
// input           write;
// input[7:0]      address;
// input[7:0]      writedata;
// output reg [7:0]readdata;
// output reg      busywait;

// //Declare memory array 256x8-bits 
// reg [7:0] memory_array [255:0];

// //Detecting an incoming memory access
// reg readaccess, writeaccess;
// always @(read, write)
// begin
// 	busywait = (read || write)? 1 : 0;
// 	readaccess = (read && !write)? 1 : 0;
// 	writeaccess = (!read && write)? 1 : 0;
// end

// //Reading & writing
// always @(posedge clock)
// begin
//     if(readaccess)
//     begin
//         readdata = #40 memory_array[address];
//         busywait = 0;
// 		readaccess = 0;
//     end
//     if(writeaccess)
// 	begin
//         memory_array[address] = #40 writedata;
//         busywait = 0;
// 		writeaccess = 0;
//     end
// end

// integer i;

// //Reset memory
// always @(posedge reset)
// begin
//     if (reset)
//     begin
//         for (i=0;i<256; i=i+1)
//             memory_array[i] = 0;
        
//         busywait = 0;
// 		readaccess = 0;
// 		writeaccess = 0;
//     end
// end

// endmodule
