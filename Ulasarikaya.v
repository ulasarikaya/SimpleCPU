
module SimpleCPU(clk, rst, data_fromRAM, wrEn, addr_toRAM, data_toRAM);

parameter SIZE = 14;

input clk, rst;
input wire [31:0] data_fromRAM;
output reg wrEn;
output reg [SIZE-1:0] addr_toRAM;
output reg [31:0] data_toRAM;

reg[3:0] state_current , state_next;
reg[SIZE-1:0] pc_current , pc_next;
reg[31:0] iw_current , iw_next;
reg[31:0] r1_current , r1_next;
reg[31:0] r2_current , r2_next;

always@(posedge clk) begin
	if(rst) begin
	state_current <= 0 ;
	pc_current <= 14'b0 ;
	iw_current <= 32'b0 ;
	r1_current <= 32'b0 ;
	r2_current <= 32'b0 ;
	end
	else begin
	state_current <= state_next;
	pc_current <= pc_next ;
	iw_current <= iw_next;
	r1_current <= r1_next ;
	r2_current <= r2_next ;
	end
end	

always@(*) begin
	state_next = state_current ;
	pc_next    = pc_current;
	iw_next	  = iw_current;
	r1_next	  = r1_current ;
	r2_next    = r2_current ;
	wrEn		  = 0;
	addr_toRAM = 0;
	data_toRAM = 0;
	case(state_current)
	0:begin
	pc_next= 0;
	iw_next= 0;
	r1_next= 0;
	r2_next= 0;
	state_next =1;
	end
	1:begin
	addr_toRAM = pc_current;
	state_next = 2;
	end
	2:begin
		iw_next =data_fromRAM;
		case(data_fromRAM [31:28])
			{3'b000,1'b0}:begin // ADD
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next=3;
			end
			{3'b000,1'b1}:begin // ADDi
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next=3;
			end
			{3'b100,1'b0}:begin // CP
				addr_toRAM = data_fromRAM[13:0]; //address b
				state_next = 3;
			end
			{3'b100, 1'b1}: begin // CPi
                r2_next    = data_fromRAM[13:0];
                wrEn       = 1'b1;
                pc_next    = pc_current + 1;
                addr_toRAM = data_fromRAM[27:14];
                data_toRAM = data_fromRAM[13:0];
                state_next = 1;
            end
            {3'b101,1'b0}:begin // CPl
				addr_toRAM = data_fromRAM[13:0]; //address b
				state_next = 3;
			end
			{3'b101,1'b1}:begin // CPli
				addr_toRAM = data_fromRAM[13:0]; //address b
				state_next = 3;
			end
			{3'b001,1'b0}:begin // NAND
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			{3'b001,1'b1}:begin // NANDi
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			{3'b010,1'b0}:begin // SRL
				addr_toRAM = data_fromRAM[13:0]; //address b
				state_next = 3;
			end
			{3'b010,1'b1}:begin // SRLi
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			{3'b011,1'b0}:begin // LT
				addr_toRAM = data_fromRAM[13:0]; //address b
				state_next = 3;
			end
			{3'b011,1'b1}:begin // LTi
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			{3'b111,1'b0}:begin // MUL
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			{3'b111,1'b1}:begin // MULi
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			{3'b110,1'b0}:begin // BZJ
				addr_toRAM = data_fromRAM[13:0]; //address b
				state_next = 3;
			end
			{3'b110,1'b1}:begin // BZJi
				addr_toRAM = data_fromRAM[27:14]; //address a
				state_next = 3;
			end
			default: begin
				pc_next = pc_current;
				state_next = 1;
			end
		endcase
	end
	3:begin
	case(data_fromRAM [31:28])
		{3'b000,1'b0}:begin // ADD
			r1_next = data_fromRAM;
			addr_toRAM = iw_current[13:0];
			state_next =4;
		end
		{3'b000,1'b1}:begin // ADDi
			r1_next = data_fromRAM;
			addr_toRAM = iw_current[13:0];
			state_next =4;
		end
		{3'b100,1'b0}:begin // CP
			r1_next = data_fromRAM;
			addr_toRAM = iw_current[13:0];
			state_next =4;
		end
		{3'b101,1'b0}:begin // CPl
			addr_toRAM = data_fromRAM;
			state_next =4;
		end
		{3'b101,1'b1}:begin // CPli
			r1_next = data_fromRAM;
			addr_toRAM=iw_current[27:14];
			state_next =4;
		end
		{3'b001,1'b0}:begin // NAND
			r1_next = data_fromRAM;
			addr_toRAM = iw_current[13:0];
			state_next =4;
		end
		{3'b001,1'b1}:begin // NANDi
			wrEN = 1;
			addr_toRAM = iw_current[27:14];
			data_toRAM = ~(iw_current[13:0] & data_fromRAM);
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b010,1'b0}:begin // SRL
			r1_next = data_fromRAM;
			addr_toRAM =data_fromRAM[27:14]; //address a
			state_next=4;
		end
		{3'b010,1'b1}:begin // SRLi
			wrEN = 1;
			addr_toRAM = iw_current[27:14];
			data_toRAM = (iw_current[13:0] < 32) ? (data_fromRAM >> iw_current[13:0]) : (data_fromRAM << iw_current[13:0]);
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b011,1'b0}:begin // LT
			r1_next = data_fromRAM;
			addr_toRAM = data_fromRAM[27:14]; //address a
			state_next=4;
		end
		{3'b011,1'b1}:begin // LTi
			wrEn = 1 ;
			addr_toRAM = iw_current[27:14];
			data_toRAM = (iw_current[13:0] < data_fromRAM) ? 1 : 0;
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b111,1'b0}:begin // MUL
			r1_next = data_fromRAM;
			addr_toRAM = data_fromRAM[13:0];
			state_next = 4;
		end
		{3'b111,1'b1}:begin // MULi
			wrEn = 1;
			addr_toRAM = iw_current[27:14];
			data_toRAM = data_fromRAM * iw_current[13:0];
			pc_next = pc_current +1'b1;
			state_next = 1;
		end
		{3'b110,1'b0}:begin // BZJ
			r1_next = data_fromRAM;
			addr_toRAM = iw_current[27:14]; //address a
			state_next = 4;
		end
		{3'b110,1'b1}:begin // BZJi
			pc_next = data_fromRAM + iw_current[13:0];
			state_next = 1;
		end
	endcase
	end
	4:begin
		case(iw_current[31:28])
		{3'b000,1'b0}:begin // ADD
		wrEn = 1 ;
		addr_toRAM = iw_current[27:14];
		data_toRAM = data_fromRAM + r1_current ;
		pc_next = pc_current +1'b1;
		state_next =1;
		end
		{3'b000,1'b1}:begin // ADDi
		wrEn = 1 ;
		addr_toRAM = iw_current[27:14];
		data_toRAM = data_fromRAM + r1_current ;
		pc_next = pc_current +1'b1;
		state_next =1;
		end
		{3'b100,1'b0}:begin // CP
			wrEn = 1 ;
			addr_toRAM =  iw_current[27:14];
			data_toRAM = data_fromRAM  ;
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b101,1'b0}:begin // CPl
			wrEn = 1 ;
			addr_toRAM =  iw_current[27:14];
			data_toRAM = data_fromRAM  ;
			pc_next = pc_current +1'b1;
			state_next =1;
			end
		{3'b101,1'b0}:begin // CPli
		wrEn = 1 ;
		addr_toRAM = data_fromRAM  ;
		data_toRAM = r1_current;
		pc_next = pc_current +1'b1;
		state_next =1;
			end
		{3'b001,1'b0}:begin // NAND
			wrEn = 1 ;
			addr_toRAM = iw_current[27:14];
			data_toRAM = ~(data_fromRAM & r1_current);
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b001,1'b0}:begin // SRL
			wrEn = 1 ;
			addr_toRAM = iw_current[27:14];
			data_toRAM = (r1_current < 32) ? (data_fromRAM >> r1_current) : (data_fromRAM << r1_current);
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b011,1'b0}:begin // LT
			wrEn = 1 ;
			addr_toRAM = iw_current[27:14];
			data_toRAM = (data_fromRAM < r1_current) ? 1 : 0;
			pc_next = pc_current +1'b1;
			state_next =1;
		end
		{3'b111,1'b0}:begin // MUL
			wrEn = 1;
			addr_toRAM = iw_current[27:14];
			data_toRAM = data_fromRAM * r1_current;
			pc_next = pc_current +1'b1;
			state_next = 1;
		end
		{3'b110,1'b0}:begin // BZJ
			pc_next = (r1_current == 0) ? (data_fromRAM) : (pc_current + 1);
			state_next = 1;
		end
		endcase
		end
	endcase	
end	
endmodule

module blram(clk, rst, i_we, i_addr, i_ram_data_in, o_ram_data_out);

parameter SIZE = 10, DEPTH = 1024;

input clk;
input rst;
input i_we;
input [SIZE-1:0] i_addr;
input [31:0] i_ram_data_in;
output reg [31:0] o_ram_data_out;

reg [31:0] memory[0:DEPTH-1];

always @(posedge clk) begin
  o_ram_data_out <= #1 memory[i_addr[SIZE-1:0]];
  if (i_we)
		memory[i_addr[SIZE-1:0]] <= #1 i_ram_data_in;
end 

initial begin
memory[0] = 32'h20114045;
memory[1] = 32'h10114001;
memory[2] = 32'hb0118064;
memory[3] = 32'h190045;
memory[4] = 32'h8011c064;
memory[5] = 32'h7011c001;
memory[6] = 32'h10118001;
memory[7] = 32'hc0120047;
memory[8] = 32'h118045;
memory[9] = 32'ha0128046;
memory[10] = 32'h8012c046;
memory[11] = 32'h12c045;
memory[12] = 32'ha013004b;
memory[13] = 32'he013004a;
memory[14] = 32'h8012804c;
memory[15] = 32'h8013404b;
memory[16] = 32'h701343e9;
memory[17] = 32'hc012404d;
memory[18] = 32'h8019404c;
memory[19] = 32'hd0050013;
memory[20] = 32'h0;
memory[69] = 32'h1;
memory[70] = 32'h3e8;
memory[72] = 32'h2;
memory[73] = 32'hb;
memory[100] = 32'h6;
memory[101] = 32'h0;
memory[999] = 32'h1;
end

endmodule
