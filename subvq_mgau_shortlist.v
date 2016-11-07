`define size_m 16
`define size_n 8

module sphinx_func3(start,clk,rst,n_sv,m,n,beam,score_received,score,score_ready,VQ_EVAL);
parameter data_width=32;
parameter addr_width=32;

input start,clk,rst;
input [data_width-1:0] n_sv;
input [data_width-1:0] m,n,beam;
input score_received;
input [1:0]VQ_EVAL;

output [data_width-1:0] score;
output score_ready;

wire NOT_ENABLED=0;
wire [data_width-1:0] NOT_USED=32'b0;

wire [addr_width-1:0]map_addr,map_addr_dim1,map_addr_dim2,vqdist_addr,gauscore_addr,mgau_sl_addr;

wire [data_width-1:0] gauscore_data_in,mgau_sl_data_in;
wire [data_width-1:0] map_data_out,vqdist_data_out,gauscore_data_out,mgau_sl_data_out;

wire gauscore_en,sl_en;

assign map_addr=map_addr_dim1*24+map_addr_dim2;
ram_map #(data_width,addr_width) array_map(NOT_USED,map_addr,NOT_ENABLED,clk,map_data_out);
ram_vqdist #(data_width,addr_width) array_vqdist(NOT_USED,vqdist_addr,NOT_ENABLED,clk,vqdist_data_out);
ram1 #(data_width,addr_width) array_gauscore(gauscore_data_in,gauscore_addr,gauscore_en,clk,gauscore_data_out);
ram1 #(data_width,addr_width) array_mgau_sl(mgau_sl_data_in,mgau_sl_addr,sl_en,clk,mgau_sl_data_out);

subvq_mgau_shortlist subvq_mgau_shortlist1(start,clk,rst,n_sv,m,n,beam,			
			map_addr_dim1,map_addr_dim2,map_data_out,
			vqdist_addr,vqdist_data_out,
			gauscore_data_in,gauscore_addr,gauscore_en,gauscore_data_out,
			mgau_sl_data_in,mgau_sl_addr,sl_en,mgau_sl_data_out,
			score_received,score,score_ready,VQ_EVAL);
endmodule

module subvq_mgau_shortlist(start,clk,rst,n_sv,m,n,beam,
			map_addr_dim1,map_addr_dim2,map_data_out,
			vqdist_addr,vqdist_data_out,
			gauscore_data_in,gauscore_addr,gauscore_en,gauscore_data_out,
			mgau_sl_data_in,mgau_sl_addr,sl_en,mgau_sl_data_out,
			score_received,score,score_ready,VQ_EVAL);
parameter data_width=32;
parameter addr_width=32;

input start,clk,rst;
input [data_width-1:0] n_sv;
input [data_width-1:0] m,n,beam;
//------------------------------------From/To Memory----------------------------------------------
output reg [addr_width-1:0] map_addr_dim1,map_addr_dim2,vqdist_addr,gauscore_addr,mgau_sl_addr;
output reg [data_width-1:0] gauscore_data_in,mgau_sl_data_in;
input  [data_width-1:0] map_data_out,vqdist_data_out,gauscore_data_out,mgau_sl_data_out;
output reg gauscore_en,sl_en;
//----------------------------------------------------------------------------------------
input score_received;
input [1:0]VQ_EVAL;

output reg [data_width-1:0] score;
output reg score_ready;
//----------------Memory address---------------------------------------------------------------------------

reg load_map_addr_dim2,load_vqdist_addr,load_gauscore_addr,load_mgau_sl_addr;
reg [addr_width-1:0] temp_map_addr_dim2,temp_vqdist_addr,temp_gauscore_addr,temp_mgau_sl_addr;
wire [addr_width-1:0] o_temp_map_addr_dim2,o_temp_vqdist_addr,o_temp_gauscore_addr,o_temp_mgau_sl_addr;

//------------------------------------------------------------------------------------

reg load_i,load_j,load_k,load_bv,load_v,load_th,load_nc;
reg [data_width-1:0] temp_bv,temp_v,temp_th,temp_nc;
wire [data_width-1:0] bv,v,th,nc;

reg [3:0] temp_i,temp_k;
wire [3:0] i,k;

reg [6:0]temp_j;
wire [6:0]j;


reg load_flag_inc_i;
reg  temp_flag_inc_i,clear_v;
wire flag_inc_i;



reg flag_case3_branch_vq,flag_case3_branch;

reg [8:0]state,next_state;
parameter state_init=8'd0,state_read_map=8'd1,state_read_vqdist=8'd2,state_write_gauscore_case1=8'd3,state_write_gauscore_case2=8'd4,state_update_bv=8'd5,state_inc_i=8'd6,
	state_final_loop_read_gauscore=8'd7,state_final_loop_cmp=8'd8,state_final_loop_write_sl=8'd9,state_final_loop_inc_i=8'd10,state_write_score=8'd11,
	state_write_score_wait=8'd12,state_done=8'd13,state_write_gauscore_case2_2=8'd14,state_write_gauscore_case2_3=8'd15,state_default_write=8'd16;

vdff #(4) i1(clk,load_i,temp_i,i);
vdff #(7) j1(clk,load_j,temp_j,j);
vdff #(4) k1(clk,load_k,temp_k,k);
vdff #(data_width) bv1(clk,load_bv,temp_bv,bv);
vdff #(data_width) v1(clk,load_v,temp_v,v);
vdff #(data_width) th1(clk,load_th,temp_th,th);
vdff #(data_width) nc1(clk,load_nc,temp_nc,nc);

vdff #(1)flag1(clk,load_flag_inc_i,temp_flag_inc_i,flag_inc_i);


	always @(*)
	begin
	//avoid latch. 
	//----------------------------------------------------------------------------	
	map_addr_dim1<=m;map_addr_dim2<=8'b0;vqdist_addr<=8'b0;gauscore_addr<=8'b0;mgau_sl_addr<=8'b0;
	
	gauscore_data_in<=0;mgau_sl_data_in<=0;
	gauscore_en<=0;sl_en<=0;

	load_i<=0;load_j<=0;load_k<=0;load_bv<=0;load_v<=0;load_th<=0;load_nc<=0;
	temp_i<=0;temp_j<=0;temp_k<=0;temp_bv<=0;temp_v<=0;temp_th<=0;temp_nc<=0;
	load_flag_inc_i<=0;
	temp_flag_inc_i<=0;

	score<=0;score_ready<=0;	
	//---------------------------------------------------------------------------	
	case(state)
	state_init:begin
		load_i<=1;load_j<=1;load_k<=1;load_bv<=1;load_v<=1;load_th<=1;load_nc<=1;		
		temp_bv<=32'd2147483648;
		load_flag_inc_i<=1;
		temp_flag_inc_i<=0;	
		flag_case3_branch_vq=0;flag_case3_branch=0;clear_v=0;
		
		next_state<=state_read_map;end	

	state_read_map:begin
		if(clear_v==1)
		load_v<=1;
		if(i<n)begin
		map_addr_dim2<=j;
		next_state<=state_read_vqdist;end
		else begin
		temp_th<=bv+beam;
		load_th<=1;	
		//empty nc,i by enable load(default temp values are always 0)		
		load_nc<=1;
		load_i<=1;	
		load_j<=1;			
		next_state<=state_final_loop_read_gauscore;end end
	
	state_read_vqdist:begin
		vqdist_addr<=map_data_out;			
		
		case(n_sv)
		1:begin temp_j<=j+1;load_j<=1;next_state<=state_write_gauscore_case1;clear_v=0; end
		2:begin temp_j<=j+1;load_j<=1;next_state<=state_write_gauscore_case2; clear_v=0;end
		3:begin 
			if(VQ_EVAL==1)begin
			temp_j<=j+3;load_j<=1;next_state<=state_write_gauscore_case1;clear_v=0;end
			else if(VQ_EVAL==2)begin
			temp_j<=j+1;load_j<=1;next_state<=state_write_gauscore_case2;flag_case3_branch_vq=1;clear_v=0;end
			else begin 
			temp_j<=j+1;load_j<=1; next_state<=state_write_gauscore_case2;flag_case3_branch=1;clear_v=0;end			
		end
		default:begin temp_j<=j+1;load_j<=1;temp_k<=k+1;load_k<=1;next_state<=state_default_write;clear_v=1;end
		endcase
		end
	
	state_default_write:begin
		if(k<n_sv)begin
		temp_v<=vqdist_data_out+v;
		load_v<=1;
		map_addr_dim2<=j;
		next_state<=state_read_vqdist;end
		else begin
		load_k<=1;
		gauscore_addr<=i;		
		gauscore_data_in<=v;
		gauscore_en<=1;			
		next_state<=state_update_bv;end end
	
	state_write_gauscore_case1:begin
		gauscore_addr<=i;		
		gauscore_data_in<=vqdist_data_out;
		gauscore_en<=1;		
		temp_v<=vqdist_data_out;		
		load_v<=1;
		next_state<=state_update_bv;end
	
	state_write_gauscore_case2:begin
		temp_v<=vqdist_data_out;		
		load_v<=1;
		map_addr_dim2<=j;
		next_state<=state_write_gauscore_case2_2;end
		
	state_write_gauscore_case2_2:begin
		vqdist_addr<=map_data_out;	
		temp_j<=j+1;
		load_j<=1;
		next_state<=state_write_gauscore_case2_3;end
		
	state_write_gauscore_case2_3:begin
		temp_v<=(vqdist_data_out<<(flag_case3_branch_vq))+ v;
		load_v<=1;
		
		case(flag_case3_branch)
		0:begin
		gauscore_addr<=i;		
		gauscore_data_in<=(vqdist_data_out<<(flag_case3_branch_vq))+v;
		gauscore_en<=1;
		if(flag_case3_branch_vq!=0)begin
		temp_j<=j+1;
		load_j<=1;
		flag_case3_branch_vq=0;end
		next_state<=state_update_bv;end
		1:begin
		map_addr_dim2<=j;
		next_state<=state_write_gauscore_case2_2;
		flag_case3_branch=0;end endcase  end
	
	state_update_bv:begin
		if(bv<v)begin
		temp_bv<=v;
		load_bv<=1;end
		temp_flag_inc_i<=1;
		load_flag_inc_i<=1;
		next_state<=state_inc_i;end	    
	
	state_final_loop_read_gauscore:begin	
		gauscore_addr<=i;
		next_state<=state_final_loop_cmp;end

	state_final_loop_cmp:begin
		if(i<n)begin
		if (gauscore_data_out>=th)begin
		mgau_sl_addr<=nc;
		mgau_sl_data_in<=i;
		sl_en<=1;
		temp_nc<=nc+1;
		load_nc<=1;
		end
		temp_flag_inc_i<=0;
		load_flag_inc_i<=1;
		next_state<=state_inc_i;end
		else begin
		mgau_sl_addr<=nc;
		mgau_sl_data_in<=-1;
		sl_en<=1;
		next_state<=state_write_score;end end	
	
	state_inc_i:begin
		temp_i<=i+1;
		load_i<=1;
		case(flag_inc_i)
		0:next_state<=state_final_loop_read_gauscore;
		1:next_state<=state_read_map;	
		endcase 
		load_flag_inc_i<=1;end

	state_write_score:begin
		score<=nc;
		score_ready<=1;
		if(score_received)
		next_state<=state_done;
		else
		next_state<=state_write_score;end
		
	state_done:begin
		if(start)
		next_state<=state_init;
		else
		next_state<=state_done;end
	endcase
	end
		
	always @(posedge clk) begin 
	  if (rst) begin 
	    state <= state_init;   // Initial state 
	  end else begin 
	    state <= next_state; 
	  end 
	end
endmodule 	

module vdff(clk,we,data,q);
parameter data_width=32;
input clk,we;
input [data_width-1:0] data;
output [data_width-1:0] q;

reg [data_width-1:0] q_r;
always @ (posedge clk)
begin
	if(we)
	q_r<=data;
end
assign q=q_r;
endmodule	


module ram_map (data,addr,we,clk,q);
parameter data_width=32;
parameter addr_width=8;
input [data_width-1:0] data;
input [addr_width-1:0] addr;
input we, clk;
output [data_width-1:0] q;

integer mymap [0:95];
reg [addr_width-1:0] addr_reg;	
initial begin
mymap[0]=5;
mymap[1]=3;
mymap[2]=7;
mymap[3]=9;
mymap[4]=6;
mymap[5]=1;
mymap[6]=8;
mymap[7]=5;
mymap[8]=1;
mymap[9]=0;
mymap[10]=7;
mymap[11]=9;
mymap[12]=12;
mymap[13]=19;
mymap[14]=23;
mymap[15]=25;
mymap[16]=28;
mymap[17]=9;
mymap[18]=28;
mymap[19]=26;
mymap[20]=10;
mymap[21]=7;
mymap[22]=15;
mymap[23]=11;         
mymap[24]=5;
mymap[25]=3;
mymap[26]=7;
mymap[27]=9;
mymap[28]=6;
mymap[29]=1;
mymap[30]=8;
mymap[31]=25;
mymap[32]=28;
mymap[33]=9;
mymap[34]=28;
mymap[35]=26;
mymap[36]=10;
mymap[37]=7;
mymap[38]=15;
mymap[39]=11;
mymap[40]=5;
mymap[41]=1;
mymap[42]=0;
mymap[43]=7;
mymap[44]=9;
mymap[45]=12;
mymap[46]=19;
mymap[47]=23;
mymap[48]=28;
mymap[49]=9;
mymap[50]=28;
mymap[51]=26;
mymap[52]=10;
mymap[53]=7;
mymap[54]=15;
mymap[55]=5;
mymap[56]=1;
mymap[57]=0;
mymap[58]=7;
mymap[59]=9;
mymap[60]=12;
mymap[61]=19;
mymap[62]=23;
mymap[63]=5;
mymap[64]=1;
mymap[65]=0;
mymap[66]=7;
mymap[67]=9;
mymap[68]=18;
mymap[69]=15;
mymap[70]=22;
mymap[71]=26;


end

always @ (posedge clk)
begin
	if (we)begin
	mymap[addr] <= data;
	end	
	addr_reg <= addr;	
	end	
assign q = mymap[addr_reg];	
endmodule 


module ram1(data,addr,we,clk,q);
parameter data_width=32;
parameter addr_width=8;
input [data_width-1:0] data;
input [addr_width-1:0] addr;
input we, clk;
output [data_width-1:0] q;

	
// Variable to hold the registered read address
reg [addr_width-1:0] addr_reg;
reg [data_width-1:0] ram[0:8];

initial begin

ram[0]=0;
ram[1]=0;
ram[2]=0;
ram[3]=0;
ram[4]=0;
ram[5]=0;
ram[6]=0;
ram[7]=0;
ram[8]=0;
end	
always @ (posedge clk)
begin
	if (we)begin
	ram[addr] <= data;
	end	
	addr_reg <= addr;		
	end
assign q = ram[addr_reg];
endmodule



module ram_vqdist(data,addr,we,clk,q);
parameter data_width=32;
parameter addr_width=8;

input [data_width-1:0] data;
input [addr_width-1:0] addr;
input we, clk;
output [data_width-1:0] q;

// Declare the RAM variable
integer ram[0:28];	
// Variable to hold the registered read address
reg [addr_width-1:0] addr_reg;

initial begin
ram[0]=-32'd71473;
ram[1]=-32'd120699;
ram[2]=-32'd178511;
ram[3]=-32'd406707;
ram[4]=-32'd408780;
ram[5]=-32'd208568;
ram[6]=-32'd133468;
ram[7]=-32'd125057;
ram[8]=-32'd285419;
ram[9]=-32'd89299;
ram[10]=-32'd78179;
ram[11]=-32'd395027;
ram[12]=-32'd370001;
ram[13]=-32'd147657;
ram[14]=-32'd128752;
ram[15]=-32'd83480;
ram[16]=-32'd235287;
ram[17]=-32'd89939;
ram[18]=-32'd203729;
ram[19]=-32'd140722;
ram[20]=-32'd426228;
ram[21]=-32'd102132;
ram[22]=-32'd81783;
ram[23]=-32'd117958;
ram[24]=-32'd105634;
ram[25]=-32'd269504;
ram[26]=-32'd130565;
ram[27]=-32'd212597;
ram[28]=-32'd80666;
end
always @ (posedge clk)
begin
	
	if (we)begin
	ram[addr] <= data;
	end	
	addr_reg <= addr;		
	end
assign q = ram[addr_reg];
endmodule
