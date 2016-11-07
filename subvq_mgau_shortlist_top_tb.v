module subvq_mgau_shortlist_top_tb();
parameter data_width=32;
parameter address_width=8;
reg start,clk=0,rst;
reg [data_width-1:0] n_sv;
reg [data_width-1:0] m,n,beam;
reg score_received;

wire [data_width-1:0] score;
wire score_ready;
reg clk2=0;
reg [1:0]VQ_EVAL;

subvq_mgau_shortlist_top dut(start,clk,rst,n_sv,m,n,beam,score_received,score,score_ready,VQ_EVAL);

	always begin
	#2 clk=~clk;
	end
always begin
	#10 clk2=~clk2;
	end
	initial begin
	start=0;
	rst=1;
	n_sv=3;
	m=0;
	n=8;
	beam=-153503;
	score_received=0;
	VQ_EVAL=0;
	#40;
	rst=0;
	#420;

	rst=1;
	m=1;
	#40;
	rst=0;
	#420;

	rst=1;
	m=2;
	#40;
	rst=0;
	#420;
	VQ_EVAL=1;
	rst=1;
	m=0;
	#40;
	rst=0;
	#420;
	VQ_EVAL=1;
	rst=1;
	m=1;
	#40;
	rst=0;
	#420;
	VQ_EVAL=1;
	rst=1;
	m=2;
	#40;
	rst=0;
	#420;
	$stop;
	end
	

endmodule;