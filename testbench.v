module tb ();
    reg clk;
    reg rstn;
    reg [15:0] bus;

  	datapath datapath1 (clk, rstn, bus);

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rstn = 0;
      
        #10 rstn = 1;
        #100 $finish;
    end

endmodule