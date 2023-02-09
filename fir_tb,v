`timescale 1ns / 1ps

module fir_tb ();

reg                  i_clk_tb      ; 
reg                  i_rst_n_tb    ;
reg                  i_en_buff_tb  ;
reg                  i_en_fir_tb   ;
reg   signed  [15:0] i_fir_data_tb ;
wire  signed  [31:0] o_fir_data_tb ;

parameter CLK_PERIOD  = 5'd20 ;  // 50 MHZ (20 ns) FPGA Clock 
parameter SIG1_PERIOD = 5'd16 ;  // 100 KHZ input signal 
parameter SIG2_PERIOD = 4'd8  ;  // 200 KHZ input signal
parameter SIG3_PERIOD = 3'd4  ;  // 400 KHZ input signal
parameter ZERO        = 1'd0  ;
parameter SAMPLE_0    = 1'd1  ;
parameter SAMPLE_1    = 2'd2  ;
parameter SAMPLE_2    = 2'd3  ;
parameter SAMPLE_3    = 3'd4  ;
parameter SAMPLE_4    = 3'd5  ;
parameter SAMPLE_5    = 3'd6  ;
parameter SAMPLE_6    = 3'd7  ;
parameter SAMPLE_7    = 4'd8  ;

reg   [4:0]    counter ;
reg   [3:0]    state   ;


fir U_fir (.i_clk      (i_clk_tb     )  ,       
           .i_rst_n    (i_rst_n_tb   )  , 
           .i_en_buff  (i_en_buff_tb )  , 
           .i_en_fir   (i_en_fir_tb  )  , 
           .i_fir_data (i_fir_data_tb)  , 
           .o_fir_data (o_fir_data_tb) ); 



// clock generation 

initial 
  begin 
                          i_clk_tb = 1'b0 ;

    forever      
    #(CLK_PERIOD * 0.5)   i_clk_tb = ~i_clk_tb; 

  end                  

// reset and initiallization 

initial 
  begin 
               i_rst_n_tb = 1'b1;
    #40
               i_rst_n_tb = 1'b0;
    #80        
               i_rst_n_tb = 1'b1;

  end 

initial 
  begin
    #1000000
               i_en_buff_tb  = 1'b0  ;
               i_en_fir_tb   = 1'b0  ;
               i_fir_data_tb = 16'b0 ;
    #100
               i_en_buff_tb  = 1'b1  ;
               i_en_fir_tb   = 1'b1  ;
  end 

always @(posedge i_clk_tb or negedge i_rst_n_tb)
  begin: sine_wave_generation
    if(!i_rst_n_tb)
      begin 
        state         <= ZERO  ;
        counter       <= 5'b0  ;
        i_fir_data_tb <= 16'b0 ; 
      end 
    else 
      begin 
        case (state)
          ZERO : begin 
                   counter       <= 5'b0     ;
                   state         <= SAMPLE_0 ;
                   i_fir_data_tb <= 16'h0000 ;
                 end 

          SAMPLE_0 : begin 
                       i_fir_data_tb <= 16'h0000 ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_1 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_0       ;
                         end 
                     end 

          SAMPLE_1 : begin
                       i_fir_data_tb <= 16'h5A7E ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_2 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_1       ;
                         end 
                     end 

          SAMPLE_2 : begin
                       i_fir_data_tb <= 16'h7FFF ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_3 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_2       ;
                         end 
                     end 

          SAMPLE_3 : begin
                       i_fir_data_tb <= 16'h5A7E ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_4 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_3       ;
                         end 
                     end 

          SAMPLE_4 : begin
                       i_fir_data_tb <= 16'h0000 ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_5 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_4       ;
                         end 
                     end 

          SAMPLE_5 : begin
                       i_fir_data_tb <= 16'hA582 ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_6 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_5       ;
                         end 
                     end 

          SAMPLE_6 : begin
                       i_fir_data_tb <= 16'h8000 ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_7 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_6       ;
                         end 
                     end 

          SAMPLE_7 : begin
                       i_fir_data_tb <= 16'hA582 ;
                       counter <= counter + 1'b1 ;

                       if (counter == SIG1_PERIOD) 
                         begin
                           counter <= 5'b0     ;
                           state   <= SAMPLE_0 ;
                         end 
                       else 
                         begin 
                           counter <= counter + 1'b1 ;
                           state   <= SAMPLE_7       ;
                         end 
                     end
        endcase 
    end 
  end 

endmodule 

