/////////////////////////////////////////////
//////////////
///////////////////////////

module fir (
	        input  wire                   i_clk       ,  //system clock 50 MHZ
	        input  wire                   i_rst_n     ,  //asynch neg edge restet
	        input  wire                   i_en_buff   ,  //input buffer enable
	        input  wire                   i_en_fir    ,  //fir delay buffers enable 
	        input  wire  signed  [15:0]   i_fir_data  ,  //input data 
	        output reg   signed  [31:0]   o_fir_data );  //output data 


//////// Impulse Response (Filter Coeeficients) //////////

parameter H_0  = 16'hFFF2;
parameter H_1  = 16'h000F;
parameter H_2  = 16'h001C;
parameter H_3  = 16'h0000;
parameter H_4  = 16'hFFDA;
parameter H_5  = 16'hFFE4;
parameter H_6  = 16'h0022;
parameter H_7  = 16'h0044;
parameter H_8  = 16'h0000;
parameter H_9  = 16'hFF9E;
parameter H_10 = 16'hFFB8;
parameter H_11 = 16'h0056;
parameter H_12 = 16'h00A4;
parameter H_13 = 16'h0000;
parameter H_14 = 16'hFF1F;
parameter H_15 = 16'hFF5F;
parameter H_16 = 16'h00BA;
parameter H_17 = 16'h015C;
parameter H_18 = 16'h0000;
parameter H_19 = 16'hFE35;
parameter H_20 = 16'hFEBB;
parameter H_21 = 16'h0175;
parameter H_22 = 16'h02B6;
parameter H_23 = 16'h0000;
parameter H_24 = 16'hFC62;
parameter H_25 = 16'hFD64;
parameter H_26 = 16'h0315;
parameter H_27 = 16'h05FB;
parameter H_28 = 16'h0000;
parameter H_29 = 16'hF6A3;
parameter H_30 = 16'hF82C;
parameter H_31 = 16'h0BDE;
parameter H_32 = 16'h26A7;
parameter H_33 = 16'h332E;
parameter H_34 = 16'h26A7;
parameter H_35 = 16'h0BDE;
parameter H_36 = 16'hF82C;
parameter H_37 = 16'hF6A3;
parameter H_38 = 16'h0000;
parameter H_39 = 16'h05FB;
parameter H_40 = 16'h0315;
parameter H_41 = 16'hFD64;
parameter H_42 = 16'hFC62;
parameter H_43 = 16'h0000;
parameter H_44 = 16'h02B6;
parameter H_45 = 16'h0175;
parameter H_46 = 16'hFEBB;
parameter H_47 = 16'hFE35;
parameter H_48 = 16'h0000;
parameter H_49 = 16'h015C;
parameter H_50 = 16'h00BA;
parameter H_51 = 16'hFF5F;
parameter H_52 = 16'hFF1F;
parameter H_53 = 16'h0000;
parameter H_54 = 16'h00A4;
parameter H_55 = 16'h0056;
parameter H_56 = 16'hFFB8;
parameter H_57 = 16'hFF9E;
parameter H_58 = 16'h0000;
parameter H_59 = 16'h0044;
parameter H_60 = 16'h0022;
parameter H_61 = 16'hFFE4;
parameter H_62 = 16'hFFDA;
parameter H_63 = 16'h0000;
parameter H_64 = 16'h001C;
parameter H_65 = 16'h000F;
parameter H_66 = 16'hFFF2;


///////// Internal wires,regs Declarations //////////

reg                [6:0]    buff_count  ;
reg    signed      [15:0]   in_sample   ;

// z_transform buffers
reg    signed      [15:0]   buff_0      ;
reg    signed      [15:0]   buff_1      ;
reg    signed      [15:0]   buff_2      ;
reg    signed      [15:0]   buff_3      ;
reg    signed      [15:0]   buff_4      ;
reg    signed      [15:0]   buff_5      ;
reg    signed      [15:0]   buff_6      ;
reg    signed      [15:0]   buff_7      ;
reg    signed      [15:0]   buff_8      ;
reg    signed      [15:0]   buff_9      ;
reg    signed      [15:0]   buff_10     ;
reg    signed      [15:0]   buff_11     ;
reg    signed      [15:0]   buff_12     ;
reg    signed      [15:0]   buff_13     ;
reg    signed      [15:0]   buff_14     ;
reg    signed      [15:0]   buff_15     ;
reg    signed      [15:0]   buff_16     ;
reg    signed      [15:0]   buff_17     ;
reg    signed      [15:0]   buff_18     ;
reg    signed      [15:0]   buff_19     ;
reg    signed      [15:0]   buff_20     ;
reg    signed      [15:0]   buff_21     ;
reg    signed      [15:0]   buff_22     ;
reg    signed      [15:0]   buff_23     ;
reg    signed      [15:0]   buff_24     ;
reg    signed      [15:0]   buff_25     ;
reg    signed      [15:0]   buff_26     ;
reg    signed      [15:0]   buff_27     ;
reg    signed      [15:0]   buff_28     ;
reg    signed      [15:0]   buff_29     ;
reg    signed      [15:0]   buff_30     ;
reg    signed      [15:0]   buff_31     ;
reg    signed      [15:0]   buff_32     ;
reg    signed      [15:0]   buff_33     ;
reg    signed      [15:0]   buff_34     ;
reg    signed      [15:0]   buff_35     ;
reg    signed      [15:0]   buff_36     ;
reg    signed      [15:0]   buff_37     ;
reg    signed      [15:0]   buff_38     ;
reg    signed      [15:0]   buff_39     ;
reg    signed      [15:0]   buff_40     ;
reg    signed      [15:0]   buff_41     ;
reg    signed      [15:0]   buff_42     ;
reg    signed      [15:0]   buff_43     ;
reg    signed      [15:0]   buff_44     ;
reg    signed      [15:0]   buff_45     ;
reg    signed      [15:0]   buff_46     ;
reg    signed      [15:0]   buff_47     ;
reg    signed      [15:0]   buff_48     ;
reg    signed      [15:0]   buff_49     ;
reg    signed      [15:0]   buff_50     ;
reg    signed      [15:0]   buff_51     ;
reg    signed      [15:0]   buff_52     ;
reg    signed      [15:0]   buff_53     ;
reg    signed      [15:0]   buff_54     ;
reg    signed      [15:0]   buff_55     ;
reg    signed      [15:0]   buff_56     ;
reg    signed      [15:0]   buff_57     ;
reg    signed      [15:0]   buff_58     ;
reg    signed      [15:0]   buff_59     ;
reg    signed      [15:0]   buff_60     ;
reg    signed      [15:0]   buff_61     ;
reg    signed      [15:0]   buff_62     ;
reg    signed      [15:0]   buff_63     ;
reg    signed      [15:0]   buff_64     ;
reg    signed      [15:0]   buff_65     ;
reg    signed      [15:0]   buff_66     ;

// impulse response filter taps (coeff)
wire   signed      [15:0]   tap_0       ;
wire   signed      [15:0]   tap_1       ;
wire   signed      [15:0]   tap_2       ;
wire   signed      [15:0]   tap_3       ;
wire   signed      [15:0]   tap_4       ;
wire   signed      [15:0]   tap_5       ;
wire   signed      [15:0]   tap_6       ;
wire   signed      [15:0]   tap_7       ;
wire   signed      [15:0]   tap_8       ;
wire   signed      [15:0]   tap_9       ;
wire   signed      [15:0]   tap_10      ;
wire   signed      [15:0]   tap_11      ;
wire   signed      [15:0]   tap_12      ;
wire   signed      [15:0]   tap_13      ;
wire   signed      [15:0]   tap_14      ;
wire   signed      [15:0]   tap_15      ;
wire   signed      [15:0]   tap_16      ;
wire   signed      [15:0]   tap_17      ;
wire   signed      [15:0]   tap_18      ;
wire   signed      [15:0]   tap_19      ;
wire   signed      [15:0]   tap_20      ;
wire   signed      [15:0]   tap_21      ;
wire   signed      [15:0]   tap_22      ;
wire   signed      [15:0]   tap_23      ;
wire   signed      [15:0]   tap_24      ;
wire   signed      [15:0]   tap_25      ;
wire   signed      [15:0]   tap_26      ;
wire   signed      [15:0]   tap_27      ;
wire   signed      [15:0]   tap_28      ;
wire   signed      [15:0]   tap_29      ;
wire   signed      [15:0]   tap_30      ;
wire   signed      [15:0]   tap_31      ;
wire   signed      [15:0]   tap_32      ;
wire   signed      [15:0]   tap_33      ;
wire   signed      [15:0]   tap_34      ;
wire   signed      [15:0]   tap_35      ;
wire   signed      [15:0]   tap_36      ;
wire   signed      [15:0]   tap_37      ;
wire   signed      [15:0]   tap_38      ;
wire   signed      [15:0]   tap_39      ;
wire   signed      [15:0]   tap_40      ;
wire   signed      [15:0]   tap_41      ;
wire   signed      [15:0]   tap_42      ;
wire   signed      [15:0]   tap_43      ;
wire   signed      [15:0]   tap_44      ;
wire   signed      [15:0]   tap_45      ;
wire   signed      [15:0]   tap_46      ;
wire   signed      [15:0]   tap_47      ;
wire   signed      [15:0]   tap_48      ;
wire   signed      [15:0]   tap_49      ;
wire   signed      [15:0]   tap_50      ;
wire   signed      [15:0]   tap_51      ;
wire   signed      [15:0]   tap_52      ;
wire   signed      [15:0]   tap_53      ;
wire   signed      [15:0]   tap_54      ;
wire   signed      [15:0]   tap_55      ;
wire   signed      [15:0]   tap_56      ;
wire   signed      [15:0]   tap_57      ;
wire   signed      [15:0]   tap_58      ;
wire   signed      [15:0]   tap_59      ;
wire   signed      [15:0]   tap_60      ;
wire   signed      [15:0]   tap_61      ;
wire   signed      [15:0]   tap_62      ;
wire   signed      [15:0]   tap_63      ;
wire   signed      [15:0]   tap_64      ;
wire   signed      [15:0]   tap_65      ;
wire   signed      [15:0]   tap_66      ;

// summing outputs 
reg    signed      [31:0]   accum_0     ;
reg    signed      [31:0]   accum_1     ;
reg    signed      [31:0]   accum_2     ;
reg    signed      [31:0]   accum_3     ;
reg    signed      [31:0]   accum_4     ;
reg    signed      [31:0]   accum_5     ;
reg    signed      [31:0]   accum_6     ;
reg    signed      [31:0]   accum_7     ;
reg    signed      [31:0]   accum_8     ;
reg    signed      [31:0]   accum_9     ;
reg    signed      [31:0]   accum_10    ;
reg    signed      [31:0]   accum_11    ;
reg    signed      [31:0]   accum_12    ;
reg    signed      [31:0]   accum_13    ;
reg    signed      [31:0]   accum_14    ;
reg    signed      [31:0]   accum_15    ;
reg    signed      [31:0]   accum_16    ;
reg    signed      [31:0]   accum_17    ;
reg    signed      [31:0]   accum_18    ;
reg    signed      [31:0]   accum_19    ;
reg    signed      [31:0]   accum_20    ;
reg    signed      [31:0]   accum_21    ;
reg    signed      [31:0]   accum_22    ;
reg    signed      [31:0]   accum_23    ;
reg    signed      [31:0]   accum_24    ;
reg    signed      [31:0]   accum_25    ;
reg    signed      [31:0]   accum_26    ;
reg    signed      [31:0]   accum_27    ;
reg    signed      [31:0]   accum_28    ;
reg    signed      [31:0]   accum_29    ;
reg    signed      [31:0]   accum_30    ;
reg    signed      [31:0]   accum_31    ;
reg    signed      [31:0]   accum_32    ;
reg    signed      [31:0]   accum_33    ;
reg    signed      [31:0]   accum_34    ;
reg    signed      [31:0]   accum_35    ;
reg    signed      [31:0]   accum_36    ;
reg    signed      [31:0]   accum_37    ;
reg    signed      [31:0]   accum_38    ;
reg    signed      [31:0]   accum_39    ;
reg    signed      [31:0]   accum_40    ;
reg    signed      [31:0]   accum_41    ;
reg    signed      [31:0]   accum_42    ;
reg    signed      [31:0]   accum_43    ;
reg    signed      [31:0]   accum_44    ;
reg    signed      [31:0]   accum_45    ;
reg    signed      [31:0]   accum_46    ;
reg    signed      [31:0]   accum_47    ;
reg    signed      [31:0]   accum_48    ;
reg    signed      [31:0]   accum_49    ;
reg    signed      [31:0]   accum_50    ;
reg    signed      [31:0]   accum_51    ;
reg    signed      [31:0]   accum_52    ;
reg    signed      [31:0]   accum_53    ;
reg    signed      [31:0]   accum_54    ;
reg    signed      [31:0]   accum_55    ;
reg    signed      [31:0]   accum_56    ;
reg    signed      [31:0]   accum_57    ;
reg    signed      [31:0]   accum_58    ;
reg    signed      [31:0]   accum_59    ;
reg    signed      [31:0]   accum_60    ;
reg    signed      [31:0]   accum_61    ;
reg    signed      [31:0]   accum_62    ;
reg    signed      [31:0]   accum_63    ;
reg    signed      [31:0]   accum_64    ;
reg    signed      [31:0]   accum_65    ;
reg    signed      [31:0]   accum_66    ;


//////////// filter taps assignments ////////////

assign tap_0  = H_0  ;
assign tap_1  = H_1  ;
assign tap_2  = H_2  ;
assign tap_3  = H_3  ;
assign tap_4  = H_4  ;
assign tap_5  = H_5  ;
assign tap_6  = H_6  ;
assign tap_7  = H_7  ;
assign tap_8  = H_8  ;
assign tap_9  = H_9  ;
assign tap_10 = H_10 ;
assign tap_11 = H_11 ;
assign tap_12 = H_12 ;
assign tap_13 = H_13 ;
assign tap_14 = H_14 ;
assign tap_15 = H_15 ;
assign tap_16 = H_16 ;
assign tap_17 = H_17 ;
assign tap_18 = H_18 ;
assign tap_19 = H_19 ;
assign tap_20 = H_20 ;
assign tap_21 = H_21 ;
assign tap_22 = H_22 ;
assign tap_23 = H_23 ;
assign tap_24 = H_24 ;
assign tap_25 = H_25 ;
assign tap_26 = H_26 ;
assign tap_27 = H_27 ;
assign tap_28 = H_28 ;
assign tap_29 = H_29 ;
assign tap_30 = H_30 ;
assign tap_31 = H_31 ;
assign tap_32 = H_32 ;
assign tap_33 = H_33 ;
assign tap_34 = H_34 ;
assign tap_35 = H_35 ;
assign tap_36 = H_36 ;
assign tap_37 = H_37 ;
assign tap_38 = H_38 ;
assign tap_39 = H_39 ;
assign tap_40 = H_40 ;
assign tap_41 = H_41 ;
assign tap_42 = H_42 ;
assign tap_43 = H_43 ;
assign tap_44 = H_44 ;
assign tap_45 = H_45 ;
assign tap_46 = H_46 ;
assign tap_47 = H_47 ;
assign tap_48 = H_48 ;
assign tap_49 = H_49 ;
assign tap_50 = H_50 ;
assign tap_51 = H_51 ;
assign tap_52 = H_52 ;
assign tap_53 = H_53 ;
assign tap_54 = H_54 ;
assign tap_55 = H_55 ;
assign tap_56 = H_56 ;
assign tap_57 = H_57 ;
assign tap_58 = H_58 ;
assign tap_59 = H_59 ;
assign tap_60 = H_60 ;
assign tap_61 = H_61 ;
assign tap_62 = H_62 ;
assign tap_63 = H_63 ;
assign tap_64 = H_64 ;
assign tap_65 = H_65 ;
assign tap_66 = H_66 ;


//////////// buffers counter and input samples ////////////

always @(posedge i_clk or negedge i_rst_n) 
  begin: counter_input_buffer 
    if (!i_rst_n) 
      begin 
        buff_count <= 7'b0  ;
        in_sample  <= 16'b0 ;
      end 
    else 
      begin
        if (buff_count == 7'd67)
          begin 
            buff_count <= 4'b0       ;
            in_sample  <= i_fir_data ;
          end 
        else 
          begin 
            buff_count <= buff_count + 1'b1 ;
            in_sample  <= i_fir_data        ;
          end 
      end 
  end 

//////////// buffers block ////////////

always @(posedge i_clk or negedge i_rst_n) 
  begin: z_transform_buffers_block 
    if (!i_rst_n) 
      begin 
        buff_0  <= 16'b0 ;
        buff_1  <= 16'b0 ;
        buff_2  <= 16'b0 ;
        buff_3  <= 16'b0 ;
        buff_4  <= 16'b0 ;
        buff_5  <= 16'b0 ;
        buff_6  <= 16'b0 ;
        buff_7  <= 16'b0 ;
        buff_8  <= 16'b0 ;
        buff_9  <= 16'b0 ;
        buff_10 <= 16'b0 ;
        buff_11 <= 16'b0 ;
        buff_12 <= 16'b0 ;
        buff_13 <= 16'b0 ;
        buff_14 <= 16'b0 ;
        buff_15 <= 16'b0 ;
        buff_16 <= 16'b0 ;
        buff_17 <= 16'b0 ;
        buff_18 <= 16'b0 ;
        buff_19 <= 16'b0 ;
        buff_20 <= 16'b0 ;
        buff_21 <= 16'b0 ;
        buff_22 <= 16'b0 ;
        buff_23 <= 16'b0 ;
        buff_24 <= 16'b0 ;
        buff_25 <= 16'b0 ;
        buff_26 <= 16'b0 ;
        buff_27 <= 16'b0 ;
        buff_28 <= 16'b0 ;
        buff_29 <= 16'b0 ;
        buff_30 <= 16'b0 ;
        buff_31 <= 16'b0 ;
        buff_32 <= 16'b0 ;
        buff_33 <= 16'b0 ;
        buff_34 <= 16'b0 ;
        buff_35 <= 16'b0 ;
        buff_36 <= 16'b0 ;
        buff_37 <= 16'b0 ;
        buff_38 <= 16'b0 ;
        buff_39 <= 16'b0 ;
        buff_40 <= 16'b0 ;
        buff_41 <= 16'b0 ;
        buff_42 <= 16'b0 ;
        buff_43 <= 16'b0 ;
        buff_44 <= 16'b0 ;
        buff_45 <= 16'b0 ;
        buff_46 <= 16'b0 ;
        buff_47 <= 16'b0 ;
        buff_48 <= 16'b0 ;
        buff_49 <= 16'b0 ;
        buff_50 <= 16'b0 ;
        buff_51 <= 16'b0 ;
        buff_52 <= 16'b0 ;
        buff_53 <= 16'b0 ;
        buff_54 <= 16'b0 ;
        buff_55 <= 16'b0 ;
        buff_56 <= 16'b0 ;
        buff_57 <= 16'b0 ;
        buff_58 <= 16'b0 ;
        buff_59 <= 16'b0 ;
        buff_60 <= 16'b0 ;
        buff_61 <= 16'b0 ;
        buff_62 <= 16'b0 ;
        buff_63 <= 16'b0 ;
        buff_64 <= 16'b0 ;
        buff_65 <= 16'b0 ;
        buff_66 <= 16'b0 ;
      end 
    else 
      begin 
        if (i_en_buff) 
          begin
            buff_0  <= in_sample ;
            buff_1  <= buff_0    ;
            buff_2  <= buff_1    ;
            buff_3  <= buff_2    ;
            buff_4  <= buff_3    ;
            buff_5  <= buff_4    ;
            buff_6  <= buff_5    ;
            buff_7  <= buff_6    ;
            buff_8  <= buff_7    ;
            buff_9  <= buff_8    ;
            buff_10 <= buff_9    ;
            buff_11 <= buff_10   ;
            buff_12 <= buff_11   ;
            buff_13 <= buff_12   ;
            buff_14 <= buff_13   ;
            buff_15 <= buff_14   ;
            buff_16 <= buff_15   ;
            buff_17 <= buff_16   ;
            buff_18 <= buff_17   ;
            buff_19 <= buff_18   ;
            buff_20 <= buff_19   ;
            buff_21 <= buff_20   ;
            buff_22 <= buff_21   ;
            buff_23 <= buff_22   ;
            buff_24 <= buff_23   ;
            buff_25 <= buff_24   ;
            buff_26 <= buff_25   ;
            buff_27 <= buff_26   ;
            buff_28 <= buff_27   ;
            buff_29 <= buff_28   ;
            buff_30 <= buff_29   ;
            buff_31 <= buff_30   ;
            buff_32 <= buff_31   ;
            buff_33 <= buff_32   ;
            buff_34 <= buff_33   ;
            buff_35 <= buff_34   ;
            buff_36 <= buff_35   ;
            buff_37 <= buff_36   ;
            buff_38 <= buff_37   ;
            buff_39 <= buff_38   ;
            buff_40 <= buff_39   ;
            buff_41 <= buff_40   ;
            buff_42 <= buff_41   ;
            buff_43 <= buff_42   ;
            buff_44 <= buff_43   ;
            buff_45 <= buff_44   ;
            buff_46 <= buff_45   ;
            buff_47 <= buff_46   ;
            buff_48 <= buff_47   ;
            buff_49 <= buff_48   ;
            buff_50 <= buff_49   ;
            buff_51 <= buff_50   ;
            buff_52 <= buff_51   ;
            buff_53 <= buff_52   ;
            buff_54 <= buff_53   ;
            buff_55 <= buff_54   ;
            buff_56 <= buff_55   ;
            buff_57 <= buff_56   ;
            buff_58 <= buff_57   ;
            buff_59 <= buff_58   ;
            buff_60 <= buff_59   ;
            buff_61 <= buff_60   ;
            buff_62 <= buff_61   ;
            buff_63 <= buff_62   ;
            buff_64 <= buff_63   ;
            buff_65 <= buff_64   ;
            buff_66 <= buff_65   ;
          end 
        else 
          begin 
            buff_0  <= buff_0  ;
            buff_1  <= buff_1  ;
            buff_2  <= buff_2  ;
            buff_3  <= buff_3  ;
            buff_4  <= buff_4  ;
            buff_5  <= buff_5  ;
            buff_6  <= buff_6  ;
            buff_7  <= buff_7  ;
            buff_8  <= buff_8  ;
            buff_9  <= buff_9  ;
            buff_10 <= buff_10 ;
            buff_11 <= buff_11 ;
            buff_12 <= buff_12 ;
            buff_13 <= buff_13 ;
            buff_14 <= buff_14 ;
            buff_15 <= buff_15 ;
            buff_16 <= buff_16 ;
            buff_17 <= buff_17 ;
            buff_18 <= buff_18 ;
            buff_19 <= buff_19 ;
            buff_20 <= buff_20 ;
            buff_21 <= buff_21 ;
            buff_22 <= buff_22 ;
            buff_23 <= buff_23 ;
            buff_24 <= buff_24 ;
            buff_25 <= buff_25 ;
            buff_26 <= buff_26 ;
            buff_27 <= buff_27 ;
            buff_28 <= buff_28 ;
            buff_29 <= buff_29 ;
            buff_30 <= buff_30 ;
            buff_31 <= buff_31 ;
            buff_32 <= buff_32 ;
            buff_33 <= buff_33 ;
            buff_34 <= buff_34 ;
            buff_35 <= buff_35 ;
            buff_36 <= buff_36 ;
            buff_37 <= buff_37 ;
            buff_38 <= buff_38 ;
            buff_39 <= buff_39 ;
            buff_40 <= buff_40 ;
            buff_41 <= buff_41 ;
            buff_42 <= buff_42 ;
            buff_43 <= buff_43 ;
            buff_44 <= buff_44 ;
            buff_45 <= buff_45 ;
            buff_46 <= buff_46 ;
            buff_47 <= buff_47 ;
            buff_48 <= buff_48 ;
            buff_49 <= buff_49 ;
            buff_50 <= buff_50 ;
            buff_51 <= buff_51 ;
            buff_52 <= buff_52 ;
            buff_53 <= buff_53 ;
            buff_54 <= buff_54 ;
            buff_55 <= buff_55 ;
            buff_56 <= buff_56 ;
            buff_57 <= buff_57 ;
            buff_58 <= buff_58 ;
            buff_59 <= buff_59 ;
            buff_60 <= buff_60 ;
            buff_61 <= buff_61 ;
            buff_62 <= buff_62 ;
            buff_63 <= buff_63 ;
            buff_64 <= buff_64 ;
            buff_65 <= buff_65 ;
            buff_66 <= buff_66 ;
          end 
      end 
  end             

//////////// impulse response ////////////

always @(posedge i_clk) 
  begin: impulse_response_filter_coeef_block 
    if (i_en_fir) 
      begin
        accum_0  <= tap_0  * buff_0  ;
        accum_1  <= tap_1  * buff_1  ;
        accum_2  <= tap_2  * buff_2  ;
        accum_3  <= tap_3  * buff_3  ;
        accum_4  <= tap_4  * buff_4  ;
        accum_5  <= tap_5  * buff_5  ;
        accum_6  <= tap_6  * buff_6  ;
        accum_7  <= tap_7  * buff_7  ;
        accum_8  <= tap_8  * buff_8  ;
        accum_9  <= tap_9  * buff_9  ;
        accum_10 <= tap_10 * buff_10 ;
        accum_11 <= tap_11 * buff_11 ;
        accum_12 <= tap_12 * buff_12 ;
        accum_13 <= tap_13 * buff_13 ;
        accum_14 <= tap_14 * buff_14 ;
        accum_15 <= tap_15 * buff_15 ;
        accum_16 <= tap_16 * buff_16 ;
        accum_17 <= tap_17 * buff_17 ;
        accum_18 <= tap_18 * buff_18 ;
        accum_19 <= tap_19 * buff_19 ;
        accum_20 <= tap_20 * buff_20 ;
        accum_21 <= tap_21 * buff_21 ;
        accum_22 <= tap_22 * buff_22 ;
        accum_23 <= tap_23 * buff_23 ;
        accum_24 <= tap_24 * buff_24 ;
        accum_25 <= tap_25 * buff_25 ;
        accum_26 <= tap_26 * buff_26 ;
        accum_27 <= tap_27 * buff_27 ;
        accum_28 <= tap_28 * buff_28 ;
        accum_29 <= tap_29 * buff_29 ;
        accum_30 <= tap_30 * buff_30 ;
        accum_31 <= tap_31 * buff_31 ;
        accum_32 <= tap_32 * buff_32 ;
        accum_33 <= tap_33 * buff_33 ;
        accum_34 <= tap_34 * buff_34 ;
        accum_35 <= tap_35 * buff_35 ;
        accum_36 <= tap_36 * buff_36 ;
        accum_37 <= tap_37 * buff_37 ;
        accum_38 <= tap_38 * buff_38 ;
        accum_39 <= tap_39 * buff_39 ;
        accum_40 <= tap_40 * buff_40 ;
        accum_41 <= tap_41 * buff_41 ;
        accum_42 <= tap_42 * buff_42 ;
        accum_43 <= tap_43 * buff_43 ;
        accum_44 <= tap_44 * buff_44 ;
        accum_45 <= tap_45 * buff_45 ;
        accum_46 <= tap_46 * buff_46 ;
        accum_47 <= tap_47 * buff_47 ;
        accum_48 <= tap_48 * buff_48 ;
        accum_49 <= tap_49 * buff_49 ;
        accum_50 <= tap_50 * buff_50 ;
        accum_51 <= tap_51 * buff_51 ;
        accum_52 <= tap_52 * buff_52 ;
        accum_53 <= tap_53 * buff_53 ;
        accum_54 <= tap_54 * buff_54 ;
        accum_55 <= tap_55 * buff_55 ;
        accum_56 <= tap_56 * buff_56 ;
        accum_57 <= tap_57 * buff_57 ;
        accum_58 <= tap_58 * buff_58 ;
        accum_59 <= tap_59 * buff_59 ;
        accum_60 <= tap_60 * buff_60 ;
        accum_61 <= tap_61 * buff_61 ;
        accum_62 <= tap_62 * buff_62 ;
        accum_63 <= tap_63 * buff_63 ;
        accum_64 <= tap_64 * buff_64 ;
        accum_65 <= tap_65 * buff_65 ;
        accum_66 <= tap_66 * buff_66 ;
      end 
  end 

//////////// Accumulation ////////////

always @(posedge i_clk) 
  begin: accumulation_block 
    if (i_en_fir) 
      begin
        o_fir_data <= accum_0  + accum_1  + accum_2  + accum_3  + 
                      accum_4  + accum_5  + accum_6  + accum_7  +
                      accum_8  + accum_9  + accum_10 + accum_11 +
                      accum_12 + accum_13 + accum_14 + accum_15 +
                      accum_16 + accum_17 + accum_18 + accum_19 +
                      accum_20 + accum_21 + accum_22 + accum_23 +
                      accum_24 + accum_25 + accum_26 + accum_27 +
                      accum_28 + accum_29 + accum_30 + accum_31 +
                      accum_32 + accum_33 + accum_34 + accum_35 +
                      accum_36 + accum_37 + accum_38 + accum_39 +
                      accum_40 + accum_41 + accum_42 + accum_43 +
                      accum_44 + accum_45 + accum_46 + accum_47 +
                      accum_48 + accum_49 + accum_50 + accum_51 +
                      accum_52 + accum_53 + accum_54 + accum_55 +
                      accum_56 + accum_57 + accum_58 + accum_59 +
                      accum_60 + accum_61 + accum_62 + accum_63 +
                      accum_64 + accum_65 + accum_66 ;
      end 
  end 

endmodule
