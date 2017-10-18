/******************************************************************************
简介：I2C协议有2线双向串行线，数据线SDA，时钟线SCL，
      SDA传输为大端模式，每次传输8bit ；总线上每个设备都有一个地址，共7bit，广播地址为0.
      数据传输：SCL为高电平时，SDA线若保持稳定，那么SDA上是在传输数据bit；
                若SDA发生跳变，则用来表示一个会话的开始或结束
      数据改变：SCL为低电平时，SDA线才能改变传输的bit
      开始信号：SCL为高电平时，SDA由高电平向低电平跳变，开始传送数据。
      结束信号：SCL为高电平时，SDA由低电平向高电平跳变，结束传送数据。
      写寄存器的标准流程为：
          1.    Master发起START
          2.    Master发送I2C addr（7bit）和w操作0（1bit），等待ACK
          3.    Slave发送ACK
          4.    Master发送reg addr（8bit），等待ACK
          5.    Slave发送ACK
          6.    Master发送data（8bit），即要写入寄存器中的数据，等待ACK
          7.    Slave发送ACK
          8.    第6步和第7步可以重复多次，即顺序写多个寄存器
          9.    Master发起STOP
      读寄存器的标准流程为：
          1.    Master发送I2C addr（7bit）和w操作1（1bit），等待ACK
          2.    Slave发送ACK
          3.    Master发送reg addr（8bit），等待ACK
          4.    Slave发送ACK
          5.    Master发起START
          6.    Master发送I2C addr（7bit）和r操作1（1bit），等待ACK
          7.    Slave发送ACK
          8.    Slave发送data（8bit），即寄存器里的值
          9.    Master发送ACK
          10.    第8步和第9步可以重复多次，即顺序读多个寄存器
参考链接：  http://blog.csdn.net/subkiller/article/details/6854910
            https://github.com/simoore/FMC151-driver
******************************************************************************/
module i2c_master #(
    parameter INPUT_CLK = 200_000_000,      //输入的 系统时钟
    parameter BUS_CLK = 400_000             //设定的 I2C总线时钟
    )(
    input wire clk,                         //时钟
    input wire rst,                         //重置
    input wire ena,                         //I2C模块使能
    input wire [6:0] addr,                  //从设备地址
    input wire rw,                          //
    input wire [7:0] data_wr,
    output wire busy,
    output wire load,
    output reg [7:0] data_rd,
    inout sda,                  //I2C 数据线
    inout scl                   //I2C 时钟线
    );
    
    localparam MAX_COUNT_CLK = INPUT_CLK / BUS_CLK / 4;
    localparam READY        = 9'b0_0000_0001;
    localparam START        = 9'b0_0000_0010;
    localparam COMMD        = 9'b0_0000_0100;
    localparam SLV_ACK1     = 9'b0_0000_1000;
    localparam WRITE        = 9'b0_0001_0000;
    localparam READ         = 9'b0_0010_0000;
    localparam SLV_ACK2     = 9'b0_0100_0000;
    localparam MSTR_ACK     = 9'b0_1000_0000;
    localparam STOP         = 9'b1_0000_0000;
    
    reg rst_lcl;
    reg [8:0] state;
    reg [8:0] next_state;
    wire same_cmd;
    
    reg [7:0] clk_cnt;
    reg [1:0] quad_cnt;
    reg data_ce;
    wire stretching;
    
    reg scl_ce;
    reg scl_ena;
    wire sda_int;
    wire sda_ena_n;
    
    reg [7:0] addr_rw;
    reg [7:0] data_tx;
    reg [7:0] data_rx;
    reg [7:0] bit_count;
    reg [7:0] data_rd_buff;
    
    //==============================================================//
    //-- 
    //==============================================================//
    always@(posedge clk)
        rst_lcl <= rst;
        
    //==============================================================//
    //-- 
    //==============================================================//    
    always@(posedge clk)
        if(rst_lcl) begin
            clk_cnt <= 0;
            quad_cnt <= 0;
        end 
        else if(clk_cnt == MAX_COUNT_CLK) begin
            clk_cnt <= 0;
            quad_cnt = quad_cnt + 1;
        end
        else if(stretching == 0)
            clk_cnt <= clk_cnt + 1;
     always@(posedge clk)   begin
        if(clk_cnt == MAX_COUNT_CLK && quad_cnt == 0) 
            data_ce <= 1;
        else
            data_ce <= 0;
     end    
       
     assign stretching = (quad_cnt == 2'b10) && (scl == 0);
    
     //==============================================================//
     //-- 
     //==============================================================//   
     assign sda = (sda_ena_n == 0) ? 0 : 1'bZ;   
     assign sda_ena_n = (state == START || state == STOP) ? 0 : sda_int;
     assign sda_int = (state == COMMD) ? addr_rw[bit_count]:
                      (state == WRITE) ? data_tx[bit_count]:
                      (state == MSTR_ACK && same_cmd) ? 0 : 1;
     //==============================================================//
     //-- 
     //==============================================================//   
     always@(posedge clk)   begin
        if(clk_cnt == MAX_COUNT_CLK && quad_cnt == 1) 
            scl_ce <= 1;
        else
            scl_ce <= 0;
     end    
     
     always@(posedge clk)
        if(rst_lcl) scl_ena <= 0;
        else if(scl_ce) scl_ena <= (state & (READY | STOP)) == 0;
     assign scl = (scl_ena == 1 && quad_cnt[1] == 0)?0:1'bZ;
     
      //==============================================================//
      //-- 
      //==============================================================//    
     always@(*)begin
        next_state = state;
        case(state)
            READY:      if(ena == 1)           next_state = START;
            START:                             next_state = COMMD;
            COMMD:      if(bit_count == 0)     next_state = SLV_ACK1;
            SLV_ACK1:   if(addr_rw[0] == 0)    next_state = WRITE;
                        else                   next_state = READ;
            WRITE:      if(bit_count == 0)     next_state = SLV_ACK2;
            READ:       if(bit_count == 0)     next_state = MSTR_ACK;
            SLV_ACK2:   if(ena == 1)
                            if(same_cmd == 1)  next_state = WRITE;
                            else               next_state = START;
                        else                   next_state = STOP;
            MSTR_ACK:   if(ena == 1)
                            if(same_cmd == 1)  next_state = WRITE;
                            else               next_state = START;
                        else                   next_state = STOP;
            STOP:                              next_state = READY;
        endcase
     end
     
     assign busy        = (state != READY);
     assign load        = (state == SLV_ACK2);
     assign same_cmd    = (addr_rw == {addr,rw});
     //==============================================================//
     //-- 
     //==============================================================// 
     always@(posedge clk)
        if(data_ce)
            if(state == COMMD || state == READ || state == WRITE)
                bit_count <= bit_count - 1;
            else    
                bit_count <= 7;
       
     //==============================================================//
     //-- 
     //==============================================================// 
     always@(posedge clk)
        if(data_ce) begin
            if(state == START)      addr_rw <= {addr,rw};
            if(state == SLV_ACK1 || state == SLV_ACK2) data_tx <= data_wr;
        end
      //==============================================================//
      //-- 
      //==============================================================//    
      always@(posedge clk)
        if(rst_lcl)         data_rd <= 0;
        else if(data_ce) begin
            if(state == MSTR_ACK)   data_rd <= data_rx;
            if(state == READ)       data_rx[bit_count] <= sda;
        end
        
endmodule
