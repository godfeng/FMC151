/******************************************************************************
简介：一、SPI协议通常有4线单向串行线，SDI(数据输入)，SDO(数据输出)，SCLK(时钟)，CS(片选)。
        (1)SDO/MOSI - 主设备数据输出，从设备数据输入;
        (2)SDI/MISO - 主设备数据输入，从设备数据输出;
        (3)SCLK - 时钟信号，由主设备产生;
        (4)CS/SS - 从设备使能信号，由主设备控制。
      二、SPI有四种传输模式，由CPOL（时钟极性）和CPHA（时钟相位）控制
        上升沿、下降沿、前沿、后沿触发。当然也有MSB和LSB传输方式
        CPOL=0，串行同步时钟的空闲状态为低电平；
        CPOL=1，串行同步时钟的空闲状态为高电平；
        CPHA=0，表示数据采样是在第1个边沿，数据发送在第2个边沿
        CPHA=1，表示数据采样是在第2个边沿，数据发送在第1个边沿
        TODO：SPI主模块和与之通信的外设备时钟相位和极性应该一致
      三、当没有数据交流的时候我们的时钟线要么是保持高电平要么是保持低电平
参考链接： https://github.com/simoore/FMC151-driver      
******************************************************************************/
module io_spi #(
    parameter WIDTH = 32,           //数据位宽
    parameter FLIP = 0,             //大/小端模式
    parameter SCLK_TIME = 4         //SPI 时钟分频
    )(
    input wire rst,                 //复位
    input wire clk,                 //输入时钟
    input wire start_tx,            //开始传输
    output reg done_tx,             //传输完成
    
    output reg spi_cs,              //SPI 从设备使能信号
    output reg spi_clk,             //SPI 主设备时钟线
    output wire spi_mosi,           //SPI 主设备输出线
    input wire spi_miso,            //SPI 主设备输入线
    
    input wire [WIDTH-1:0] tx_data, //SPI 预发送数据
    output reg [WIDTH-1:0] rx_data  //SPI 预读取数据
    );
    
    localparam IDLE     = 6'b000001;    //空闲
    localparam QUIT     = 6'b000010;    //传输结束，从设备使能无效
    localparam HIGH     = 6'b000100;    //
    localparam READ     = 6'b001000;    //读取
    localparam LOW      = 6'b010000;    //
    localparam BUFFER   = 6'b100000;    //通信结束，读取缓存
    
    reg rst_lcl;
    wire last_read, at_zero, shift, quit, low, buffer, idle;
    reg [5:0] state,next_state;
    reg [7:0] shift_counter,state_counter;
    reg [WIDTH-1:0] shift_reg;
    
    //------  重置信号   -------//
    always@(posedge clk)
        rst_lcl <= rst;
             
    //------  片选信号    -------//
    always@(posedge clk)
        if(!quit)   spi_cs <= 0;        
        else        spi_cs <= 1;    
        
    //------  SPI时钟信号  -----//
    //-将clk信号按照读取的速度，由low信号指示进行输出-//  
    always@(posedge clk)
        if(low)     spi_clk <= 0;
        else        spi_clk <= 1;
        
    //----- 移位寄存器输入   ------//
    assign last_read = (shift_counter == 0);                        //计数归零，传输数据完毕
    assign spi_mosi = FLIP ? shift_reg[0] : shift_reg[WIDTH - 1];   //spi 输出为移位寄存器的首或尾信号
    
    always@(posedge clk)    begin
        if(start_tx)    shift_counter <= WIDTH - 1;                //开始发送，计数重置 
        else if(shift)  shift_counter <= shift_counter - 1;        //正在发送，计数减1 
        
        if(start_tx)    shift_reg <= tx_data;                      //开始发送，预发送信号转存到移位寄存器中
        else if(shift)  
            if(FLIP)    shift_reg <= {spi_miso, shift_reg[WIDTH-1:1]};  //正在发送，寄存器首字节为输入bit位，并寄存器后移1位
            else        shift_reg <= {shift_reg[WIDTH-2:0], spi_miso};  //同上，相反
    end
    
    //--------  缓存输出  ---------//
    always@(posedge clk)    begin
        done_tx <= buffer;                      //传输完成             
        if(buffer) rx_data <= shift_reg;        //从移位寄存器提取接收数据
    end
    
    //----- 状态机   --------------//
    assign at_zero = (state_counter == 0);      //
    always@(posedge clk)
        if(at_zero | idle)  state_counter <= SCLK_TIME;
        else                state_counter <= state_counter - 1;
    
    always@(posedge clk)
        if(rst_lcl) state <= IDLE;
        else    state <= next_state;
            
    always @(*) begin
        next_state = state;
        case(state)
            IDLE:   if(start_tx)            next_state = QUIT;
            QUIT:   if(at_zero)             next_state = LOW;
            LOW:    if(at_zero)             next_state = HIGH;
            HIGH:   if(at_zero & last_read) next_state = BUFFER;
                    else if(at_zero)        next_state = READ;
            READ:                           next_state = LOW;
            BUFFER:                         next_state = IDLE;
        endcase
    end
    
    assign shift = (state == READ);
    assign quit  = (state == QUIT) || (state == IDLE) || (state == BUFFER);
    assign low = (state != HIGH);
    assign buffer = (state == BUFFER);
    assign idle = (state == IDLE);
endmodule
