/******************************************************************************
��飺һ��SPIЭ��ͨ����4�ߵ������ߣ�SDI(��������)��SDO(�������)��SCLK(ʱ��)��CS(Ƭѡ)��
        (1)SDO/MOSI - ���豸������������豸��������;
        (2)SDI/MISO - ���豸�������룬���豸�������;
        (3)SCLK - ʱ���źţ������豸����;
        (4)CS/SS - ���豸ʹ���źţ������豸���ơ�
      ����SPI�����ִ���ģʽ����CPOL��ʱ�Ӽ��ԣ���CPHA��ʱ����λ������
        �����ء��½��ء�ǰ�ء����ش�������ȻҲ��MSB��LSB���䷽ʽ
        CPOL=0������ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ��
        CPOL=1������ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ��
        CPHA=0����ʾ���ݲ������ڵ�1�����أ����ݷ����ڵ�2������
        CPHA=1����ʾ���ݲ������ڵ�2�����أ����ݷ����ڵ�1������
        TODO��SPI��ģ�����֮ͨ�ŵ����豸ʱ����λ�ͼ���Ӧ��һ��
      ������û�����ݽ�����ʱ�����ǵ�ʱ����Ҫô�Ǳ��ָߵ�ƽҪô�Ǳ��ֵ͵�ƽ
�ο����ӣ� https://github.com/simoore/FMC151-driver      
******************************************************************************/
module io_spi #(
    parameter WIDTH = 32,           //����λ��
    parameter FLIP = 0,             //��/С��ģʽ
    parameter SCLK_TIME = 4         //SPI ʱ�ӷ�Ƶ
    )(
    input wire rst,                 //��λ
    input wire clk,                 //����ʱ��
    input wire start_tx,            //��ʼ����
    output reg done_tx,             //�������
    
    output reg spi_cs,              //SPI ���豸ʹ���ź�
    output reg spi_clk,             //SPI ���豸ʱ����
    output wire spi_mosi,           //SPI ���豸�����
    input wire spi_miso,            //SPI ���豸������
    
    input wire [WIDTH-1:0] tx_data, //SPI Ԥ��������
    output reg [WIDTH-1:0] rx_data  //SPI Ԥ��ȡ����
    );
    
    localparam IDLE     = 6'b000001;    //����
    localparam QUIT     = 6'b000010;    //������������豸ʹ����Ч
    localparam HIGH     = 6'b000100;    //
    localparam READ     = 6'b001000;    //��ȡ
    localparam LOW      = 6'b010000;    //
    localparam BUFFER   = 6'b100000;    //ͨ�Ž�������ȡ����
    
    reg rst_lcl;
    wire last_read, at_zero, shift, quit, low, buffer, idle;
    reg [5:0] state,next_state;
    reg [7:0] shift_counter,state_counter;
    reg [WIDTH-1:0] shift_reg;
    
    //------  �����ź�   -------//
    always@(posedge clk)
        rst_lcl <= rst;
             
    //------  Ƭѡ�ź�    -------//
    always@(posedge clk)
        if(!quit)   spi_cs <= 0;        
        else        spi_cs <= 1;    
        
    //------  SPIʱ���ź�  -----//
    //-��clk�źŰ��ն�ȡ���ٶȣ���low�ź�ָʾ�������-//  
    always@(posedge clk)
        if(low)     spi_clk <= 0;
        else        spi_clk <= 1;
        
    //----- ��λ�Ĵ�������   ------//
    assign last_read = (shift_counter == 0);                        //�������㣬�����������
    assign spi_mosi = FLIP ? shift_reg[0] : shift_reg[WIDTH - 1];   //spi ���Ϊ��λ�Ĵ������׻�β�ź�
    
    always@(posedge clk)    begin
        if(start_tx)    shift_counter <= WIDTH - 1;                //��ʼ���ͣ��������� 
        else if(shift)  shift_counter <= shift_counter - 1;        //���ڷ��ͣ�������1 
        
        if(start_tx)    shift_reg <= tx_data;                      //��ʼ���ͣ�Ԥ�����ź�ת�浽��λ�Ĵ�����
        else if(shift)  
            if(FLIP)    shift_reg <= {spi_miso, shift_reg[WIDTH-1:1]};  //���ڷ��ͣ��Ĵ������ֽ�Ϊ����bitλ�����Ĵ�������1λ
            else        shift_reg <= {shift_reg[WIDTH-2:0], spi_miso};  //ͬ�ϣ��෴
    end
    
    //--------  �������  ---------//
    always@(posedge clk)    begin
        done_tx <= buffer;                      //�������             
        if(buffer) rx_data <= shift_reg;        //����λ�Ĵ�����ȡ��������
    end
    
    //----- ״̬��   --------------//
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
