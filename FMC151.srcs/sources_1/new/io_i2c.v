/******************************************************************************
��飺I2CЭ����2��˫�����ߣ�������SDA��ʱ����SCL��
      SDA����Ϊ���ģʽ��ÿ�δ���8bit ��������ÿ���豸����һ����ַ����7bit���㲥��ַΪ0.
      ���ݴ��䣺SCLΪ�ߵ�ƽʱ��SDA���������ȶ�����ôSDA�����ڴ�������bit��
                ��SDA�������䣬��������ʾһ���Ự�Ŀ�ʼ�����
      ���ݸı䣺SCLΪ�͵�ƽʱ��SDA�߲��ܸı䴫���bit
      ��ʼ�źţ�SCLΪ�ߵ�ƽʱ��SDA�ɸߵ�ƽ��͵�ƽ���䣬��ʼ�������ݡ�
      �����źţ�SCLΪ�ߵ�ƽʱ��SDA�ɵ͵�ƽ��ߵ�ƽ���䣬�����������ݡ�
      д�Ĵ����ı�׼����Ϊ��
          1.    Master����START
          2.    Master����I2C addr��7bit����w����0��1bit�����ȴ�ACK
          3.    Slave����ACK
          4.    Master����reg addr��8bit�����ȴ�ACK
          5.    Slave����ACK
          6.    Master����data��8bit������Ҫд��Ĵ����е����ݣ��ȴ�ACK
          7.    Slave����ACK
          8.    ��6���͵�7�������ظ���Σ���˳��д����Ĵ���
          9.    Master����STOP
      ���Ĵ����ı�׼����Ϊ��
          1.    Master����I2C addr��7bit����w����1��1bit�����ȴ�ACK
          2.    Slave����ACK
          3.    Master����reg addr��8bit�����ȴ�ACK
          4.    Slave����ACK
          5.    Master����START
          6.    Master����I2C addr��7bit����r����1��1bit�����ȴ�ACK
          7.    Slave����ACK
          8.    Slave����data��8bit�������Ĵ������ֵ
          9.    Master����ACK
          10.    ��8���͵�9�������ظ���Σ���˳�������Ĵ���
�ο����ӣ�  http://blog.csdn.net/subkiller/article/details/6854910
            https://github.com/simoore/FMC151-driver
******************************************************************************/
module i2c_master #(
    parameter INPUT_CLK = 200_000_000,      //����� ϵͳʱ��
    parameter BUS_CLK = 400_000             //�趨�� I2C����ʱ��
    )(
    input wire clk,                         //ʱ��
    input wire rst,                         //����
    input wire ena,                         //I2Cģ��ʹ��
    input wire [6:0] addr,                  //���豸��ַ
    input wire rw,                          //
    input wire [7:0] data_wr,
    output wire busy,
    output wire load,
    output reg [7:0] data_rd,
    inout sda,                  //I2C ������
    inout scl                   //I2C ʱ����
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
