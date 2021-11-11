// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 32,
    parameter USER_ADDRESS = 32'h30000000
)(
`ifdef USE_POWER_PINS
    // inout vdda1,	// User area 1 3.3V supply
    // inout vdda2,	// User area 2 3.3V supply
    // inout vssa1,	// User area 1 analog ground
    // inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    // inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    // inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata; 
    reg [31:0] wdata;
    reg [BITS-1:0] count;
    reg wack;
    reg sig;
    wire intr;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = (wbs_cyc_i && wbs_stb_i && (wbs_adr_i == USER_ADDRESS)) && ~sig; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    // assign wdata = wbs_dat_i;
    assign wbs_ack_o = wack;

    // IO
    assign io_out = count;
    assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    assign irq = intr;

    // LA
    assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset
    assign clk = wb_clk_i;
    assign rst = wb_rst_i;  
    // assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    // assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    always @(posedge clk) begin
        if (rst)
            sig <= 0;
        else
            sig <= wbs_cyc_i && wbs_stb_i && (wbs_adr_i == USER_ADDRESS);
    end

    always @(posedge clk) begin
        if(rst)
            wdata <= 32'b0;
        else if(wbs_stb_i && wbs_cyc_i && wbs_we_i && wbs_adr_i == USER_ADDRESS) begin
            wdata <= wbs_dat_i;
        end
    end

    // acks
    always @(posedge clk) begin
        if(rst)
            wack <= 0;
        else
            // return ack immediately
            wack <= (wbs_stb_i && wbs_adr_i == USER_ADDRESS);
    end

    FCNet #(
        .BITS(BITS),
        .USER_ADDRESS(USER_ADDRESS)
    ) FCNet(
        .clk(clk),
        .reset(rst),
        .ready(wbs_ack_o),
        .valid(valid),
        .rdata(rdata),
        .wdata(wdata),
        .wstrb(wstrb),
        .la_write(la_write),
        .la_input(la_data_in[63:32]),
        .intr(intr)
    );

endmodule

module FCNet #(
    parameter BITS = 32,
    parameter USER_ADDRESS = 32'h03000000
)(
    input clk,
    input reset,
    input valid,
    input [3:0] wstrb,
    input [BITS-1:0] wdata,
    input [BITS-1:0] la_write,
    input [BITS-1:0] la_input,
    output ready,
    output wack,
    output [BITS-1:0] rdata,
    output intr
);

    wire [31:0] out;
    wire out_valid;

    assign intr = out_valid;
    assign rdata = out;

    localparam  IDLE = 'd0,
                SEND = 'd1;
    wire    [`numNeuronLayer1-1:0]              o1_valid;
    wire    [`numNeuronLayer1*`dataWidth-1:0]   x1_out;
    reg     [`numNeuronLayer1*`dataWidth-1:0]   holdData_1;
    reg     [`dataWidth-1:0] out_data_1;
    reg     data_out_valid_1;

    Layer_1 #(.NN(`numNeuronLayer1),
            .numWeight(`numWeightLayer1),
            .dataWidth(`dataWidth),
            .layerNum(1),
            .sigmoidSize(`sigmoidSize),
            .weightIntWidth(`weightIntWidth),
            .actType(`Layer1ActType)) l1(
        .clk(clk),
        .rst(reset),
        .x_valid(valid),
        .x_in(wdata[15:0]),
        .o_valid(o1_valid),
        .x_out(x1_out)
    );

    //State machine for data pipelining

    reg       state_1;
    integer   count_1;
    always @(posedge clk)
    begin
        if(reset)
        begin
            state_1 <= IDLE;
            count_1 <= 0;
            data_out_valid_1 <=0;
        end
        else
        begin
            case(state_1)
                IDLE: begin
                    count_1 <= 0;
                    data_out_valid_1 <=0;
                    if (o1_valid[0] == 1'b1)
                    begin
                        holdData_1 <= x1_out;
                        state_1 <= SEND;
                    end
                end
                SEND: begin
                    out_data_1 <= holdData_1[`dataWidth-1:0];
                    holdData_1 <= holdData_1>>`dataWidth;
                    count_1 <= count_1 +1;
                    data_out_valid_1 <= 1;
                    if (count_1 == `numNeuronLayer1)
                    begin
                        state_1 <= IDLE;
                        data_out_valid_1 <= 0;
                    end
                end
            endcase
        end
    end

    maxFinder #(.numInput(`numNeuronLayer1),.inputWidth(`dataWidth))
        mFind(
            .i_clk(clk),
            .i_data(x1_out),
            .i_valid(o1_valid[0]),
            .o_data(out),
            .o_data_valid(out_valid)
        );

endmodule

`default_nettype wire
