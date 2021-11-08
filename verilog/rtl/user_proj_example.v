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

    // wire [`numNeuronLayer2-1:0] o2_valid;
    // wire [`numNeuronLayer2*`dataWidth-1:0] x2_out;
    // reg [`numNeuronLayer2*`dataWidth-1:0] holdData_2;
    // reg [`dataWidth-1:0] out_data_2;
    // reg data_out_valid_2;

    // Layer_2 #(.NN(`numNeuronLayer2),.numWeight(`numWeightLayer2),.dataWidth(`dataWidth),.layerNum(2),.sigmoidSize(`sigmoidSize),.weightIntWidth(`weightIntWidth),.actType(`Layer2ActType)) l2(
    //     .clk(clk),
    //     .rst(reset),
    // //	.weightValid(weightValid),
    // //	.biasValid(biasValid),
    // //	.weightValue(weightValue),
    // //	.biasValue(biasValue),
    // //	.config_layer_num(config_layer_num),
    // //	.config_neuron_num(config_neuron_num),
    //     .x_valid(data_out_valid_1),
    //     .x_in(out_data_1),
    //     .o_valid(o2_valid),
    //     .x_out(x2_out)
    // );

    // //State machine for data pipelining

    // reg       state_2;
    // integer   count_2;
    // always @(posedge clk)
    // begin
    //     if(reset)
    //     begin
    //         state_2 <= IDLE;
    //         count_2 <= 0;
    //         data_out_valid_2 <=0;
    //     end
    //     else
    //     begin
    //         case(state_2)
    //             IDLE: begin
    //                 count_2 <= 0;
    //                 data_out_valid_2 <=0;
    //                 if (o2_valid[0] == 1'b1)
    //                 begin
    //                     holdData_2 <= x2_out;
    //                     state_2 <= SEND;
    //                 end
    //             end
    //             SEND: begin
    //                 out_data_2 <= holdData_2[`dataWidth-1:0];
    //                 holdData_2 <= holdData_2>>`dataWidth;
    //                 count_2 <= count_2 +1;
    //                 data_out_valid_2 <= 1;
    //                 if (count_2 == `numNeuronLayer2)
    //                 begin
    //                     state_2 <= IDLE;
    //                     data_out_valid_2 <= 0;
    //                 end
    //             end
    //         endcase
    //     end
    // end

    // wire [`numNeuronLayer3-1:0] o3_valid;
    // wire [`numNeuronLayer3*`dataWidth-1:0] x3_out;
    // reg [`numNeuronLayer3*`dataWidth-1:0] holdData_3;
    // reg [`dataWidth-1:0] out_data_3;
    // reg data_out_valid_3;

    // Layer_3 #(.NN(`numNeuronLayer3),.numWeight(`numWeightLayer3),.dataWidth(`dataWidth),.layerNum(3),.sigmoidSize(`sigmoidSize),.weightIntWidth(`weightIntWidth),.actType(`Layer3ActType)) l3(
    //     .clk(clk),
    //     .rst(reset),
    // //	.weightValid(weightValid),
    // //	.biasValid(biasValid),
    // //	.weightValue(weightValue),
    // //	.biasValue(biasValue),
    // //	.config_layer_num(config_layer_num),
    // //	.config_neuron_num(config_neuron_num),
    //     .x_valid(data_out_valid_2),
    //     .x_in(out_data_2),
    //     .o_valid(o3_valid),
    //     .x_out(x3_out)
    // );

    // //State machine for data pipelining

    // reg       state_3;
    // integer   count_3;
    // always @(posedge clk)
    // begin
    //     if(reset)
    //     begin
    //         state_3 <= IDLE;
    //         count_3 <= 0;
    //         data_out_valid_3 <=0;
    //     end
    //     else
    //     begin
    //         case(state_3)
    //             IDLE: begin
    //                 count_3 <= 0;
    //                 data_out_valid_3 <=0;
    //                 if (o3_valid[0] == 1'b1)
    //                 begin
    //                     holdData_3 <= x3_out;
    //                     state_3 <= SEND;
    //                 end
    //             end
    //             SEND: begin
    //                 out_data_3 <= holdData_3[`dataWidth-1:0];
    //                 holdData_3 <= holdData_3>>`dataWidth;
    //                 count_3 <= count_3 +1;
    //                 data_out_valid_3 <= 1;
    //                 if (count_3 == `numNeuronLayer3)
    //                 begin
    //                     state_3 <= IDLE;
    //                     data_out_valid_3 <= 0;
    //                 end
    //             end
    //         endcase
    //     end
    // end

    // wire [`numNeuronLayer4-1:0] o4_valid;
    // wire [`numNeuronLayer4*`dataWidth-1:0] x4_out;
    // reg [`numNeuronLayer4*`dataWidth-1:0] holdData_4;
    // reg [`dataWidth-1:0] out_data_4;
    // reg data_out_valid_4;

    // Layer_4 #(.NN(`numNeuronLayer4),.numWeight(`numWeightLayer4),.dataWidth(`dataWidth),.layerNum(4),.sigmoidSize(`sigmoidSize),.weightIntWidth(`weightIntWidth),.actType(`Layer4ActType)) l4(
    //     .clk(clk),
    //     .rst(reset),
    // //	.weightValid(weightValid),
    // //	.biasValid(biasValid),
    // //	.weightValue(weightValue),
    // //	.biasValue(biasValue),
    // //	.config_layer_num(config_layer_num),
    // //	.config_neuron_num(config_neuron_num),
    //     .x_valid(data_out_valid_1),
    //     .x_in(out_data_1),
    //     .o_valid(o4_valid),
    //     .x_out(x4_out)
    // );

    // //State machine for data pipelining

    // reg       state_4;
    // integer   count_4;
    // always @(posedge clk)
    // begin
    //     if(reset)
    //     begin
    //         state_4 <= IDLE;
    //         count_4 <= 0;
    //         data_out_valid_4 <=0;
    //     end
    //     else
    //     begin
    //         case(state_4)
    //             IDLE: begin
    //                 count_4 <= 0;
    //                 data_out_valid_4 <=0;
    //                 if (o4_valid[0] == 1'b1)
    //                 begin
    //                     holdData_4 <= x4_out;
    //                     state_4 <= SEND;
    //                 end
    //             end
    //             SEND: begin
    //                 out_data_4 <= holdData_4[`dataWidth-1:0];
    //                 holdData_4 <= holdData_4>>`dataWidth;
    //                 count_4 <= count_4 +1;
    //                 data_out_valid_4 <= 1;
    //                 if (count_4 == `numNeuronLayer4)
    //                 begin
    //                     state_4 <= IDLE;
    //                     data_out_valid_4 <= 0;
    //                 end
    //             end
    //         endcase
    //     end
    // end

    // reg [`numNeuronLayer4*`dataWidth-1:0] holdData_5;
    // assign rdata = holdData_5[`dataWidth-1:0];

    // always @(posedge clk)
    //     begin
    //         if (o4_valid[0] == 1'b1)
    //             holdData_5 <= x4_out;
    //         else if(!ready)
    //         begin
    //             holdData_5 <= holdData_5>>`dataWidth;
    //         end
    //     end


    maxFinder #(.numInput(`numNeuronLayer1),.inputWidth(`dataWidth))
        mFind(
            .i_clk(clk),
            .i_data(x1_out),
            .i_valid(o1_valid[0]),
            .o_data(out),
            .o_data_valid(out_valid)
        );

    // always @(posedge clk) begin
    //     if(rst)
    //         count <= 32'b0;
    //     else if(wbs_stb_i && wbs_cyc_i && wbs_we_i && wbs_adr_i == USER_ADDRESS) begin
    //         count <= wbs_dat_i;
    //     end
    // end

    // // acks
    // always @(posedge clk) begin
    //     if(rst)
    //         wack <= 0;
    //     else
    //         // return ack immediately
    //         wack <= (wbs_stb_i && wbs_adr_i == USER_ADDRESS);
    // end

endmodule


//     reg ready;
//     reg [BITS-1:0] count;
//     reg [BITS-1:0] rdata;

//     always @(posedge clk) begin
//         if (reset) begin
//             count <= 0;
//             ready <= 0;
//         end else begin
//             ready <= 1'b0;
//             count <= count + 1;

//             if (valid && !ready) begin
//                 ready <= 1'b1;
//                 rdata <= count;
//                 if (wstrb[0]) count[7:0]   <= wdata[7:0];
//                 if (wstrb[1]) count[15:8]  <= wdata[15:8];
//                 if (wstrb[2]) count[23:16] <= wdata[23:16];
//                 if (wstrb[3]) count[31:24] <= wdata[31:24];
//             end
//         end
//     end

// endmodule

// module counter #(
//     parameter BITS = 32
// )(
//     input clk,
//     input reset,
//     input valid,
//     input [3:0] wstrb,
//     input [BITS-1:0] wdata,
//     input [BITS-1:0] la_write,
//     input [BITS-1:0] la_input,
//     output ready,
//     output [BITS-1:0] rdata,
//     output [BITS-1:0] count
// );
//     reg ready;
//     reg [BITS-1:0] count;
//     reg [BITS-1:0] rdata;

//     always @(posedge clk) begin
//         if (reset) begin
//             count <= 0;
//             ready <= 0;
//         end else begin
//             ready <= 1'b0;
//             if (~|la_write) begin
//                 count <= count + 1;
//             end
//             if (valid && !ready) begin
//                 ready <= 1'b1;
//                 rdata <= count;
//                 if (wstrb[0]) count[7:0]   <= wdata[7:0];
//                 if (wstrb[1]) count[15:8]  <= wdata[15:8];
//                 if (wstrb[2]) count[23:16] <= wdata[23:16];
//                 if (wstrb[3]) count[31:24] <= wdata[31:24];
//             end else if (|la_write) begin
//                 count <= la_write & la_input;
//             end
//         end
//     end

// endmodule

`default_nettype wire
