// 4-Bit Early Computer for TinyTapeout
// A simple processor with ALU, two registers, and control logic

module tt_um_william_carter (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Internal signals
    wire [3:0] alu_a, alu_b;
    wire [3:0] alu_out;
    wire [2:0] alu_op;
    wire [3:0] reg_a_data, reg_b_data;
    wire reg_a_en, reg_b_en;
    wire reg_a_sel, reg_b_sel; // 0 = register value, 1 = zero
    wire [3:0] mux_a_out, mux_b_out;
    
    // Input assignments from ui_in
    assign alu_op = ui_in[2:0];      // ALU operation select
    assign reg_a_en = ui_in[3];      // Register A write enable
    assign reg_b_en = ui_in[4];      // Register B write enable  
    assign reg_a_sel = ui_in[5];     // Register A mux select (0=reg, 1=zero)
    assign reg_b_sel = ui_in[6];     // Register B mux select (0=reg, 1=zero)
    // ui_in[7] reserved for future use
    
    // Use bidirectional pins for data input (4 bits)
    assign uio_oe = 8'b11110000;     // Lower 4 bits as inputs, upper 4 as outputs
    wire [3:0] data_in = uio_in[3:0]; // Input data for registers
    
    // Output assignments
    assign uo_out[3:0] = alu_out;    // ALU result
    assign uo_out[7:4] = reg_a_data; // Register A contents for monitoring
    assign uio_out[7:4] = reg_b_data; // Register B contents for monitoring
    assign uio_out[3:0] = 4'b0;      // Input pins, driven to 0
    
    // ALU Implementation
    alu_4bit alu_inst (
        .a(alu_a),
        .b(alu_b),
        .op(alu_op),
        .out(alu_out)
    );
    
    // Register A
    register_4bit reg_a_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena & reg_a_en),
        .data_in(alu_out),  // Register stores ALU result
        .data_out(reg_a_data)
    );
    
    // Register B  
    register_4bit reg_b_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena & reg_b_en),
        .data_in(data_in),  // Register B can be loaded from external data
        .data_out(reg_b_data)
    );
    
    // Multiplexers for ALU inputs
    // A input: can be Register A value or zero
    assign mux_a_out = reg_a_sel ? 4'b0000 : reg_a_data;
    
    // B input: can be Register B value or zero  
    assign mux_b_out = reg_b_sel ? 4'b0000 : reg_b_data;
    
    // ALU inputs
    assign alu_a = mux_a_out;
    assign alu_b = mux_b_out;

endmodule

// 4-bit ALU Module
module alu_4bit (
    input  wire [3:0] a,
    input  wire [3:0] b,
    input  wire [2:0] op,
    output reg  [3:0] out
);

    always @(*) begin
        case (op)
            3'b000: out = a + b;      // Addition
            3'b001: out = a - b;      // Subtraction  
            3'b010: out = a & b;      // Bitwise AND
            3'b011: out = a | b;      // Bitwise OR
            3'b100: out = ~a;         // Bitwise NOT
            3'b101: out = a ^ b;      // Bitwise XOR
            3'b110: out = a << 1;     // Left shift
            3'b111: out = a >> 1;     // Right shift
            default: out = 4'b0000;   // Default case
        endcase
    end

endmodule

// 4-bit Register Module
module register_4bit (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ena,
    input  wire [3:0] data_in,
    output reg  [3:0] data_out
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= 4'b0000;
        end else if (ena) begin
            data_out <= data_in;
        end
        // If ena is low, register holds its current value
    end

endmodule