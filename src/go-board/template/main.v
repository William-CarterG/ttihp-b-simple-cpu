// 4-Bit Simple CPU for Go-board FPGA
// Based on the original tt_um_william_carter.v design
// Modified to include hardcoded instruction memory and LED output

module main(
    input CLK,             // Clock input
    input SW1,             // Reset button (active low)
    input SW2,             // Start button
    input SW3,             // User button 3
    input SW4,             // User button 4
    output LED1,           // LED output
    output LED2,           // LED output
    output LED3,           // LED output
    output LED4            // LED output
);

    // Internal signals
    wire [3:0] alu_a, alu_b;
    wire [3:0] alu_out;
    wire [2:0] alu_op;
    wire [3:0] reg_a_data, reg_b_data;
    wire reg_a_en, reg_b_en;
    wire reg_a_sel, reg_b_sel; // 0 = register value, 1 = zero
    wire [3:0] mux_a_out, mux_b_out;
    reg [3:0] data_in;        // Immediate value for Register B
    wire halt;                // Halt signal
    
    // Program counter and instruction memory
    reg [3:0] pc;             // 4-bit program counter (16 instructions max)
    reg [7:0] instruction;    // Current instruction
    reg [3:0] led_output;     // LED output register
    reg running;              // CPU running state
    
    // Blink counter for SW3 test
    reg [24:0] blink_counter = 0;
    reg blink_state = 0;
    
    // Instruction format:
    // [7:5] ALU_OP     - Operation for ALU
    // [4]   REG_A_EN   - Register A write enable
    // [3]   REG_B_EN   - Register B write enable
    // [2]   REG_A_SEL  - Register A mux select (0=reg, 1=zero)
    // [1]   REG_B_SEL  - Register B mux select (0=reg, 1=zero)
    // [0]   HALT       - Halt execution when 1
    
    // Instruction memory - 16 instructions max
    always @(*) begin
        case(pc)
            // Example program: Calculate (3 + 5) - 2 and display result
            4'd0: begin
                instruction = 8'b00001000; // Load immediate 3 into RegB (REG_B_EN=1)
                data_in = 4'd3;           // Immediate value 3
            end
            4'd1: begin
                instruction = 8'b00010000; // Store RegB in RegA (REG_A_EN=1)
                data_in = 4'd0;           // Immediate value (not used)
            end
            4'd2: begin
                instruction = 8'b00001000; // Load immediate 5 into RegB
                data_in = 4'd5;           // Immediate value 5
            end
            4'd3: begin
                instruction = 8'b00010000; // Add RegA + RegB, store in RegA (REG_A_EN=1)
                data_in = 4'd0;           // Immediate value (not used)
            end
            4'd4: begin
                instruction = 8'b00001000; // Load immediate 2 into RegB
                data_in = 4'd2;           // Immediate value 2
            end
            4'd5: begin
                instruction = 8'b00110000; // Subtract RegA - RegB, store in RegA (ALU_OP=001, REG_A_EN=1)
                data_in = 4'd0;           // Immediate value (not used)
            end
            4'd6: begin
                instruction = 8'b00000001; // Halt and display result
                data_in = 4'd0;           // Immediate value (not used)
            end
            default: begin
                instruction = 8'b00000001; // Halt by default
                data_in = 4'd0;           // Immediate value (not used)
            end
        endcase
    end
    
    // Instruction decoding
    assign alu_op = instruction[7:5];     // ALU operation select
    assign reg_a_en = instruction[4];     // Register A write enable
    assign reg_b_en = instruction[3];     // Register B write enable
    assign reg_a_sel = instruction[2];    // Register A mux select (0=reg, 1=zero)
    assign reg_b_sel = instruction[1];    // Register B mux select (0=reg, 1=zero)
    assign halt = instruction[0];         // Halt signal
    
    // Program counter control with improved logic
    always @(posedge CLK or negedge SW1) begin
        if (!SW1) begin
            // Reset - clear everything
            pc <= 4'd0;
            running <= 1'b0;
            led_output <= 4'd0;
        end else begin
            // Normal operation
            if (SW2) begin
                // Start or restart execution
                pc <= 4'd0;
                running <= 1'b1;
            end else if (running && !halt) begin
                // Increment PC if not halted
                pc <= pc + 1'b1;
            end else if (running && halt) begin
                // When halted, capture the result to LED output
                led_output <= reg_a_data;
                running <= 1'b0;
            end
        end
    end
    
    // Blink counter logic
    always @(posedge CLK) begin
        if (SW3) begin
            blink_counter <= blink_counter + 1;
            if (blink_counter == 0) begin
                blink_state <= ~blink_state;
            end
        end else begin
            blink_counter <= 0;
            blink_state <= 0;
        end
    end
    
    // Simplified LED display logic for easier debugging
    // SW3: Blink test
    // SW4: Show program counter value
    // Otherwise: Show register A value or led_output based on running state
    assign LED1 = SW3 ? blink_state : (SW4 ? pc[0] : (running ? reg_a_data[0] : led_output[0]));
    assign LED2 = SW3 ? blink_state : (SW4 ? pc[1] : (running ? reg_a_data[1] : led_output[1]));
    assign LED3 = SW3 ? blink_state : (SW4 ? pc[2] : (running ? reg_a_data[2] : led_output[2]));
    assign LED4 = SW3 ? blink_state : (SW4 ? pc[3] : (running ? reg_a_data[3] : led_output[3]));
    
    // ALU Implementation
    alu_4bit alu_inst (
        .a(alu_a),
        .b(alu_b),
        .op(alu_op),
        .out(alu_out)
    );
    
    // Register A
    register_4bit reg_a_inst (
        .clk(CLK),
        .rst_n(SW1),
        .ena(running & reg_a_en),
        .data_in(alu_out),  // Register stores ALU result
        .data_out(reg_a_data)
    );
    
    // Register B  
    register_4bit reg_b_inst (
        .clk(CLK),
        .rst_n(SW1),
        .ena(running & reg_b_en),
        .data_in(data_in),  // Register B can be loaded from immediate data
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