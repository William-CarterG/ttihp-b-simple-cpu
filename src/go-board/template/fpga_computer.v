// FPGA Computer based on design-3.sv
// Modified for Go-board FPGA with LED outputs
// Simplified to fit within FPGA resources

module main(
    input  CLK,    // Clock input
    input  SW1,    // Reset button (active HIGH)
    input  SW2,    // Run/Step button (active LOW)
    input  SW3,    // Display mode button (active LOW)
    input  SW4,    // Instruction select button (active LOW)
    output LED1,   // LED output
    output LED2,   // LED output
    output LED3,   // LED output
    output LED4    // LED output
);

    // ========== Clock Management ==========
    // Slow clock for visible operation
    reg [24:0] slow_counter = 0;
    reg slow_clock = 0;
    
    // Generate slow clock for visible operation
    always @(posedge CLK) begin
        slow_counter <= slow_counter + 1;
        if (slow_counter == 0) begin
            slow_clock <= ~slow_clock;
        end
    end
    
    // Manual clock from step button
    reg prev_step = 0;
    reg step_pulse = 0;
    
    always @(posedge CLK) begin
        prev_step <= SW2;
        if (!SW2 && prev_step) begin
            step_pulse <= 1;
        end else begin
            step_pulse <= 0;
        end
    end
    
    // CPU clock selection - either slow clock or step pulse
    wire cpu_clock = slow_clock | step_pulse;

    // ========== Computer Components ==========
    // Buses
    wire [7:0] pc_out_bus;
    wire [14:0] im_out_bus;
    wire [7:0] regA_out_bus;
    wire [7:0] regB_out_bus;
    wire [7:0] muxA_out_bus;
    wire [7:0] muxB_out_bus;
    wire [7:0] muxD_out_bus;
    wire [7:0] alu_out_bus;
    wire [3:0] alu_zncv_out_bus;
    wire [11:0] cu_out_bus;
    wire [3:0] status_out_bus;
    wire [7:0] dm_out_bus;

    // ========== Display Mode Control ==========
    reg [1:0] display_mode = 0;  // Different display modes
    
    // Change display mode when SW3 is pressed
    reg prev_mode = 0;
    always @(posedge CLK) begin
        prev_mode <= SW3;
        if (!SW3 && prev_mode) begin
            display_mode <= display_mode + 1;
            if (display_mode == 3) display_mode <= 0;
        end
    end
    
    // ========== LED Output Control ==========
    // LED output based on display mode
    reg [3:0] led_output;
    
    always @(*) begin
        case (display_mode)
            0: begin  // Show PC lower bits
                led_output = pc_out_bus[3:0];
            end
            1: begin  // Show Register A lower bits
                led_output = regA_out_bus[3:0];
            end
            2: begin  // Show Register B lower bits
                led_output = regB_out_bus[3:0];
            end
            3: begin  // Show ALU result lower bits
                led_output = alu_out_bus[3:0];
            end
            default: begin
                led_output = pc_out_bus[3:0];
            end
        endcase
    end
    
    // Connect to physical LEDs
    // LED mapping is RIGHT TO LEFT: LED4 is rightmost (LSB), LED1 is leftmost (MSB)
    // So binary 0001 (decimal 1) would light up just LED4 (rightmost)
    // Binary 1000 (decimal 8) would light up just LED1 (leftmost)
    assign LED1 = led_output[3]; // Leftmost LED (MSB)
    assign LED2 = led_output[2];
    assign LED3 = led_output[1];
    assign LED4 = led_output[0]; // Rightmost LED (LSB)

    // ========== Computer Components ==========
    // Program Counter
    pc PC(
        .clk(cpu_clock),
        .rst(SW1),
        .l(cu_out_bus[11]),
        .im(im_out_bus[7:0]),
        .pc(pc_out_bus)
    );
    
    // Instruction Memory
    instruction_memory IM(
        .address(pc_out_bus),
        .out(im_out_bus)
    );
    
    // Control Unit
    control_unit CU(
        .opcode(im_out_bus[14:8]),
        .zncv(status_out_bus),
        .out(cu_out_bus)
    );
    
    // Register A
    register regA(
        .clk(cpu_clock),
        .rst(SW1),
        .data(alu_out_bus),
        .load(cu_out_bus[8]),
        .out(regA_out_bus)
    );
    
    // Register B
    register regB(
        .clk(cpu_clock),
        .rst(SW1),
        .data(alu_out_bus),
        .load(cu_out_bus[7]),
        .out(regB_out_bus)
    );
    
    // MUX A
    muxA muxA(
        .e0(regA_out_bus),
        .e1(8'b00000001),
        .e2(8'b00000000),
        .e3(regB_out_bus),
        .c(cu_out_bus[6:5]),
        .out(muxA_out_bus)
    );
    
    // MUX B
    muxB muxB(
        .e0(regB_out_bus), 
        .e1(dm_out_bus),
        .e2(im_out_bus[7:0]),
        .e3(8'b00000000),
        .c(cu_out_bus[4:3]),
        .out(muxB_out_bus)
    );
    
    // MUX D
    muxD muxD(
        .e0(im_out_bus[7:0]), 
        .e1(regB_out_bus),
        .c(cu_out_bus[9]),
        .out(muxD_out_bus)
    );
    
    // ALU
    alu ALU(
        .a(muxA_out_bus),
        .b(muxB_out_bus),
        .s(cu_out_bus[2:0]),
        .out(alu_out_bus),
        .zncv(alu_zncv_out_bus)
    );
    
    // Data Memory
    data_memory DM(
        .clk(cpu_clock), 
        .rst(SW1),
        .data_in(alu_out_bus), 
        .address(muxD_out_bus), 
        .w(cu_out_bus[10]),
        .data_out(dm_out_bus)
    );
    
    // Status Register
    status Status(
        .clk(cpu_clock),
        .rst(SW1),
        .zncv(alu_zncv_out_bus),
        .out(status_out_bus)
    );

endmodule

// Program Counter module with reset
module pc(clk, rst, pc, im, l);
    input clk, rst, l;
    input [7:0] im;
    output [7:0] pc;
    
    wire clk, rst, l;
    wire [7:0] im;
    reg [7:0] pc;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 8'b00000000;
        end else if (l) begin
            pc <= im;
        end else begin
            pc <= pc + 1;
        end
    end
endmodule

// Instruction Memory module with hardcoded program
module instruction_memory(address, out);
    input [7:0] address;
    output [14:0] out;

    wire [7:0] address;
    reg [14:0] out;

    // Hardcoded instruction memory
    always @(address) begin
        case(address)
            // Example program: Simple counter and arithmetic
            // PC | Instruction                | RegA after | RegB after | Notes
            // ---+---------------------------+------------+------------+------------------------
            8'd0: out = 15'b000000000000000;  // 00000000   | 00000000   | NOP - No operation
            8'd1: out = 15'b000010100000001;  // 00000000   | 00000001   | Load 1 into RegB
            8'd2: out = 15'b000010000000000;  // 00000001   | 00000001   | Move RegB to RegA
            8'd3: out = 15'b000000000000001;  // 00000010   | 00000001   | Add RegA + RegB -> RegA (1+1=2)
            8'd4: out = 15'b000010100000010;  // 00000010   | 00000010   | Load 2 into RegB
            8'd5: out = 15'b000000000000001;  // 00000100   | 00000010   | Add RegA + RegB -> RegA (2+2=4)
            8'd6: out = 15'b000110000000000;  // 00000010   | 00000010   | Subtract RegA - RegB -> RegA (4-2=2)
            8'd7: out = 15'b000010100000011;  // 00000010   | 00000011   | Load 3 into RegB
            8'd8: out = 15'b001000000000000;  // 00000010   | 00000011   | AND RegA & RegB -> RegA (2&3=2)
            8'd9: out = 15'b001010000000000;  // 00000011   | 00000011   | OR RegA | RegB -> RegA (2|3=3)
            8'd10: out = 15'b001100000000000; // 11111100   | 00000011   | NOT RegA -> RegA (~3=252)
            8'd11: out = 15'b001110000000000; // 11111111   | 00000011   | XOR RegA ^ RegB -> RegA (252^3=255)
            8'd12: out = 15'b010000000000000; // 11111110   | 00000011   | Shift left RegA (255<<1=254)
            8'd13: out = 15'b010010000000000; // 01111111   | 00000011   | Shift right RegA (254>>1=127)
            8'd14: out = 15'b100000000000000; // 01111111   | 00000011   | Jump to address 0 (loop)
            default: out = 15'b000000000000000; // NOP for undefined addresses
        endcase
    end
endmodule

// ALU module
module alu(a, b, s, out, zncv);
    input [7:0] a, b;
    input [2:0] s;
    output [7:0] out;
    output [3:0] zncv;

    wire [7:0] a, b;
    wire [2:0] s;
    reg [7:0] out;
    reg [3:0] zncv;
    
    reg [8:0] temp;

    always @(a, b, s) begin
        case(s)
            3'b000: out = a + b;         // Addition
            3'b001: out = a - b;         // Subtraction
            3'b010: out = a & b;         // Bitwise AND
            3'b011: out = a | b;         // Bitwise OR
            3'b100: out = ~a;            // Bitwise NOT
            3'b101: out = a ^ b;         // Bitwise XOR
            3'b110: out = a << 1;        // Left shift
            3'b111: out = a >> 1;        // Right shift
            default: out = 8'b00000000;  // Default case
        endcase
        
        // Calculate flags
        temp = (s == 3'b001) ? {1'b0, a} - {1'b0, b} : {1'b0, a} + {1'b0, b};
        zncv[3] = (out == 8'b00000000);                  // Zero flag
        zncv[2] = out[7];                                // Negative flag
        zncv[1] = temp[8];                               // Carry flag
        zncv[0] = (a[7] == b[7]) && (out[7] != a[7]);    // Overflow flag
    end
endmodule

// Register module with reset
module register(clk, rst, data, load, out);
    input clk, rst, load;
    input [7:0] data;
    output [7:0] out;

    wire clk, rst, load;
    wire [7:0] data;
    reg [7:0] out;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            out <= 8'b00000000;
        end else if (load) begin
            out <= data;
        end
    end
endmodule

// MUX A module
module muxA(e0, e1, e2, e3, c, out); 
    input [7:0] e0, e1, e2, e3; // e0: RegA; e1 = 1, e2=0; e3:RegB
    input [1:0] c;
    output [7:0] out;
    
    wire [7:0] e0, e1, e2, e3;
    wire [1:0] c;
    reg [7:0] out;
    
    always @(*) begin
        case(c)
            2'b00: out = e0;
            2'b01: out = e1;
            2'b10: out = e2;
            2'b11: out = e3;
        endcase
    end
endmodule

// MUX B module
module muxB(e0, e1, e2, e3, c, out);
    input [7:0] e0, e1, e2, e3; //e0: RegB, e1: DataMem, e2: IM, e3 = 0
    input [1:0] c;
    output [7:0] out;
    
    wire [7:0] e0, e1, e2, e3;
    wire [1:0] c;
    reg [7:0] out;

    always @(*) begin
        case(c)
            2'b00: out = e0;
            2'b01: out = e1;
            2'b10: out = e2;
            2'b11: out = e3;
        endcase
    end
endmodule

// MUX D module
module muxD(e0, e1, c, out);
    input [7:0] e0, e1; //e0: im, e1: regB
    input c;
    output [7:0] out;
    
    wire [7:0] e0, e1;
    wire c;
    reg [7:0] out;
    
    always @(*) begin
        case(c)
            1'b0: out = e0;
            1'b1: out = e1;
        endcase
    end
endmodule

// Control Unit module
module control_unit(opcode, zncv, out);
    input [6:0] opcode;
    input [3:0] zncv;
    output [11:0] out;
    
    wire [6:0] opcode;
    wire [3:0] zncv;
    reg [11:0] out;
    
    always @(*) begin
        // Default: No operation
        out = 12'b000000000000;
        
        case(opcode)
            // Basic operations
            7'b0000000: out = 12'b000000000000; // NOP
            7'b0000001: out = 12'b000100000000; // Load RegA from ALU
            7'b0000010: out = 12'b000010000000; // Load RegB from ALU
            7'b0000011: out = 12'b000110000000; // Load both RegA and RegB from ALU
            
            // ALU operations with RegA as destination
            7'b0000100: out = 12'b000100000000; // ADD: RegA = RegA + RegB
            7'b0000110: out = 12'b000100000001; // SUB: RegA = RegA - RegB
            7'b0001000: out = 12'b000100000010; // AND: RegA = RegA & RegB
            7'b0001010: out = 12'b000100000011; // OR: RegA = RegA | RegB
            7'b0001100: out = 12'b000100000100; // NOT: RegA = ~RegA
            7'b0001110: out = 12'b000100000101; // XOR: RegA = RegA ^ RegB
            7'b0010000: out = 12'b000100000110; // SHL: RegA = RegA << 1
            7'b0010010: out = 12'b000100000111; // SHR: RegA = RegA >> 1
            
            // Memory operations
            7'b0010100: out = 12'b001000000000; // Store RegA to memory at address from RegB
            7'b0010110: out = 12'b000100010000; // Load RegA from memory at address from RegB
            
            // Jump operations
            7'b1000000: out = 12'b100000000000; // Unconditional jump
            7'b1000001: begin // Jump if zero
                if (zncv[3] == 1) out = 12'b100000000000;
            end
            7'b1000010: begin // Jump if not zero
                if (zncv[3] == 0) out = 12'b100000000000;
            end
            7'b1000011: begin // Jump if negative
                if (zncv[2] == 1) out = 12'b100000000000;
            end
            7'b1000100: begin // Jump if carry
                if (zncv[1] == 1) out = 12'b100000000000;
            end
            
            // Default case
            default: out = 12'b000000000000;
        endcase
    end
endmodule

// Data Memory module with reset
module data_memory(clk, rst, data_in, address, w, data_out);
    input clk, rst, w;
    input [7:0] data_in, address;
    output [7:0] data_out;
    
    wire clk, rst, w;
    wire [7:0] data_in, address;
    reg [7:0] data_out;
    
    reg [7:0] mem [0:31]; // Reduced size for FPGA
    
    integer i;
    
    // Write operation
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Initialize memory to zeros
            for (i = 0; i < 32; i = i + 1) begin
                mem[i] <= 8'b00000000;
            end
        end else if (w) begin
            mem[address[4:0]] <= data_in; // Use only lower 5 bits of address
        end
    end
    
    // Read operation - separate always block
    always @(*) begin
        data_out = mem[address[4:0]];
    end
endmodule

// Status Register module with reset
module status(clk, rst, zncv, out);
    input clk, rst;
    input [3:0] zncv;
    output [3:0] out;
    
    wire clk, rst;
    wire [3:0] zncv;
    reg [3:0] out;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            out <= 4'b0000;
        end else begin
            out <= zncv;
        end
    end
endmodule
