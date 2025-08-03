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
    output LED4,   // LED output
    // 7-segment display outputs (active LOW)
    // Upper digit (tens)
    output o_Segment1_A,
    output o_Segment1_B,
    output o_Segment1_C,
    output o_Segment1_D,
    output o_Segment1_E,
    output o_Segment1_F,
    output o_Segment1_G,
    // Lower digit (ones)
    output o_Segment2_A,
    output o_Segment2_B,
    output o_Segment2_C,
    output o_Segment2_D,
    output o_Segment2_E,
    output o_Segment2_F,
    output o_Segment2_G
);

    // Debounce limit parameter
`ifdef SIMULATION
    parameter DEBOUNCE_LIMIT = `DEBOUNCE_LIMIT;
`else
    parameter DEBOUNCE_LIMIT = 10000; // Adjust as needed for hardware
`endif

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
    
    // ========== Button Debouncing ==========
    // Debounce registers for each button
    reg [15:0] debounce_counter_sw1 = 0;
    reg [15:0] debounce_counter_sw2 = 0;
    reg [15:0] debounce_counter_sw3 = 0;
    reg [15:0] debounce_counter_sw4 = 0;
    
    reg sw1_state = 0;
    reg sw2_state = 0;
    reg sw3_state = 0;
    reg sw4_state = 0;
    
    reg sw1_debounced = 0;
    reg sw2_debounced = 0;
    reg sw3_debounced = 0;
    reg sw4_debounced = 0;
    
    // Debounce logic for SW1 (Reset)
    always @(posedge CLK) begin
        if (SW1 != sw1_state) begin
            debounce_counter_sw1 <= 0;
            sw1_state <= SW1;
        end else if (debounce_counter_sw1 < DEBOUNCE_LIMIT) begin
            debounce_counter_sw1 <= debounce_counter_sw1 + 1;
            if (debounce_counter_sw1 == DEBOUNCE_LIMIT - 1) begin
                sw1_debounced <= sw1_state;
            end
        end
    end
    
    // Debounce logic for SW2 (Run/Step)
    always @(posedge CLK) begin
        if (SW2 != sw2_state) begin
            debounce_counter_sw2 <= 0;
            sw2_state <= SW2;
        end else if (debounce_counter_sw2 < DEBOUNCE_LIMIT) begin
            debounce_counter_sw2 <= debounce_counter_sw2 + 1;
            if (debounce_counter_sw2 == DEBOUNCE_LIMIT - 1) begin
                sw2_debounced <= sw2_state;
            end
        end
    end
    
    // Debounce logic for SW3 (Display mode)
    always @(posedge CLK) begin
        if (SW3 != sw3_state) begin
            debounce_counter_sw3 <= 0;
            sw3_state <= SW3;
        end else if (debounce_counter_sw3 < DEBOUNCE_LIMIT) begin
            debounce_counter_sw3 <= debounce_counter_sw3 + 1;
            if (debounce_counter_sw3 == DEBOUNCE_LIMIT - 1) begin
                sw3_debounced <= sw3_state;
            end
        end
    end
    
    // Debounce logic for SW4 (Instruction select)
    always @(posedge CLK) begin
        if (SW4 != sw4_state) begin
            debounce_counter_sw4 <= 0;
            sw4_state <= SW4;
        end else if (debounce_counter_sw4 < DEBOUNCE_LIMIT) begin
            debounce_counter_sw4 <= debounce_counter_sw4 + 1;
            if (debounce_counter_sw4 == DEBOUNCE_LIMIT - 1) begin
                sw4_debounced <= sw4_state;
            end
        end
    end
    
    // Manual clock from step button - now using debounced signal
    reg prev_step = 0;
    reg step_pulse = 0;
    
    always @(posedge CLK) begin
        prev_step <= sw2_debounced;
        if (!sw2_debounced && prev_step) begin
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
    
    // Change display mode when SW3 is pressed - now using debounced signal
    reg prev_mode = 0;
    always @(posedge CLK) begin
        prev_mode <= sw3_debounced;
        if (!sw3_debounced && prev_mode) begin
            display_mode <= display_mode + 1;
            if (display_mode == 3) display_mode <= 0;
        end
    end
    
    // ========== LED Output Control ==========
    // LEDs always show Program Counter (PC) for easy debugging
    // LED mapping is RIGHT TO LEFT: LED4 is rightmost (LSB), LED1 is leftmost (MSB)
    // So binary 0001 (decimal 1) would light up just LED4 (rightmost)
    // Binary 1000 (decimal 8) would light up just LED1 (leftmost)
    assign LED1 = pc_out_bus[3]; // Leftmost LED (MSB of PC)
    assign LED2 = pc_out_bus[2];
    assign LED3 = pc_out_bus[1];
    assign LED4 = pc_out_bus[0]; // Rightmost LED (LSB of PC)
    
    // ========== 7-Segment Display Value Selection ==========
    // 7-segment display shows value based on SW3 display mode
    reg [3:0] display_value;
    
    always @(*) begin
        case (display_mode)
            0: begin  // Show PC lower bits
                display_value = pc_out_bus[3:0];
            end
            1: begin  // Show Register A lower bits
                display_value = regA_out_bus[3:0];
            end
            2: begin  // Show Register B lower bits
                display_value = regB_out_bus[3:0];
            end
            3: begin  // Show ALU result lower bits
                display_value = alu_out_bus[3:0];
            end
            default: begin
                display_value = pc_out_bus[3:0];
            end
        endcase
    end
    
    // ========== 7-Segment Display Control ==========
    // Convert 4-bit value (0-15) to two-digit decimal display (00-15)
    // Segments are active LOW (0 = ON, 1 = OFF)
    
    // Binary to decimal conversion
    reg [3:0] tens_digit;
    reg [3:0] ones_digit;
    
    always @(*) begin
        if (display_value >= 10) begin
            tens_digit = 1;  // Show "1" in tens place for 10-15
            ones_digit = display_value - 10;  // Show remainder in ones place
        end else begin
            tens_digit = 0;  // Show "0" in tens place for 0-9
            ones_digit = display_value;  // Show value in ones place
        end
    end
    
    // 7-segment decoder for decimal digits (0-9 only)
    reg [6:0] tens_pattern;
    reg [6:0] ones_pattern;
    
    // Decoder for tens digit (0 or 1)
    always @(*) begin
        case (tens_digit)
            4'd0: tens_pattern = 7'b0000001; // 0: segments A,B,C,D,E,F on
            4'd1: tens_pattern = 7'b1001111; // 1: segments B,C on
            default: tens_pattern = 7'b1111111; // Blank
        endcase
    end
    
    // Decoder for ones digit (0-9)
    always @(*) begin
        case (ones_digit)
            4'd0: ones_pattern = 7'b0000001; // 0: segments A,B,C,D,E,F on
            4'd1: ones_pattern = 7'b1001111; // 1: segments B,C on
            4'd2: ones_pattern = 7'b0010010; // 2: segments A,B,G,E,D on
            4'd3: ones_pattern = 7'b0000110; // 3: segments A,B,G,C,D on
            4'd4: ones_pattern = 7'b1001100; // 4: segments F,G,B,C on
            4'd5: ones_pattern = 7'b0100100; // 5: segments A,F,G,C,D on
            4'd6: ones_pattern = 7'b0100000; // 6: segments A,F,G,E,D,C on
            4'd7: ones_pattern = 7'b0001111; // 7: segments A,B,C on
            4'd8: ones_pattern = 7'b0000000; // 8: all segments on
            4'd9: ones_pattern = 7'b0000100; // 9: segments A,B,C,D,F,G on
            default: ones_pattern = 7'b1111111; // Blank
        endcase
    end
    
    // Connect to 7-segment display outputs
    // Upper digit (tens)
    assign o_Segment1_A = tens_pattern[6];
    assign o_Segment1_B = tens_pattern[5];
    assign o_Segment1_C = tens_pattern[4];
    assign o_Segment1_D = tens_pattern[3];
    assign o_Segment1_E = tens_pattern[2];
    assign o_Segment1_F = tens_pattern[1];
    assign o_Segment1_G = tens_pattern[0];
    
    // Lower digit (ones)
    assign o_Segment2_A = ones_pattern[6];
    assign o_Segment2_B = ones_pattern[5];
    assign o_Segment2_C = ones_pattern[4];
    assign o_Segment2_D = ones_pattern[3];
    assign o_Segment2_E = ones_pattern[2];
    assign o_Segment2_F = ones_pattern[1];
    assign o_Segment2_G = ones_pattern[0];

    // ========== Computer Components ==========
    // Program Counter
    pc PC(
        .clk(cpu_clock),
        .rst(sw1_debounced),  // Now using debounced reset
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
        .rst(sw1_debounced),  // Now using debounced reset
        .data(alu_out_bus),  // RegA gets data from ALU output
        .load(cu_out_bus[8]),  // RegA load from control unit bit [8] - matches original design
        .out(regA_out_bus)
    );
    
    // Register B
    register regB(
        .clk(cpu_clock),
        .rst(sw1_debounced),  // Now using debounced reset
        .data(alu_out_bus),  // RegB gets data from ALU output - matches original design
        .load(cu_out_bus[7]),  // RegB load from control unit bit [7] - matches original design
        .out(regB_out_bus)
    );
    
    // MUX A
    muxA muxA(
        .e0(regA_out_bus),
        .e1(8'b00000001),
        .e2(8'b00000000),
        .e3(regB_out_bus),
        .c(cu_out_bus[6:5]),  // MUX A control from bits [6:5] - matches original design
        .out(muxA_out_bus)
    );
    
    // MUX B
    muxB muxB(
        .e0(regB_out_bus), 
        .e1(dm_out_bus),
        .e2(im_out_bus[7:0]),
        .e3(8'b00000000),
        .c(cu_out_bus[4:3]),  // MUX B control from bits [4:3] - matches original design
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
        .rst(sw1_debounced),  // Now using debounced reset
        .data_in(alu_out_bus), 
        .address(muxD_out_bus), 
        .w(cu_out_bus[10]),
        .data_out(dm_out_bus)
    );
    
    // Status Register
    status Status(
        .clk(cpu_clock),
        .rst(sw1_debounced),  // Now using debounced reset
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
            // Simple Loop Program: Countdown from 15 to 1 using JMP
            // Using CORRECT opcodes according to CPU specification
            // PC | Instruction                | RegA after | RegB after | ALU Result | Notes
            // ---+---------------------------+------------+------------+------------+------------------------
            8'd0: out = 15'b000000000000000;  // 00000000   | 00000000   | 00000000   | NOP - No operation
            8'd1: out = 15'b000001100001111;  // 00000000   | 00001111   | 00000000   | MOV B,Lit (Load 15 into RegB)
            8'd2: out = 15'b000000000000000;  // 00001111   | 00001111   | 00001111   | MOV A,B (Move RegB to RegA)
            8'd3: out = 15'b000001100000001;  // 00001111   | 00000001   | 00001111   | MOV B,Lit (Load 1 into RegB)
            // LOOP: Subtract 1 from RegA repeatedly
            8'd4: out = 15'b000100000000000;  // 00001110   | 00000001   | 00001110   | SUB A,B (RegA=RegA-1)
            8'd5: out = 15'b101001100000100;  // Jump back to address 4 (continue loop)
            default: out = 15'b000000000000000; // NOP for undefined addresses
        endcase
    end
endmodule

// Control Unit module - decodes instructions and generates control signals
module control_unit(opcode, zncv, out);
  input [6:0] opcode;
  input [3:0] zncv;
  output [11:0] out;
  
  wire [6:0] opcode;
  wire [3:0] zncv;
  reg [11:0] out;
  
  always @(opcode) begin
    case(opcode)
      'b0000000: out = 12'b000101000000;
      'b0000001: out = 12'b000010011000;
      'b0000010: out = 12'b000101010000;
      'b0000011: out = 12'b000011010000;
      'b0000100: out = 12'b000100000000;
      'b0000101: out = 12'b000010000000;
      'b0000110: out = 12'b000100010000;
      'b0000111: out = 12'b000011110000;
      'b0001000: out = 12'b000100000001;
      'b0001001: out = 12'b000010000001;
      'b0001010: out = 12'b000100010001;
      'b0001011: out = 12'b000011110001;
      'b0001100: out = 12'b000100000010;
      'b0001101: out = 12'b000010000010;
      'b0001110: out = 12'b000100010010;
      'b0001111: out = 12'b000011110010;
      'b0010000: out = 12'b000100000011;
      'b0010001: out = 12'b000010000011;
      'b0010010: out = 12'b000100010011;
      'b0010011: out = 12'b000011110011;
      'b0010100: out = 12'b000100000100;
      'b0010101: out = 12'b000101100100;
      'b0010110: out = 12'b000010000100;
      'b0010111: out = 12'b000011100100;
      'b0011000: out = 12'b000100000101;
      'b0011001: out = 12'b000010000101;
      'b0011010: out = 12'b000100010101;
      'b0011011: out = 12'b000011110101;
      'b0011100: out = 12'b000100000110;
      'b0011101: out = 12'b000101100110;
      'b0011110: out = 12'b000010000110;
      'b0011111: out = 12'b000011100110;
      'b0100000: out = 12'b000100000111;
      'b0100001: out = 12'b000101100111;
      'b0100010: out = 12'b000010000111;
      'b0100011: out = 12'b000011100111;
      'b0100100: out = 12'b000010100000;
      'b0100101: out = 12'b000101001000;
      'b0100110: out = 12'b000011001000;
      'b0100111: out = 12'b010000011000;
      'b0101000: out = 12'b010001000000;
      'b0101001: out = 12'b001101001000;
      'b0101010: out = 12'b001011001000;
      'b0101011: out = 12'b011000011000;
      'b0101100: out = 12'b000100001000;
      'b0101101: out = 12'b000011101000;
      'b0101110: out = 12'b001100001000;
      'b0101111: out = 12'b010000000000;
      'b0110000: out = 12'b000100001001;
      'b0110001: out = 12'b000011101001;
      'b0110010: out = 12'b001100001001;
      'b0110011: out = 12'b010000000001;
      'b0110100: out = 12'b000100001010;
      'b0110101: out = 12'b000011101010;
      'b0110110: out = 12'b001100001010;
      'b0110111: out = 12'b010000000010;
      'b0111000: out = 12'b000100001011;
      'b0111001: out = 12'b000011101011;
      'b0111010: out = 12'b001100001011;
      'b0111011: out = 12'b010000000011;
      'b0111100: out = 12'b010000000100;
      'b0111101: out = 12'b010001100100;
      'b0111110: out = 12'b011000000100;
      'b0111111: out = 12'b000100001101;
      'b1000000: out = 12'b000011101101;
      'b1000001: out = 12'b001100001101;
      'b1000010: out = 12'b010000000101;
      'b1000011: out = 12'b010000000110;
      'b1000100: out = 12'b010001100110;
      'b1000101: out = 12'b011000000110;
      'b1000110: out = 12'b010000000111;
      'b1000111: out = 12'b010001100111;
      'b1001000: out = 12'b011000000111;
      'b1001001: out = 12'b010000101000;
      'b1001010: out = 12'b011000101000;
      'b1001011: out = 12'b010001011000;
      'b1001100: out = 12'b011001011000;
      'b1001101: out = 12'b000000000001;
      'b1001110: out = 12'b000000010001;
      'b1001111: out = 12'b000001110001;
      'b1010000: out = 12'b000000001001;
      'b1010001: out = 12'b000001101001;
      'b1010010: out = 12'b001000001001;
      'b1010011: out = 12'b100000000000;
      
      'b1010100: begin
        if (zncv[3] == 1) begin //z=1
          out = 12'b100000000000;
        end
      end
      'b1010101: begin
        if (zncv[3] == 0) begin //z=0
          out = 12'b100000000000;
        end
      end
      'b1010110: begin
        if (zncv[3] == 0 && zncv[2] == 0) begin //z=0 y n=0
          out = 12'b100000000000;
      	end
      end
      'b1010111: begin
        if (zncv[2] == 1) begin //n=1
          out = 12'b100000000000;
        end
      end
      'b1011000: begin
        if (zncv[2] == 0) begin //n=0
          out = 12'b100000000000;
        end
      end
      'b1011001: begin
        if (zncv[3] == 1 || zncv[2] == 1) begin //z=1 o n=1 
          out = 12'b100000000000;
        end
      end
      'b1011010: begin
        if (zncv[1] == 1) begin //c=1
          out = 12'b100000000000;
        end
      end
      'b1011011: begin 
        if (zncv[0] == 1) begin //v=1
          out = 12'b100000000000;
        end
      end
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
