//-------------------------------------------------------------------
//-- main_tb.v
//-- Testbench for FPGA Computer Implementation
//-------------------------------------------------------------------
`default_nettype none
`timescale 100 ns / 10 ns
`define SIMULATION
`define DEBOUNCE_LIMIT 5

module main_tb();

  //-- Simulation time: 100us (100 * 100ns)
  parameter DURATION = 100;
  
  //-- Clock signal
  reg clk = 0;
  always #0.5 clk = ~clk;
  
  //-- Input buttons
  reg sw1 = 0;  // Reset button (active HIGH)
  reg sw2 = 1;  // Run/Step button (active LOW)
  reg sw3 = 1;  // Display mode button (active LOW)
  reg sw4 = 1;  // Instruction select button (active LOW)
  
  //-- Output LEDs
  wire led1, led2, led3, led4;
  
  //-- 7-segment display outputs
  // Upper digit (tens)
  wire seg1_a, seg1_b, seg1_c, seg1_d, seg1_e, seg1_f, seg1_g;
  // Lower digit (ones)
  wire seg2_a, seg2_b, seg2_c, seg2_d, seg2_e, seg2_f, seg2_g;
  
  //-- Instantiate the Unit Under Test (UUT)
  main UUT (
    .CLK(clk),
    .SW1(sw1),
    .SW2(sw2),
    .SW3(sw3),
    .SW4(sw4),
    .LED1(led1),
    .LED2(led2),
    .LED3(led3),
    .LED4(led4),
    .o_Segment1_A(seg1_a),
    .o_Segment1_B(seg1_b),
    .o_Segment1_C(seg1_c),
    .o_Segment1_D(seg1_d),
    .o_Segment1_E(seg1_e),
    .o_Segment1_F(seg1_f),
    .o_Segment1_G(seg1_g),
    .o_Segment2_A(seg2_a),
    .o_Segment2_B(seg2_b),
    .o_Segment2_C(seg2_c),
    .o_Segment2_D(seg2_d),
    .o_Segment2_E(seg2_e),
    .o_Segment2_F(seg2_f),
    .o_Segment2_G(seg2_g)
  );
  
  //-- Test sequence
  initial begin
    //-- Dump variables to the VCD file
    $dumpvars(0, main_tb);
    
    //-- Initial state - wait for stabilization
    #20;
    $display("\n===== Starting simulation =====");
    $display("Initial state: PC=%h, RegA=%h, RegB=%h", 
             UUT.pc_out_bus, UUT.regA_out_bus, UUT.regB_out_bus);
    
    //-- Test 1: Reset
    $display("\n===== Test 1: Reset =====");
    sw1 = 1;  // Assert reset
    #20;      // Hold reset longer
    sw1 = 0;  // Release reset
    #20;      // Wait longer after reset
    $display("After reset: PC=%h, RegA=%h, RegB=%h", 
             UUT.pc_out_bus, UUT.regA_out_bus, UUT.regB_out_bus);
    
    //-- Test 2: Single Step
    $display("\n===== Test 2: Single Step =====");
    sw2 = 0;  // Press step button
    #20;      // Hold button longer
    sw2 = 1;  // Release step button
    #20;      // Wait longer for execution
    $display("After step 1: PC=%h, RegA=%h, RegB=%h", 
             UUT.pc_out_bus, UUT.regA_out_bus, UUT.regB_out_bus);
    
    //-- Test 3: Multiple Steps
    $display("\n===== Test 3: Multiple Steps =====");
    // Step 2
    sw2 = 0;  // Press step button
    #20;      // Hold button longer
    sw2 = 1;  // Release step button
    #20;      // Wait longer for execution
    $display("After step 2: PC=%h, RegA=%h, RegB=%h", 
             UUT.pc_out_bus, UUT.regA_out_bus, UUT.regB_out_bus);
    
    // Step 3
    sw2 = 0;  // Press step button
    #20;      // Hold button longer
    sw2 = 1;  // Release step button
    #20;      // Wait longer for execution
    $display("After step 3: PC=%h, RegA=%h, RegB=%h", 
             UUT.pc_out_bus, UUT.regA_out_bus, UUT.regB_out_bus);
    
    //-- Test 4: SW3 Display Mode Testing
    $display("\n===== Test 4: SW3 Display Mode Testing =====");
    
    // Test all 4 display modes
    $display("Mode 0 (PC lower 4 bits): mode=%d, PC=%h, LEDs=%b%b%b%b (expected PC[3:0]=%b)", 
             UUT.display_mode, UUT.pc_out_bus, led1, led2, led3, led4, UUT.pc_out_bus[3:0]);
    $display("  7-Segment Display: %d%d (decimal value=%d)", 
             UUT.tens_digit, UUT.ones_digit, UUT.pc_out_bus[3:0]);
    $display("  Tens=%b%b%b%b%b%b%b Ones=%b%b%b%b%b%b%b", 
             seg1_a, seg1_b, seg1_c, seg1_d, seg1_e, seg1_f, seg1_g,
             seg2_a, seg2_b, seg2_c, seg2_d, seg2_e, seg2_f, seg2_g);
    
    // Change to mode 1 (RegA)
    sw3 = 0; #20; sw3 = 1; #20;
    $display("Mode 1 (RegA lower 4 bits): mode=%d, RegA=%h, LEDs=%b%b%b%b (expected RegA[3:0]=%b)", 
             UUT.display_mode, UUT.regA_out_bus, led1, led2, led3, led4, UUT.regA_out_bus[3:0]);
    $display("  7-Segment Display: %d%d (decimal value=%d)", 
             UUT.tens_digit, UUT.ones_digit, UUT.regA_out_bus[3:0]);
    
    // Change to mode 2 (RegB)
    sw3 = 0; #20; sw3 = 1; #20;
    $display("Mode 2 (RegB lower 4 bits): mode=%d, RegB=%h, LEDs=%b%b%b%b (expected RegB[3:0]=%b)", 
             UUT.display_mode, UUT.regB_out_bus, led1, led2, led3, led4, UUT.regB_out_bus[3:0]);
    $display("  7-Segment Display: %d%d (decimal value=%d)", 
             UUT.tens_digit, UUT.ones_digit, UUT.regB_out_bus[3:0]);
    
    // Change to mode 3 (ALU)
    sw3 = 0; #20; sw3 = 1; #20;
    $display("Mode 3 (ALU lower 4 bits): mode=%d, ALU=%h, LEDs=%b%b%b%b (expected ALU[3:0]=%b)", 
             UUT.display_mode, UUT.alu_out_bus, led1, led2, led3, led4, UUT.alu_out_bus[3:0]);
    $display("  7-Segment Display: %d%d (decimal value=%d)", 
             UUT.tens_digit, UUT.ones_digit, UUT.alu_out_bus[3:0]);
    
    // Change back to mode 0 (wraps around)
    sw3 = 0; #20; sw3 = 1; #20;
    $display("Back to Mode 0: mode=%d, PC=%h, LEDs=%b%b%b%b (expected PC[3:0]=%b)", 
             UUT.display_mode, UUT.pc_out_bus, led1, led2, led3, led4, UUT.pc_out_bus[3:0]);
    $display("  7-Segment Display: %d%d (decimal value=%d)", 
             UUT.tens_digit, UUT.ones_digit, UUT.pc_out_bus[3:0]);
    
    //-- Test 5: Validate Display Mode with CPU Execution
    $display("\n===== Test 5: Display Mode During CPU Execution =====");
    
    // Execute a few more steps to change register values
    sw2 = 0; #20; sw2 = 1; #20;  // Step 4
    $display("After step 4: PC=%h, RegA=%h, RegB=%h, ALU=%h", 
             UUT.pc_out_bus, UUT.regA_out_bus, UUT.regB_out_bus, UUT.alu_out_bus);
    
    // Test each display mode shows updated values
    // Mode 0: PC
    sw3 = 0; #20; sw3 = 1; #20; // Should be mode 1 now
    sw3 = 0; #20; sw3 = 1; #20; // Should be mode 2 now
    sw3 = 0; #20; sw3 = 1; #20; // Should be mode 3 now
    sw3 = 0; #20; sw3 = 1; #20; // Should be mode 0 now
    $display("Current mode 0 - PC display: LEDs=%b%b%b%b, PC[3:0]=%b", 
             led1, led2, led3, led4, UUT.pc_out_bus[3:0]);
    
    //-- End simulation
    #(DURATION) $display("\n===== End of simulation =====");
    $finish;
  end

endmodule