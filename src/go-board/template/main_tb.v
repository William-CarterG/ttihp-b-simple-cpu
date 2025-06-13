// Testbench for the 4-Bit Simple CPU FPGA Implementation
`timescale 1ns / 1ps

module main_tb();

    // Testbench signals
    reg clk = 0;
    reg sw1 = 1;          // Reset button (active low)
    reg sw2 = 0;          // Start button
    reg sw3 = 0;
    reg sw4 = 0;
    wire led1, led2, led3, led4;
    
    // Instantiate the Unit Under Test (UUT)
    main uut (
        .CLK(clk),
        .SW1(sw1),
        .SW2(sw2),
        .SW3(sw3),
        .SW4(sw4),
        .LED1(led1),
        .LED2(led2),
        .LED3(led3),
        .LED4(led4)
    );
    
    // Clock generation (100MHz)
    always #5 clk = ~clk;
    
    // Test procedure
    initial begin
        $dumpfile("main_tb.vcd");
        $dumpvars(0, main_tb);
        
        // Apply reset
        #10;
        sw1 = 0;  // Assert reset
        #20;
        sw1 = 1;  // Release reset
        #10;
        
        // Verify initial state
        if (uut.pc !== 4'd0 || uut.running !== 1'b0) begin
            $display("FAIL: Initial state incorrect");
        end else begin
            $display("PASS: Initial state correct");
        end
        
        // Start program execution
        sw2 = 1;  // Press start button
        #10;
        sw2 = 0;  // Release start button
        
        // Wait for program to complete (7 instructions)
        #150;
        
        // Check final result (should be 6 = (3 + 5) - 2)
        if ({led4, led3, led2, led1} !== 4'b0110) begin
            $display("FAIL: LED output incorrect. Expected 0110, got %b%b%b%b", led4, led3, led2, led1);
        end else begin
            $display("PASS: LED output shows correct result 6");
        end
        
        // Test register values
        if (uut.reg_a_data !== 4'd6) begin
            $display("FAIL: Register A value incorrect. Expected 6, got %d", uut.reg_a_data);
        end else begin
            $display("PASS: Register A contains correct result 6");
        end
        
        // Test program counter and halt
        if (uut.pc !== 4'd6 || uut.running !== 1'b0) begin
            $display("FAIL: Program did not halt correctly");
        end else begin
            $display("PASS: Program halted at correct instruction");
        end
        
        // Test reset after execution
        sw1 = 0;  // Assert reset
        #20;
        sw1 = 1;  // Release reset
        #10;
        
        if (uut.pc !== 4'd0 || uut.running !== 1'b0 || uut.led_output !== 4'd0) begin
            $display("FAIL: Reset after execution failed");
        end else begin
            $display("PASS: Reset after execution successful");
        end
        
        // Run program again to verify repeatability
        sw2 = 1;  // Press start button
        #10;
        sw2 = 0;  // Release start button
        
        // Wait for program to complete again
        #150;
        
        // Check final result again
        if ({led4, led3, led2, led1} !== 4'b0110) begin
            $display("FAIL: Second run LED output incorrect");
        end else begin
            $display("PASS: Second run LED output correct");
        end
        
        $display("Testbench completed");
        $finish;
    end

endmodule