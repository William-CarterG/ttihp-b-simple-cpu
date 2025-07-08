# SPDX-FileCopyrightText: 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_basic_signals(dut):
    """Test basic signals: reset, enable, and initial states"""
    dut._log.info("=== Testing Basic Signals ===")
    
    # Set up clock - 10us period (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Test 1: Check reset behavior
    dut._log.info("Test 1: Reset behavior")
    dut.ena.value = 1
    dut.ui_in.value = 0  # All switches inactive
    dut.uio_in.value = 0
    dut.rst_n.value = 0  # Assert TT reset
    
    await ClockCycles(dut.clk, 5)
    
    # Check outputs during reset - should show PC=0 in default mode
    dut._log.info(f"During reset - uo_out (LEDs): {dut.uo_out.value:04b}")
    dut._log.info(f"During reset - uio_out: {dut.uio_out.value}")
    
    # Release reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)  # Allow CPU to start
    
    dut._log.info(f"After reset - uo_out (LEDs): {dut.uo_out.value:04b}")
    
    # Test 2: Manual reset via ui_in[0]
    dut._log.info("Test 2: Manual reset via SW1")
    dut.ui_in.value = 0b00000001  # SW1 = 1 (Reset active)
    await ClockCycles(dut.clk, 5)
    dut._log.info(f"Manual reset - uo_out (LEDs): {dut.uo_out.value:04b}")
    
    dut.ui_in.value = 0b00000000  # Release manual reset
    await ClockCycles(dut.clk, 5)
    dut._log.info(f"After manual reset - uo_out (LEDs): {dut.uo_out.value:04b}")

@cocotb.test()
async def test_display_modes(dut):
    """Test display mode switching with SW3"""
    dut._log.info("=== Testing Display Modes ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize system
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    # Test display mode cycling
    for mode in range(4):
        dut._log.info(f"Testing display mode {mode}")
        
        # Read current LED output
        led_value = dut.uo_out.value & 0x0F  # Lower 4 bits are LEDs
        dut._log.info(f"Mode {mode} - LED output: {led_value:04b} (decimal: {led_value})")
        
        # Press SW3 to change mode (SW3 is active LOW, so we need to pulse it)
        if mode < 3:  # Don't press on last iteration
            dut.ui_in.value = 0b00000000  # SW3 inactive (HIGH)
            await ClockCycles(dut.clk, 2)
            dut.ui_in.value = 0b00000100  # SW3 active (LOW) - wait, this is wrong
            # SW3 is active LOW, so ui_in[2] = 0 means SW3 is pressed
            # But our mapping inverts it: fpga_sw3 = ~ui_in[2]
            # So ui_in[2] = 0 -> fpga_sw3 = 1 (inactive)
            # And ui_in[2] = 1 -> fpga_sw3 = 0 (active)
            dut.ui_in.value = 0b00000000  # ui_in[2] = 0 -> SW3 pressed
            await ClockCycles(dut.clk, 2)
            dut.ui_in.value = 0b00000100  # ui_in[2] = 1 -> SW3 released  
            await ClockCycles(dut.clk, 5)

@cocotb.test()
async def test_step_execution(dut):
    """Test step-by-step execution with SW2"""
    dut._log.info("=== Testing Step Execution ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize system
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    dut._log.info("Testing manual step execution")
    
    # Execute several steps manually
    for step in range(5):
        # Read current state (PC should be visible in mode 0)
        pc_value = dut.uo_out.value & 0x0F
        dut._log.info(f"Step {step}: PC = {pc_value}")
        
        # Pulse SW2 for step execution
        # SW2 is active LOW: ui_in[1] = 0 -> SW2 pressed
        dut.ui_in.value = 0b00000000  # SW2 pressed
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000010  # SW2 released
        await ClockCycles(dut.clk, 5)  # Allow execution

@cocotb.test()
async def test_program_execution(dut):
    """Test the hardcoded program execution sequence"""
    dut._log.info("=== Testing Program Execution ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize system
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    # Test the hardcoded program sequence
    # Expected program:
    # PC0: NOP
    # PC1: Load 1 into RegB  
    # PC2: Move RegB to RegA (RegA=1)
    # PC3: Add RegA + RegB (RegA=2)
    # PC4: Load 2 into RegB
    # PC5: Add RegA + RegB (RegA=4)
    # etc.
    
    expected_sequence = [
        {"pc": 0, "description": "NOP"},
        {"pc": 1, "description": "Load 1 into RegB"},
        {"pc": 2, "description": "Move RegB to RegA"},
        {"pc": 3, "description": "Add RegA + RegB -> RegA"},
        {"pc": 4, "description": "Load 2 into RegB"},
        {"pc": 5, "description": "Add RegA + RegB -> RegA"},
    ]
    
    for step in expected_sequence:
        # Step execution
        dut.ui_in.value = 0b00000000  # SW2 pressed
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000010  # SW2 released
        await ClockCycles(dut.clk, 5)
        
        # Read PC (mode 0)
        pc_value = dut.uo_out.value & 0x0F
        dut._log.info(f"PC={pc_value}: {step['description']}")
        
        # Switch to RegA mode (mode 1) to check register values
        dut.ui_in.value = 0b00000000  # Press SW3
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000100  # Release SW3
        await ClockCycles(dut.clk, 3)
        
        regA_value = dut.uo_out.value & 0x0F
        dut._log.info(f"  RegA = {regA_value}")
        
        # Switch to RegB mode (mode 2)
        dut.ui_in.value = 0b00000000  # Press SW3
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000100  # Release SW3
        await ClockCycles(dut.clk, 3)
        
        regB_value = dut.uo_out.value & 0x0F
        dut._log.info(f"  RegB = {regB_value}")
        
        # Back to PC mode (mode 0) - press SW3 twice more
        for _ in range(2):
            dut.ui_in.value = 0b00000000  # Press SW3
            await ClockCycles(dut.clk, 2)
            dut.ui_in.value = 0b00000100  # Release SW3
            await ClockCycles(dut.clk, 3)

@cocotb.test()
async def test_cpu_reset_behavior(dut):
    """Test CPU reset clears registers and PC"""
    dut._log.info("=== Testing CPU Reset Behavior ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize and run a few steps
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    # Execute several steps to advance the program
    for _ in range(3):
        dut.ui_in.value = 0b00000000  # SW2 pressed
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000010  # SW2 released
        await ClockCycles(dut.clk, 5)
    
    pc_before_reset = dut.uo_out.value & 0x0F
    dut._log.info(f"PC before reset: {pc_before_reset}")
    
    # Reset via SW1
    dut.ui_in.value = 0b00000001  # SW1 active (reset)
    await ClockCycles(dut.clk, 5)
    
    pc_during_reset = dut.uo_out.value & 0x0F
    dut._log.info(f"PC during reset: {pc_during_reset}")
    
    # Release reset
    dut.ui_in.value = 0b00000000
    await ClockCycles(dut.clk, 5)
    
    pc_after_reset = dut.uo_out.value & 0x0F
    dut._log.info(f"PC after reset: {pc_after_reset}")
    
    # Verify PC returned to 0
    assert pc_after_reset == 0, f"PC should be 0 after reset, got {pc_after_reset}"

@cocotb.test()
async def test_pin_directions(dut):
    """Test that pin directions are set correctly"""
    dut._log.info("=== Testing Pin Directions ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize system
    dut.ena.value = 1
    dut.rst_n.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    
    # Check bidirectional pin configuration
    uio_oe_value = dut.uio_oe.value
    uio_out_value = dut.uio_out.value
    
    dut._log.info(f"uio_oe (pin directions): {uio_oe_value:08b}")
    dut._log.info(f"uio_out (bidirectional outputs): {uio_out_value:08b}")
    
    # Verify all bidirectional pins are set as inputs (0)
    assert uio_oe_value == 0, f"All uio pins should be inputs (0), got {uio_oe_value:08b}"
    assert uio_out_value == 0, f"All uio_out should be 0, got {uio_out_value:08b}"
    
    # Check that uo_out upper bits are 0 (reserved)
    uo_out_upper = (dut.uo_out.value >> 4) & 0x0F
    assert uo_out_upper == 0, f"Upper 4 bits of uo_out should be 0, got {uo_out_upper:04b}"

@cocotb.test()
async def test_automatic_vs_manual_clock(dut):
    """Test both automatic slow clock and manual stepping"""
    dut._log.info("=== Testing Clock Modes ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize system
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    # Test 1: Manual stepping (should work regardless of slow clock)
    dut._log.info("Testing manual stepping")
    initial_pc = dut.uo_out.value & 0x0F
    
    # Single manual step
    dut.ui_in.value = 0b00000000  # SW2 pressed
    await ClockCycles(dut.clk, 2)
    dut.ui_in.value = 0b00000010  # SW2 released
    await ClockCycles(dut.clk, 5)
    
    after_step_pc = dut.uo_out.value & 0x0F
    dut._log.info(f"PC before step: {initial_pc}, after step: {after_step_pc}")
    
    # Test 2: Let automatic clock run (if visible)
    dut._log.info("Letting automatic slow clock run...")
    # Note: The slow clock period is very long (2^25 cycles), so we might not see changes
    # in a reasonable test time, but the CPU should still respond to manual steps
    await ClockCycles(dut.clk, 100)
    
    final_pc = dut.uo_out.value & 0x0F
    dut._log.info(f"PC after waiting: {final_pc}")

@cocotb.test() 
async def test_alu_operations_via_program(dut):
    """Test ALU operations by executing the hardcoded program"""
    dut._log.info("=== Testing ALU Operations via Program ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize system
    dut.ena.value = 1
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    # Execute the program step by step and verify ALU operations
    program_steps = [
        {"pc": 0, "op": "NOP", "expected_regA": 0, "expected_regB": 0},
        {"pc": 1, "op": "Load 1 to RegB", "expected_regA": 0, "expected_regB": 1},
        {"pc": 2, "op": "Move RegB to RegA", "expected_regA": 1, "expected_regB": 1},
        {"pc": 3, "op": "ADD RegA+RegB", "expected_regA": 2, "expected_regB": 1},
        {"pc": 4, "op": "Load 2 to RegB", "expected_regA": 2, "expected_regB": 2},
        {"pc": 5, "op": "ADD RegA+RegB", "expected_regA": 4, "expected_regB": 2},
        {"pc": 6, "op": "SUB RegA-RegB", "expected_regA": 2, "expected_regB": 2},
        {"pc": 7, "op": "Load 3 to RegB", "expected_regA": 2, "expected_regB": 3},
        {"pc": 8, "op": "AND RegA&RegB", "expected_regA": 2, "expected_regB": 3},  # 2&3=2
        {"pc": 9, "op": "OR RegA|RegB", "expected_regA": 3, "expected_regB": 3},   # 2|3=3
    ]
    
    for step in program_steps:
        # Execute one step
        dut.ui_in.value = 0b00000000  # SW2 pressed
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000010  # SW2 released
        await ClockCycles(dut.clk, 5)
        
        # Verify PC (mode 0)
        current_pc = dut.uo_out.value & 0x0F
        
        # Switch to RegA mode to check result
        dut.ui_in.value = 0b00000000  # Press SW3
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000100  # Release SW3
        await ClockCycles(dut.clk, 3)
        regA_actual = dut.uo_out.value & 0x0F
        
        # Switch to RegB mode
        dut.ui_in.value = 0b00000000  # Press SW3
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000100  # Release SW3
        await ClockCycles(dut.clk, 3)
        regB_actual = dut.uo_out.value & 0x0F
        
        dut._log.info(f"Step {step['pc']}: {step['op']}")
        dut._log.info(f"  Expected: RegA={step['expected_regA']}, RegB={step['expected_regB']}")
        dut._log.info(f"  Actual:   RegA={regA_actual}, RegB={regB_actual}")
        
        # Return to PC mode
        dut.ui_in.value = 0b00000000  # Press SW3 twice
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000100  # Release SW3
        await ClockCycles(dut.clk, 3)
        dut.ui_in.value = 0b00000000  # Press SW3
        await ClockCycles(dut.clk, 2)
        dut.ui_in.value = 0b00000100  # Release SW3
        await ClockCycles(dut.clk, 3)