# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
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
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0  # Assert reset
    
    await ClockCycles(dut.clk, 5)
    
    # Check outputs during reset
    dut._log.info(f"During reset - uo_out: {dut.uo_out.value}")
    dut._log.info(f"During reset - uio_out: {dut.uio_out.value}")
    
    # Release reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    dut._log.info(f"After reset - uo_out: {dut.uo_out.value}")
    dut._log.info(f"After reset - uio_out: {dut.uio_out.value}")
    
    # Test 2: Check enable signal
    dut._log.info("Test 2: Enable signal")
    dut.ena.value = 0  # Disable
    await ClockCycles(dut.clk, 2)
    dut._log.info(f"With ena=0 - uo_out: {dut.uo_out.value}")
    
    dut.ena.value = 1  # Enable
    await ClockCycles(dut.clk, 2)
    dut._log.info(f"With ena=1 - uo_out: {dut.uo_out.value}")
    
    # Test 3: Simple input/output test
    dut._log.info("Test 3: Basic input/output")
    dut.ui_in.value = 0b00000001  # Set ALU op to 001 (subtraction)
    await ClockCycles(dut.clk, 1)
    dut._log.info(f"ui_in set to 1 - uo_out: {dut.uo_out.value}")
    
    dut.ui_in.value = 0b00000010  # Set ALU op to 010 (AND)
    await ClockCycles(dut.clk, 1)
    dut._log.info(f"ui_in set to 2 - uo_out: {dut.uo_out.value}")
    
    dut._log.info("=== Basic signals test completed ===")

@cocotb.test()
async def test_alu_simple(dut):
    """Test ALU with hardcoded zero inputs"""
    dut._log.info("=== Testing ALU with Zero Inputs ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Force both ALU inputs to zero using mux selects
    # reg_a_sel=1 (zero), reg_b_sel=1 (zero)
    base_control = 0b01100000  # Both mux selects set to 1 (zero)
    
    # Test each ALU operation with 0,0 inputs
    operations = [
        (0b000, "ADD", 0),      # 0 + 0 = 0
        (0b001, "SUB", 0),      # 0 - 0 = 0  
        (0b010, "AND", 0),      # 0 & 0 = 0
        (0b011, "OR", 0),       # 0 | 0 = 0
        (0b100, "NOT", 15),     # ~0 = 15 (0xF in 4-bit)
        (0b101, "XOR", 0),      # 0 ^ 0 = 0
        (0b110, "LSHIFT", 0),   # 0 << 1 = 0
        (0b111, "RSHIFT", 0),   # 0 >> 1 = 0
    ]
    
    for op_code, op_name, expected in operations:
        dut.ui_in.value = base_control | op_code
        await ClockCycles(dut.clk, 1)
        
        alu_result = dut.uo_out.value & 0xF  # Get lower 4 bits
        dut._log.info(f"{op_name} (0,0): got {alu_result}, expected {expected}")
        
        if alu_result != expected:
            dut._log.error(f"FAILED: {op_name} expected {expected}, got {alu_result}")
        else:
            dut._log.info(f"PASSED: {op_name}")

@cocotb.test()
async def test_register_basic(dut):
    """Test basic register functionality"""
    dut._log.info("=== Testing Basic Register Functionality ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Check initial register states
    dut._log.info("Checking initial register states...")
    reg_a_initial = (dut.uo_out.value >> 4) & 0xF
    reg_b_initial = (dut.uio_out.value >> 4) & 0xF
    dut._log.info(f"Initial RegA: {reg_a_initial}, RegB: {reg_b_initial}")
    
    # Test Register B loading
    dut._log.info("Testing Register B loading...")
    dut.uio_in.value = 0b00000101  # Set data_in to 5
    dut.ui_in.value = 0b00010000   # Set reg_b_en = 1 (bit 4)
    await ClockCycles(dut.clk, 1)
    
    # Check if Register B changed
    reg_b_after = (dut.uio_out.value >> 4) & 0xF
    dut._log.info(f"After loading: RegB = {reg_b_after} (expected: 5)")
    
    # Disable reg_b_en and check it holds value
    dut.ui_in.value = 0b00000000   # Clear all enables
    await ClockCycles(dut.clk, 2)
    
    reg_b_hold = (dut.uio_out.value >> 4) & 0xF
    dut._log.info(f"After disabling enable: RegB = {reg_b_hold} (should still be 5)")
    
    dut._log.info("=== Basic register test completed ===")

@cocotb.test()
async def test_pin_directions(dut):
    """Test that pin directions are set correctly"""
    dut._log.info("=== Testing Pin Directions ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Check uio_oe (should be 0xF0 - lower 4 bits input (0), upper 4 bits output (1))
    expected_oe = 0b11110000
    actual_oe = int(dut.uio_oe.value)
    dut._log.info(f"uio_oe: {actual_oe:08b} (expected: {expected_oe:08b})")
    
    if actual_oe == expected_oe:
        dut._log.info("PASSED: Pin directions correct")
    else:
        dut._log.error(f"FAILED: Pin directions wrong, got {actual_oe:08b}")
    
    dut._log.info("=== Pin direction test completed ===")

@cocotb.test()
async def test_one_simple_operation(dut):
    """Test one complete operation step by step"""
    dut._log.info("=== Testing One Simple Operation ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    dut._log.info("Step 1: Load 3 into Register B")
    dut.uio_in.value = 0b00000011  # data_in = 3
    dut.ui_in.value = 0b00010000   # reg_b_en = 1
    await ClockCycles(dut.clk, 1)
    
    reg_b = (dut.uio_out.value >> 4) & 0xF
    dut._log.info(f"RegB after loading: {reg_b}")
    
    dut._log.info("Step 2: Perform 0 + 3 (RegA=0, RegB=3)")
    # reg_a_sel=1 (zero), reg_b_sel=0 (register), alu_op=000 (add)
    dut.ui_in.value = 0b00100000  # 0010 0000
    await ClockCycles(dut.clk, 1)
    
    alu_result = dut.uo_out.value & 0xF
    dut._log.info(f"ALU result: {alu_result} (expected: 3)")
    
    if alu_result == 3:
        dut._log.info("PASSED: Simple operation works!")
    else:
        dut._log.error(f"FAILED: Expected 3, got {alu_result}")
    
    dut._log.info("=== Simple operation test completed ===")

@cocotb.test()
async def test_register_a_storage(dut):
    """Test Register A can store ALU results"""
    dut._log.info("=== Testing Register A Storage ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Load RegB with 7
    dut._log.info("Loading RegB with 7")
    dut.uio_in.value = 0b00000111  # data_in = 7
    dut.ui_in.value = 0b00010000   # reg_b_en = 1
    await ClockCycles(dut.clk, 1)
    
    # Do operation: 0 + 7, then store in RegA
    dut._log.info("Computing 0 + 7 and storing in RegA")
    dut.ui_in.value = 0b00101000   # reg_a_en=1, reg_a_sel=1(zero), reg_b_sel=0(reg), alu_op=000
    await ClockCycles(dut.clk, 1)
    
    # Check RegA contains 7
    reg_a = (int(dut.uo_out.value) >> 4) & 0xF
    dut._log.info(f"RegA after storage: {reg_a} (expected: 7)")
    
    if reg_a == 7:
        dut._log.info("PASSED: RegA storage works!")
    else:
        dut._log.error(f"FAILED: Expected 7, got {reg_a}")

@cocotb.test()
async def test_all_alu_operations_with_data(dut):
    """Test all ALU operations with real data"""
    dut._log.info("=== Testing All ALU Operations with Real Data ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Load RegB with 5
    dut.uio_in.value = 0b00000101  # data_in = 5
    dut.ui_in.value = 0b00010000   # reg_b_en = 1
    await ClockCycles(dut.clk, 1)
    
    # Store 3 in RegA (0 + 3)
    dut.uio_in.value = 0b00000011  # data_in = 3
    dut.ui_in.value = 0b00010000   # Load 3 into RegB temporarily
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00101000   # Store ALU result (0+3) in RegA
    await ClockCycles(dut.clk, 1)
    
    # Restore RegB to 5
    dut.uio_in.value = 0b00000101
    dut.ui_in.value = 0b00010000
    await ClockCycles(dut.clk, 1)
    
    # Now test all operations with RegA=3, RegB=5
    operations = [
        (0b000, "ADD", 3+5),        # 3 + 5 = 8
        (0b001, "SUB", (3-5)&0xF),  # 3 - 5 = -2 → 14 in 4-bit
        (0b010, "AND", 3&5),        # 3 & 5 = 1
        (0b011, "OR", 3|5),         # 3 | 5 = 7
        (0b100, "NOT", (~3)&0xF),   # ~3 = 12 in 4-bit
        (0b101, "XOR", 3^5),        # 3 ^ 5 = 6
        (0b110, "LSHIFT", (3<<1)&0xF), # 3 << 1 = 6
        (0b111, "RSHIFT", 3>>1),    # 3 >> 1 = 1
    ]
    
    for op_code, op_name, expected in operations:
        if op_name == "NOT":
            # NOT only uses RegA, set RegB to zero
            dut.ui_in.value = 0b01000000 | op_code  # reg_b_sel=1 (zero)
        else:
            # Use both registers
            dut.ui_in.value = 0b00000000 | op_code  # Both regs selected
        
        await ClockCycles(dut.clk, 1)
        
        alu_result = int(dut.uo_out.value) & 0xF
        dut._log.info(f"{op_name}: got {alu_result}, expected {expected}")
        
        if alu_result == expected:
            dut._log.info(f"PASSED: {op_name}")
        else:
            dut._log.error(f"FAILED: {op_name} expected {expected}, got {alu_result}")

@cocotb.test()
async def test_computer_sequence(dut):
    """Test a complete computer operation sequence"""
    dut._log.info("=== Testing Complete Computer Sequence ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    dut._log.info("Sequence: Load 6 → Add 4 → Store → Subtract 2")
    
    # Step 1: Load 6 into RegB
    dut._log.info("Step 1: Load 6 into RegB")
    dut.uio_in.value = 0b00000110  
    dut.ui_in.value = 0b00010000   
    await ClockCycles(dut.clk, 1)
    
    # Step 2: Add 0 + 6, store in RegA
    dut._log.info("Step 2: Compute 0 + 6, store in RegA")
    dut.ui_in.value = 0b00101000   # reg_a_en=1, reg_a_sel=1, alu_op=000
    await ClockCycles(dut.clk, 1)
    
    reg_a = (int(dut.uo_out.value) >> 4) & 0xF
    dut._log.info(f"RegA now: {reg_a} (expected: 6)")
    
    # Step 3: Load 4 into RegB
    dut._log.info("Step 3: Load 4 into RegB")
    dut.uio_in.value = 0b00000100
    dut.ui_in.value = 0b00010000
    await ClockCycles(dut.clk, 1)
    
    # Step 4: Add RegA + RegB (6 + 4)
    dut._log.info("Step 4: Add RegA + RegB (6 + 4)")
    dut.ui_in.value = 0b00000000   # Both regs, alu_op=000
    await ClockCycles(dut.clk, 1)
    
    alu_result = int(dut.uo_out.value) & 0xF
    dut._log.info(f"ALU result: {alu_result} (expected: 10)")
    
    # Step 5: Store result in RegA
    dut._log.info("Step 5: Store result (10) in RegA")
    dut.ui_in.value = 0b00001000   # reg_a_en=1, alu_op=000
    await ClockCycles(dut.clk, 1)
    
    # Step 6: Load 2 into RegB  
    dut._log.info("Step 6: Load 2 into RegB")
    dut.uio_in.value = 0b00000010
    dut.ui_in.value = 0b00010000
    await ClockCycles(dut.clk, 1)
    
    # Step 7: Subtract RegA - RegB (10 - 2)
    dut._log.info("Step 7: Subtract RegA - RegB (10 - 2)")
    dut.ui_in.value = 0b00000001   # alu_op=001 (subtract)
    await ClockCycles(dut.clk, 1)
    
    final_result = int(dut.uo_out.value) & 0xF
    dut._log.info(f"Final result: {final_result} (expected: 8)")
    
    if final_result == 8:
        dut._log.info("PASSED: Complete sequence works!")
    else:
        dut._log.error(f"FAILED: Expected 8, got {final_result}")

@cocotb.test()
async def test_overflow_behavior(dut):
    """Test 4-bit overflow behavior"""
    dut._log.info("=== Testing 4-bit Overflow Behavior ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Test addition overflow: 15 + 1 = 0 (in 4-bit)
    dut._log.info("Testing addition overflow: 15 + 1")
    
    # Load 15 into RegB
    dut.uio_in.value = 0b00001111  # 15
    dut.ui_in.value = 0b00010000   
    await ClockCycles(dut.clk, 1)
    
    # Store 15 in RegA
    dut.ui_in.value = 0b00101000   
    await ClockCycles(dut.clk, 1)
    
    # Load 1 into RegB
    dut.uio_in.value = 0b00000001  # 1
    dut.ui_in.value = 0b00010000   
    await ClockCycles(dut.clk, 1)
    
    # Add 15 + 1
    dut.ui_in.value = 0b00000000   # ADD
    await ClockCycles(dut.clk, 1)
    
    overflow_result = int(dut.uo_out.value) & 0xF
    expected = (15 + 1) & 0xF  # Should be 0
    dut._log.info(f"15 + 1 = {overflow_result} (expected: {expected})")
    
    if overflow_result == expected:
        dut._log.info("PASSED: Overflow behavior correct!")
    else:
        dut._log.error(f"FAILED: Expected {expected}, got {overflow_result}")

@cocotb.test()
async def test_mux_selects(dut):
    """Test multiplexer select functionality thoroughly"""
    dut._log.info("=== Testing Multiplexer Selects ===")
    
    # Set up clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Set up known register values
    # RegA = 9, RegB = 6
    dut.uio_in.value = 0b00001001  # Load 9
    dut.ui_in.value = 0b00010000   
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00101000   # Store in RegA
    await ClockCycles(dut.clk, 1)
    
    dut.uio_in.value = 0b00000110  # Load 6 into RegB
    dut.ui_in.value = 0b00010000   
    await ClockCycles(dut.clk, 1)
    
    # Test all 4 mux combinations
    test_cases = [
        (0, 0, "RegA + RegB", 9 + 6),     # reg_a_sel=0, reg_b_sel=0
        (0, 1, "RegA + 0", 9 + 0),        # reg_a_sel=0, reg_b_sel=1
        (1, 0, "0 + RegB", 0 + 6),        # reg_a_sel=1, reg_b_sel=0
        (1, 1, "0 + 0", 0 + 0),           # reg_a_sel=1, reg_b_sel=1
    ]
    
    for reg_a_sel, reg_b_sel, description, expected in test_cases:
        control = (reg_b_sel << 6) | (reg_a_sel << 5) | 0b000  # ADD operation
        dut.ui_in.value = control
        await ClockCycles(dut.clk, 1)
        
        result = int(dut.uo_out.value) & 0xF
        expected_4bit = expected & 0xF
        
        dut._log.info(f"{description}: got {result}, expected {expected_4bit}")
        
        if result == expected_4bit:
            dut._log.info(f"PASSED: {description}")
        else:
            dut._log.error(f"FAILED: {description} expected {expected_4bit}, got {result}")