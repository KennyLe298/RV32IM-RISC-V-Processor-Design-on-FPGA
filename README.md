# RV32IM RISC-V Processor Design on FPGA

![Status](https://img.shields.io/badge/Status-Completed-success)
![Language](https://img.shields.io/badge/Language-Verilog-blue)
![Hardware](https://img.shields.io/badge/Hardware-Arty--Z7--20-orange)
![Toolchain](https://img.shields.io/badge/Tools-Xilinx_Vivado-red)

## Project Overview

This repository contains the complete Register-Transfer Level (RTL) implementation of a 32-bit RISC-V processor supporting the RV32IM instruction set (Integer + Multiplication/Division).

## Structure

The repository is organized into three phases, reflecting the architectural evolution of the core:

| Folder | Architecture | Description |
| :--- | :--- | :--- |
| **`Single_Cycle`** | Single-Cycle | Fundamental implementation executing one instruction per clock cycle. Includes PC, Register File, and ALU integration. |
| **`Multi_Cycle`** | Multi-Cycle | Refactored design to reuse hardware resources. Integrated a high-performance 8-stage pipelined divider. |
| **`Pipelined`** | 5-Stage Pipeline | The final, most advanced core. Features full pipelining (F-D-E-M-W), hazard detection unit, forwarding unit, and dynamic branch handling. |

##  Implementation

### Phase 1: Single-Cycle Datapath
* **Objective:** Build the foundational architectural state.
* **Features:**
    * Implemented the basic Fetch-Decode-Execute loop.
    * Integrated a Carry-Lookahead Adder (CLA) for high-speed arithmetic operations.
    * Supported basic ALU instructions, branches (`beq` through `bgeu`), and `lui`/`ecall` support.

### Phase 2: Multi-Cycle Datapath & Pipelined Divider
* **Objective:** Optimize for higher clock frequencies and hardware reuse.
* **Features:**
    * **Pipelined Divider:** Converted a single-cycle divider into an **8-stage pipelined unit** to break the critical path and improve throughput.
    * **State Machine Control:** Transitioned to a finite state machine (FSM) control unit to manage instruction execution across multiple shorter clock cycles.

### Phase 3: 5-Stage Pipelined Core (Final Architecture)
* **Objective:** Maximize instruction throughput (ILP) using a classic 5-stage RISC pipeline.
* **Key Features:**
    * **Hazard Detection Unit:** Implemented stalling logic in the Decode stage to handle load-use dependencies.
    * **Forwarding Unit (Bypassing):** Solved data hazards without stalling by implementing **MX**, **WX**, and **WM** bypass paths, allowing dependent instructions to execute immediately.
    * **Control Hazard Handling:** Implemented branch prediction logic where the pipeline flushes Fetch and Decode stages upon a taken branch.
    * **Independent Divider Pipeline:** The divider operates as an independent functional unit, allowing non-dependent instructions to execute while a division is in progress.

## 💻 Hardware & Tools
* **Language:** Verilog HDL
* **FPGA Board:** Digilent Arty Z7-20 (Xilinx Zynq-7000 SoC)
* **EDA Tool:** Xilinx Vivado (Synthesis, Implementation, and Simulation)
* **ISA:** RISC-V (RV32IM)
