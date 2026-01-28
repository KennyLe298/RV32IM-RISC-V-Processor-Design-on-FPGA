# RV32IM RISC-V Processor Design on FPGA

![Status](https://img.shields.io/badge/Status-Completed-success)
![Language](https://img.shields.io/badge/Language-Verilog-blue)
![Hardware](https://img.shields.io/badge/Hardware-Arty--Z7--20-orange)
![Toolchain](https://img.shields.io/badge/Tools-Xilinx_Vivado-red)

## 📌 Project Overview

This repository contains the complete Register-Transfer Level (RTL) implementation of a 32-bit RISC-V processor supporting the **RV32IM** instruction set (Integer + Multiplication/Division).

The project documents the evolution of the core architecture through three distinct design phases, culminating in a **5-stage pipelined processor** capable of resolving data hazards and control hazards in hardware. The design was synthesized and verified on the **Digilent Arty Z7-20 FPGA** board.

**Key Achievements:**
* Achieved **timing closure** for all design phases.
* Implemented a custom **8-stage pipelined divider** to optimize critical path delay.
* Validated using industry-standard RISC-V test suites and cycle-accurate tracing.

## 📂 Repository Structure

The repository is organized into three phases, reflecting the architectural evolution of the core:

| Folder | Architecture | Description |
| :--- | :--- | :--- |
| **`01_Single_Cycle_Core`** | Single-Cycle | Fundamental implementation executing one instruction per clock cycle. Includes PC, Register File, and ALU integration. |
| **`02_Multi_Cycle_Core`** | Multi-Cycle | Refactored design to reuse hardware resources. Integrated a high-performance **8-stage pipelined divider**. |
| **`03_5_Stage_Pipelined_Core`** | 5-Stage Pipeline | The final, most advanced core. Features full pipelining (F-D-E-M-W), **hazard detection unit**, **forwarding unit**, and dynamic branch handling. |

## 🛠️ Technical Implementation Details

### Phase 1: Single-Cycle Datapath
* **Objective:** Build the foundational architectural state.
* **Features:**
    * Implemented the basic Fetch-Decode-Execute loop.
    * Integrated a **Carry-Lookahead Adder (CLA)** for high-speed arithmetic operations.
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

## 🚀 Getting Started

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/YourUsername/RV32IM-RISC-V-Processor-Design-on-FPGA.git](https://github.com/YourUsername/RV32IM-RISC-V-Processor-Design-on-FPGA.git)
