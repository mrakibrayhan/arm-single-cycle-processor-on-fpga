# ARMv4-compatible single-cycle processor on FPGA (ARM-SC)

An ARMv4-compatible (ARM-state subset) single-cycle processor implemented in Verilog and tested on the RZ-EasyFPGA A2.2 board (Intel Cyclone IV E EP4CE6E22C8).

> **Status:** complete and tested on hardware. Compiles with Intel Quartus Prime Lite.

## At a glance

| Item | Value |
|---|---|
| ISA | ARMv4-compatible (ARM-state subset: data processing with barrel shifter, LDR/STR, LDM/STM, B/BL/BX, MUL/MLA, long multiply) |
| Microarchitecture | single-cycle: Single-cycle: one instruction per clock, CPI = 1. |
| Language | Verilog / SystemVerilog |
| Board | RZ-EasyFPGA A2.2 (Intel Cyclone IV E EP4CE6E22C8, 6,272 LEs, 30 M9K) |
| Tool | Intel Quartus Prime Lite (built with 25.1std) |
| Top-level entity | `top_cpu_arm` |
| Board clock | 50 MHz input divided by 4 = 12.5 MHz CPU clock |

## Memory system
Asynchronous-read memories built from logic elements (single-cycle). Board build: 128-word ROM, 32-word RAM. Optional `MEM_M9K` parameter (file `mem_m9k_arm_sc.sv`) puts the memories into M9K blocks for larger programs; the RAM loads `data.hex`; CPI stays 1. To use it, add `mem_m9k_arm_sc.sv` to the Quartus project (Project > Add/Remove Files in Project) and set `MEM_M9K = 1`; it is not in the default file list.

## Repository contents
- `alu_arm.v`
- `condlogic_arm.v`
- `cpu_core_arm.v`
- `datapath_arm.v`
- `debounce.v`
- `decoder_arm.v`
- `dmem_arm.v`
- `ext_arm.v`
- `imem_arm.v`
- `ldmstm_seq_arm.v`
- `mem_m9k_arm_sc.sv`
- `mul_arm.v`
- `regfile_arm.v`
- `seg7_decoder.v`
- `seg7_display.v`
- `shifter_arm.v`
- `snapshot_arm.v`
- `top_cpu_arm.v`
- `timing_constraints.sdc`
- `program.hex`
- `data.hex`
- `assembler_arm.py`
- `ARM_Single_Cycle_Cpu.qpf`
- `ARM_Single_Cycle_Cpu.qsf`
- `ARM_Single_Cycle_Cpu.spf`

- `LICENSE`, `.gitignore`, `README.md`

## Build (Quartus)
1. Install Intel Quartus Prime Lite (the Cyclone IV E family is included).
2. Open `ARM_Single_Cycle_Cpu.qpf`.
3. Processing > Start Compilation.
4. Programmer: load `output_files/ARM_Single_Cycle_Cpu.sof` (not stored in the repository; see Releases for a ready-made `.sof`).

The `.qsf` already contains the device (`EP4CE6E22C8`), the pin assignments and the file list.

## Run on the board
- **KEY[3]** = reset (the CPU also stays in reset until KEY[2] is pressed once). **KEY[2]** = run.
- LEDG (active low = lit): [0] fault, [1] started, [2] Z flag, [3] halted.
- Stop rule: `SWI`, or `B .` with condition true halts; illegal encoding or BX to an odd address = fault.
- The program is loaded from `program.hex`, the initial RAM from `data.hex`, both when the FPGA is programmed. Reset does not reload the RAM; program the FPGA again to restore it.
- **Snapshot / debug:** when the program halts, or when the ISSP source bit is set, the CPU is frozen and registers and RAM words are copied into small RAMs. Read them in Quartus with *Tools > In-System Memory Content Editor*; read counters and status with *Tools > In-System Sources and Probes Editor*. CPI = cycle counter / instruction counter.

## Writing your own program
`assembler_arm.py` (Python 3, no extra packages) turns an assembly file into the `program.hex` that the CPU loads when the FPGA is programmed:
```
python assembler_arm.py prog.s -o program.hex --depth 128
```
Copy the new `program.hex` (and `data.hex` if the assembler wrote one) into the Quartus project folder and compile again. Run `python assembler_arm.py --help` for all options. The core stops on `SWI`, or on `B .` (branch to itself).

## Limitations
- Subset ISA, no interrupts, no caches, no MMU, no floating point.
{c['lim']}

## Related repositories
This is one of 9 individual processor projects on FPGA: MIPS-compatible, RISC-V and ARMv4-compatible, each as single-cycle, multi-cycle and pipelined. <!-- add links when ready -->

## License
MIT, see `LICENSE`.

## Trademarks
ARM is a trademark of Arm Limited; this project is an independent, educational design that is *compatible with a subset of the ARMv4 ARM-state instruction set*. It is not an Arm-designed core and is not affiliated with or endorsed by Arm Limited.

## Author
Md. Rakib Rayhan, Dept. of ECE, North South University. Research Assistant, Microprocessor, Embedded Systems and IoT Lab (MESIL).
