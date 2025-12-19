**Hardware: Altera DE1-SoC (Cyclone V FPGA) Tools: Verilog HDL, Quartus Prime, ModelSim, DESim**

For ECE241 (Digital Systems), my partner and I designed the famous side scrolling mobile game Geometry Dash using digital system that handles real-time graphics rendering, physics simulation, and user input using the Altera DE1-Soc board. The goal was to demonstrate proficiency in digital logic design, moving from simulation in ModelSim and DESim to synthesis and hardware deployment on the DE1-SoC board.

Game State Control: Designed a master FSM to switch seamlessly between the Start Menu, Active Gameplay, and Game Over states based on reset signals and collision flags.

VGA Controller: Built a custom driver to interface with the VGA adapter, outputting a 640x480 resolution at 60Hz.

Protocol Decoding: Interfaced directly with the FPGA’s PS/2 port to read raw scan codes from an external keyboard.

ModelSim: Used for rigorous unit testing of individual modules (counters, shifters, and FSMs) to verify timing diagrams and logic correctness before compilation.

Sprite Storage: Utilized FPGA Block RAM to store game displays (player character, obstacles) efficiently on-chip.
