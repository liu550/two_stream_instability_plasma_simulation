# 1-Dimensional Two Stream Instability Plasma Simulation
Use Particle in Cell method to simulate the turbulent phenomenon followed by two streams of plasma encountering each other (equal charge, opposite velocity with same magnitude). The simulation was run on Unity, with x-axis representing the position of a particle and y-axis representing its velocity. A multithreading approach was implemented using Unity C# Job System. The result was that it did not perform any better, and the rotational pattern demonstrated by the motions of particles in the position-velocity space was less obvious compared to the one shown in the single-thread implementation. I have not yet obtained a satisfactory explanation for either problem.

**Demo Link (single-thread)**: https://drive.google.com/file/d/155FZ5QPE00juUvsGLPc7tIjgGmge6JFN/view?usp=sharing

**Demo link (multi-thread)**: https://drive.google.com/file/d/1esmV_vhgob64qfKFg_GTibPOzHiPs9hZ/view?usp=sharing
