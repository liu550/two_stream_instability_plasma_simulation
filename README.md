# 1-dimensional two stream instability plasma simulation
Use Particle in Cell method to simulate the turbulent phenomenon followed by two streams of plasma encountering each other (equal charge, opposite velocity with same magnitude). The simulation was run on Unity, with x-axis representing the position of a particle and y-axis representing its velocity. A multithreading approach was implemented using Unity C# Job System. The result was that it did not perform any better, and the motions of particles demonstrate a less-obvious rotational pattern in the position-velocity space compared to the single-thread implementation. I have not obtained a satisfactory explanation for either problem.

**Demo link (single-thread)**: https://drive.google.com/file/d/155FZ5QPE00juUvsGLPc7tIjgGmge6JFN/view?usp=sharing
**Demo link (multi-thread)**: https://drive.google.com/file/d/1esmV_vhgob64qfKFg_GTibPOzHiPs9hZ/view?usp=sharing
