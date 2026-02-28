"""
Aerodynamic Force & Moment Model
=================================
Computes lift, drag, and pitching moment for the Piaggio P.180 Avanti
three-surface configuration (canard + wing + horizontal tail).

References
----------
- AERO50002 Lecture Notes, Chapters 4 & 8
- Aircraft parameter data: see aircraft.py

TODO: Implement the following
------------------------------
1. Dynamic pressure computation  q̄ = ½ρV²
2. ISA atmosphere model  ρ(h)
3. Wing lift & drag  (CL_W, CD_W)
4. Canard lift & drag  (CL_C, CD_C) with upwash correction
5. Tail lift & drag  (CL_H, CD_H) with downwash correction
6. Fuselage pitching moment contribution
7. Total aerodynamic forces in wind axes → rotate to body axes
8. Pitching moment about CG  (wing + canard + tail + fuselage)
9. Thrust model  (turboprop: P = T·V, so T = P/V at cruise)
10. Flap effects on CL and CD (approach configuration)
11. Stall model  (CL cap + post-stall behaviour)
12. Compressibility corrections near M_DD = 0.65
"""
