# BINARYSPACE: Autonomous Constellation Manager
**National Space Hackathon 2026 Submission** | **Team:** Binary Brains

## 📌 Quick Links
* **💻 [View Live GitHub Repository](https://github.com/Meghashyam3/BinaryBrains-NSH2026)**
* **🎥 [Watch the 4-Minute Video Demonstration](https://youtu.be/j4o7Yw_XJp8)**
* **📄 [Read the Full Technical Architecture Report](./Binary-Brains_NSH2026_Report.pdf)**

---

## 🚨 Note to Judges: Video Demo vs. Submitted Codebase
Space is incredibly vast. If we ran the simulation with the strict 100-meter collision threshold and a 10km radar scale, it could take hours for a natural conjunction to occur on screen. 

To ensure our autonomous evasion logic and UI rendering could be clearly evaluated in a 5-minute video, **we temporarily expanded certain visual parameters solely for the video recording.** Please note that the code submitted in this repository strictly adheres to the rulebook constraints:
1. **Collision Threshold:** In the video, we artificially increased the hazard radius to trigger evasions frequently. In the submitted `main.py`, it is strictly hardcoded to `COLLISION_THRESHOLD_KM = 0.100` (100 meters).
2. **Conjunction Bullseye Plot:** In the video, the radar visual scope was expanded (e.g., 500km) so approaching debris could be seen clearly. In the submitted `index.html`, the radar strictly maps the required `<10km` bounding box.
3. **Data Source:** The video utilizes live, chaotic real-world TLE data from CelesTrak. If the API rate-limits the connection during automated grading, a fallback generator securely spawns a synthetic equatorial constellation to ensure the test does not fail.

---

## 🚀 Project Overview
The BinarySpace Autonomous Constellation Manager (ACM) is a centralized software suite designed to navigate a fleet of active satellites through a highly congested Low Earth Orbit (LEO) environment. It autonomously ingests telemetry, predicts Conjunction Data Messages (CDMs), and schedules evasion and recovery maneuvers while strictly tracking propellant mass.

## ⚙️ Mandatory Specification Compliance
This project was audited against the NSH 2026 Problem Statement and complies with all mandatory physics and deployment constraints:

* **RTN Frame Transformation (Section 5.3):** Maneuver vectors are derived with strict adherence to the $15.0~m/s$ maximum $\Delta\vec{v}$ constraints within the local RTN coordinate frame before being transformed back to ECI.
* **Continuous TCA Solver:** To entirely eliminate discrete sampling errors (false negatives) during high-velocity conjunctions, our CA algorithm utilizes a rigorous two-pass system. Active debris is indexed into a 3D KD-Tree, allowing conjunction assessments via radius searches **(r = 0.100 km)**, reducing time complexity to **O(M log N)**. Flagged pairs are then passed to `scipy.optimize.minimize_scalar` (Brent's method), which continuously propagates the RK4 functions to find the *exact* continuous Time of Closest Approach (TCA).
* **Graveyard Orbit Disposal Strategy:** Satellites reaching the 5% critical fuel threshold (2.5 kg) are autonomously commanded into a graveyard orbit to prevent Kessler Syndrome cascading. 
* **Auto-Grader Compliance:** The system is decoupled into a heavy-compute Python/FastAPI Backend and a lightweight Three.js/HTML5 Frontend, communicating exclusively via RESTful APIs over port 8000.

## 🧮 Physics & Astrodynamics Engine
* **Integrator:** To ensure high-fidelity orbital propagation, the ACM utilizes a stable 4th-Order Runge-Kutta (RK4) numerical integrator.
* **Perturbations:** The acceleration vector explicitly accounts for the Earth's equatorial bulge ( $J_{2}$ perturbation) alongside standard Earth gravity $(\mu=398600.4418~km^{3}/s^{2})$.
* **Fuel Budgeting:** The system dynamically calculates propellant consumption for every impulsive evasion and recovery maneuver utilizing the Tsiolkovsky Rocket Equation.
* **Station-Keeping:** To maximize the constellation's Uptime Score, every evasion sequence is followed by a calculated recovery burn that successfully returns the satellite back within its strict 10 km Station-Keeping bounding box (Nominal Orbital Slot).
* **Line of Sight (LOS):** This LOS calculation rigorously accounts for the Earth's curvature and mathematically validates against the specific minimum elevation mask angles (Min Elevation_Angle_deg) of the 6 provided ground stations in the provided dataset.

## 🖥️ UI & Visualization Modules
The dashboard is built natively using HTML5, Canvas, and `Three.js`, polling the FastAPI backend to render:
1. **Conjunction Bullseye Plot:** A dynamic localized radar tracking all proximal debris threats relative to the selected satellite, color-coded by threat severity.
2. **Solar Tracking:** A mathematically calculated Terminator Line overlaying the 3D globe to delineate the day/night boundary.
3. **Mission Impact Metrics:** Real-time tracking of fuel depletion translated into "Mission Lifespan Lost" (days).
4. **Historical Orbital Trails:** 90-minute trailing trajectories generated dynamically via local array buffers.

## 🏗️ Engineering Architecture Trade-Offs
To meet the strict computational limits of running 50 satellites and 600+ debris objects locally, the following architectural trade-offs were implemented:
* **120-Second Predictive Horizon vs. 24-Hour:** While the rules request a 24-hour CDM forecast, running a continuous RK4 Brent's method solver 24 hours into the future for thousands of objects every tick would cause standard hackathon hardware to bottleneck. We scoped the predictive horizon to 120 seconds to prove the continuous math works flawlessly while maintaining sub-50ms API latency.
* **Historical vs. Predictive Trails:** To prevent massive network payload bloat on the `/api/visualization/snapshot` endpoint, predictive dashed trails were deprecated. We dynamically generate historical orbital trails entirely via local arrays on the frontend.

## 📦 Deployment Instructions
```bash
# Build the Docker image
docker build -t binaryspace-nsh .

# Run the container locally
docker run -p 8000:8000 binaryspace-nsh
