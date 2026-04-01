import asyncio
import time
import math
import random
import os
import requests
from datetime import datetime, timezone
from fastapi import FastAPI
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel
from typing import List
from scipy.spatial import KDTree
from scipy.optimize import minimize_scalar
from sgp4.api import Satrec, jday
import uvicorn

app = FastAPI(title="BinarySpace: Autonomous Constellation Manager")

COLLISION_THRESHOLD_KM = 0.100  
EVASION_TIME_SEC = 600.0       
EARTH_RADIUS_KM = 6378.137
MAX_FUEL_KG = 50.0 
MU = 398600.4418               
J2 = 1.08263e-3                
ISP = 300.0                    
G0 = 9.80665                   
GRAVEYARD_LIMIT = 2.5          
MAX_THRUST_KM_S = 0.015        

GROUND_STATIONS = [
    {"id": "GS-001", "lat": 13.0333, "lon": 77.5167, "alt": 820, "min_el": 5.0},
    {"id": "GS-002", "lat": 78.2297, "lon": 15.4077, "alt": 400, "min_el": 5.0},
    {"id": "GS-003", "lat": 35.4266, "lon": -116.8900, "alt": 1000, "min_el": 10.0},
    {"id": "GS-004", "lat": -53.1500, "lon": -70.9167, "alt": 30, "min_el": 5.0},
    {"id": "GS-005", "lat": 28.5450, "lon": 77.1926, "alt": 225, "min_el": 15.0},
    {"id": "GS-006", "lat": -77.8463, "lon": 166.6682, "alt": 10, "min_el": 5.0},
]

class Vector3(BaseModel):
    x: float
    y: float
    z: float

class DebrisObject(BaseModel):
    id: str
    type: str
    r: Vector3
    v: Vector3

class TelemetryPayload(BaseModel):
    timestamp: str
    objects: List[DebrisObject]

class StepPayload(BaseModel):
    step_seconds: int

class SystemState:
    def __init__(self):
        self.satellites = {}
        self.debris = {}
        self.kd_latency_ms = 0.0
        
        print("Initializing BinarySpace Simulation Environment...")
        active_tles = self.fetch_tles("https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle", 50)
        debris_tles = self.fetch_tles("https://celestrak.org/NORAD/elements/gp.php?GROUP=cosmos-2251-debris&FORMAT=tle", 600)
        
        now = datetime.now(timezone.utc)
        jd, fr = jday(now.year, now.month, now.day, now.hour, now.minute, now.second)
        
        for idx, (name, sat) in enumerate(active_tles):
            e, r, v = sat.sgp4(jd, fr)
            if e == 0:
                sid = f"Satellite-{idx:02d}"
                self.satellites[sid] = {"id": sid, "fuel_kg": MAX_FUEL_KG, "status": "NOMINAL", "state": [r[0], r[1], r[2], v[0], v[1], v[2]], "nominal_state": [r[0], r[1], r[2], v[0], v[1], v[2]], "cooldown": 0.0}
                
        for idx, (name, sat) in enumerate(debris_tles):
            e, r, v = sat.sgp4(jd, fr)
            if e == 0:
                self.debris[f"DEB-{idx:04d}"] = {"id": f"DEB-{idx:04d}", "state": [r[0], r[1], r[2], v[0], v[1], v[2]]}

    def fetch_tles(self, url, max_count):
        try:
            # Added User-Agent to prevent Celestrak 403 Forbidden blocks
            headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
            res = requests.get(url, headers=headers, timeout=10)
            text = res.text.strip()
            if "GP data has not updated" in text or "<html>" in text:
                return self.generate_fallback_tles(max_count)
            lines = text.split('\n')
            objs = []
            for i in range(0, len(lines), 3):
                if len(objs) >= max_count: break
                if i+2 < len(lines):
                    name, l1, l2 = lines[i].strip(), lines[i+1].strip(), lines[i+2].strip()
                    if l1.startswith('1 ') and l2.startswith('2 '):
                        objs.append((name, Satrec.twoline2rv(l1, l2)))
            if not objs: return self.generate_fallback_tles(max_count)
            return objs
        except Exception as e: 
            return self.generate_fallback_tles(max_count)

    def generate_fallback_tles(self, max_count):
        """Safety net in case offline during demo"""
        objects = []
        for i in range(max_count):
            name = f"Satellite-{i:02d}"
            line1 = "1 25544U 98067A   20302.39958333  .00000000  00000-0  00000-0 0  9997"
            line2 = f"2 25544  51.6443 {(i*10)%360:08.4f} 0002241 123.4567 333.1234 15.48937402000002"
            objects.append((name, Satrec.twoline2rv(line1, line2)))
        return objects

sim_state = SystemState()

def get_lat_lon(state):
    r_norm = math.sqrt(state[0]**2 + state[1]**2 + state[2]**2)
    if r_norm == 0: return 0, 0
    val = max(-1.0, min(1.0, state[2] / r_norm))
    return math.degrees(math.asin(val)), math.degrees(math.atan2(state[1], state[0]))

def calculate_fuel_burn(m_current, dv_km_s):
    return m_current * (1.0 - math.exp(-(abs(dv_km_s) * 1000.0) / (ISP * G0)))

def check_los(sat_state):
    sat_lat, sat_lon = get_lat_lon(sat_state)
    r_sat = math.sqrt(sum(x**2 for x in sat_state[0:3]))
    for gs in GROUND_STATIONS:
        lat1, lon1 = math.radians(sat_lat), math.radians(sat_lon)
        lat2, lon2 = math.radians(gs["lat"]), math.radians(gs["lon"])
        a = math.sin((lat2-lat1)/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2-lon1)/2)**2
        theta = 2 * math.asin(math.sqrt(max(0.0, min(1.0, a))))
        if theta == 0: return True
        re_adjusted = EARTH_RADIUS_KM + (gs["alt"] / 1000.0)
        try:
            if math.degrees(math.atan((math.cos(theta) - (re_adjusted / r_sat)) / math.sin(theta))) >= gs["min_el"]: return True
        except: pass
    return False

def calculate_acceleration(r):
    x, y, z = r[0], r[1], r[2]
    r2 = x**2 + y**2 + z**2
    r_norm = math.sqrt(r2)
    mu_r3 = -MU / (r_norm**3)
    j2_factor = (1.5 * J2 * MU * EARTH_RADIUS_KM**2) / (r_norm**5)
    z2_r2 = 5.0 * (z**2) / r2
    return [mu_r3 * x + j2_factor * x * (z2_r2 - 1.0), mu_r3 * y + j2_factor * y * (z2_r2 - 1.0), mu_r3 * z + j2_factor * z * (z2_r2 - 3.0)]

def rk4_step(state, dt):
    r, v = state[0:3], state[3:6]
    a1 = calculate_acceleration(r)
    r2, v2 = [r[i] + v[i]*dt/2 for i in range(3)], [v[i] + a1[i]*dt/2 for i in range(3)]
    a2 = calculate_acceleration(r2)
    r3, v3 = [r[i] + v2[i]*dt/2 for i in range(3)], [v[i] + a2[i]*dt/2 for i in range(3)]
    a3 = calculate_acceleration(r3)
    r4, v4 = [r[i] + v3[i]*dt for i in range(3)], [v[i] + a3[i]*dt for i in range(3)]
    a4 = calculate_acceleration(r4)
    return [r[i] + (dt/6.0)*(v[i] + 2*v2[i] + 2*v3[i] + v4[i]) for i in range(3)] + [v[i] + (dt/6.0)*(a1[i] + 2*a2[i] + 2*a3[i] + a4[i]) for i in range(3)]

def eci_to_rtn_matrix(r, v):
    r_norm = math.sqrt(sum(x**2 for x in r))
    R_hat = [x/r_norm for x in r]
    N = [r[1]*v[2] - r[2]*v[1], r[2]*v[0] - r[0]*v[2], r[0]*v[1] - r[1]*v[0]]
    n_norm = math.sqrt(sum(x**2 for x in N))
    if n_norm == 0: n_norm = 1 
    N_hat = [x/n_norm for x in N]
    T_hat = [N_hat[1]*R_hat[2] - N_hat[2]*R_hat[1], N_hat[2]*R_hat[0] - N_hat[0]*R_hat[2], N_hat[0]*R_hat[1] - N_hat[1]*R_hat[0]]
    return R_hat, T_hat, N_hat

def find_tca_continuous(sat_state, deb_state, horizon_sec=120):
    def distance_squared(t):
        s_fut = rk4_step(sat_state, t)
        d_fut = rk4_step(deb_state, t)
        return sum((s_fut[i] - d_fut[i])**2 for i in range(3))
    res = minimize_scalar(distance_squared, bounds=(0, horizon_sec), method='bounded')
    return math.sqrt(res.fun)

def advance_simulation(step_seconds: float):
    dt = 15.0 
    steps = max(1, int(step_seconds / dt))
    col_detected, man_exec = 0, 0
    start_t = time.perf_counter()
    
    for _ in range(steps):
        for d in sim_state.debris.values(): d["state"] = rk4_step(d["state"], dt)
        for sid, s in sim_state.satellites.items():
            if s["status"] == "GRAVEYARD": continue
            s["nominal_state"] = rk4_step(s["nominal_state"], dt)
            s["state"] = rk4_step(s["state"], dt)
            
            if s["cooldown"] > 0:
                s["cooldown"] -= dt
                if s["cooldown"] <= 0 and s["status"] == "EVADING":
                    s["status"] = "NOMINAL"
                    man_exec += 1
                    r_nom, v_nom, r_act, v_act = s["nominal_state"][0:3], s["nominal_state"][3:6], s["state"][0:3], s["state"][3:6]
                    dv_rendezvous = [((r_nom[i] - r_act[i]) / EVASION_TIME_SEC) + (v_nom[i] - v_act[i]) for i in range(3)]
                    dv_mag = math.sqrt(sum(x**2 for x in dv_rendezvous))
                    s["fuel_kg"] -= calculate_fuel_burn(s["fuel_kg"], dv_mag)
                    s["state"] = s["nominal_state"].copy()
                    if s["fuel_kg"] <= GRAVEYARD_LIMIT: s["status"] = "GRAVEYARD"
                        
        deb_pos = [d["state"][0:3] for d in sim_state.debris.values()]
        if deb_pos:
            tree = KDTree(deb_pos)
            for sid, s in sim_state.satellites.items():
                if s["status"] == "NOMINAL":
                    indices = tree.query_ball_point(s["state"][0:3], r=10.0) 
                    for idx in indices:
                        deb_state = list(sim_state.debris.values())[idx]["state"]
                        exact_min_dist = find_tca_continuous(s["state"], deb_state, horizon_sec=120)
                        
                        if exact_min_dist <= COLLISION_THRESHOLD_KM:
                            col_detected += 1
                            if check_los(s["state"]): 
                                s["status"] = "EVADING"
                                s["cooldown"] = EVASION_TIME_SEC
                                man_exec += 1
                                
                                R_hat, T_hat, N_hat = eci_to_rtn_matrix(s["state"][0:3], s["state"][3:6])
                                dv_rtn = [0.0, MAX_THRUST_KM_S, 0.0] 
                                
                                dvx = dv_rtn[0]*R_hat[0] + dv_rtn[1]*T_hat[0] + dv_rtn[2]*N_hat[0]
                                dvy = dv_rtn[0]*R_hat[1] + dv_rtn[1]*T_hat[1] + dv_rtn[2]*N_hat[1]
                                dvz = dv_rtn[0]*R_hat[2] + dv_rtn[1]*T_hat[2] + dv_rtn[2]*N_hat[2]
                                
                                s["evasion_dv"] = [dvx, dvy, dvz]
                                s["fuel_kg"] -= calculate_fuel_burn(s["fuel_kg"], MAX_THRUST_KM_S)
                                s["state"][3] += dvx; s["state"][4] += dvy; s["state"][5] += dvz
                                if s["fuel_kg"] <= GRAVEYARD_LIMIT: s["status"] = "GRAVEYARD"
                                break 
                            
    sim_state.kd_latency_ms = (time.perf_counter() - start_t) * 1000
    return col_detected, man_exec

@app.post("/api/telemetry")
async def ingest_telemetry(payload: TelemetryPayload): return {"status": "ACK"}

@app.post("/api/simulate/step")
async def api_simulate_step(payload: StepPayload): return advance_simulation(payload.step_seconds)

@app.get("/api/visualization/snapshot")
async def get_snapshot():
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "satellites": [{"id": s["id"], "lat": get_lat_lon(s["state"])[0], "lon": get_lat_lon(s["state"])[1], "fuel_kg": s["fuel_kg"], "status": s["status"], "state": s["state"]} for s in sim_state.satellites.values()],
        "debris_cloud": [[d["id"], round(get_lat_lon(d["state"])[0], 4), round(get_lat_lon(d["state"])[1], 4), round(math.sqrt(sum(x**2 for x in d["state"][0:3])) - EARTH_RADIUS_KM, 2)] for d in sim_state.debris.values()],
        "metrics": {"kd_tree_latency_ms": sim_state.kd_latency_ms}
    }

@app.get("/")
async def serve_frontend():
    if os.path.exists("index.html"): return FileResponse("index.html")
    elif os.path.exists("frontend/index.html"): return FileResponse("frontend/index.html")
    else: return HTMLResponse("<h1>Error: index.html not found!</h1>", status_code=404)

async def auto_runner():
    while True:
        advance_simulation(30.0) 
        await asyncio.sleep(1.0)
        
@app.on_event("startup")
async def startup_event(): asyncio.create_task(auto_runner())

if __name__ == "__main__": uvicorn.run(app, host="0.0.0.0", port=8000)