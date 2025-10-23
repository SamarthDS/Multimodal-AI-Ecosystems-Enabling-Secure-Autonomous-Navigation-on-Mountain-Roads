# 🏔️ Multimodal AI Ecosystems: Enabling Secure Autonomous Navigation on Mountain Roads

## 📘 Project Overview

This project(The Tofu-Delivery Protocol) presents a **co-simulation framework integrating [CARLA](https://carla.org/) and [SUMO](https://sumo.dlr.de/docs/index.html)** to investigate **Vehicle-to-Vehicle (V2V) communication strategies** for improving road safety in **mountainous terrains**.  

The primary research gap addressed is the **inadequacy of conventional collision avoidance systems** under challenging conditions such as:
- Sharp curves  
- Limited visibility  
- Unreliable connectivity  

These are typical of **elevated or hilly roads**, especially in **left-hand traffic regions like India**.  

The project aims to:
- Generate a **high-fidelity multimodal dataset** capturing **critical V2V interaction scenarios** — specifically **blind hairpin encounters**.  
- Facilitate the development and testing of **AI-driven V2V communication protocols** and **cooperative decision-making algorithms** to **mitigate collision risks** in mountainous environments.

---

## ⚙️ Simulation Environment Configuration

### 🧩 Software Requirements

| Component | Version | Purpose |
|------------|----------|----------|
| **CARLA Simulator** | 0.9.16 | High-fidelity 3D driving simulation |
| **SUMO** | Latest stable | Microscopic traffic simulation and routing |
| **Python** | 3.12.x | Primary scripting and bridge control |
| **CARLA Co-Simulation Bridge** | Included in CARLA 0.9.16 | Synchronization between CARLA & SUMO |

---

## 🏗️ Setup Procedure

### 1. Install CARLA
Download and extract **CARLA 0.9.16** for your operating system.

### 2. Install Python 3.12.x
Ensure Python is globally accessible (e.g., `py -3.12`).

### 3. Configure Python Virtual Environment
```bash
# Create and activate virtual environment
py -3.12 -m venv carla_py312_env
.\carla_py312_env\Scripts\activate

# Install CARLA client and dependencies
pip install "path/to/carla-0.9.16-cp312-cp312-win_amd64.whl"
pip install numpy pandas matplotlib
```
### 4. Install SUMO

1. **Download and install** the latest stable release of [SUMO](https://sumo.dlr.de/docs/Downloads.php).  
2. During setup, select the option **“Add SUMO to PATH”**.  
3. Verify the installation using:
   ```bash
   sumo --version
   ```
### 5. Prepare Map Network for Co-Simulation

Use **netconvert** to convert the CARLA map (Town04) to SUMO’s **.net.xml** format:
   ```bash
   netconvert --opendrive "path/to/Town04.xodr" --lefthand \
-o "path/to/Town04.net.xml"
   ```

### 6. Generate SUMO Route File

Navigate to the SUMO tools directory and create a compatible route file:
  ```bash
  cd YOUR_SUMO_INSTALL_DIR/tools
python randomTrips.py \
  -n "path/to/Town04.net.xml" \
  -o "path/to/Town04_generated.rou.xml" \
  -e 50 -l
 ```

### 7. Configure SUMO Scenario

Edit the **Town04.sumocfg** file to reference the correct network and route files:

   ```xml
   <input>
    <net-file value="Town04.net.xml"/>
    <route-files value="carlavtypes.rou.xml,Town04_generated.rou.xml"/>
   </input>
   ```
### 8. Modify Co-Simulation Bridge

Open **run_synchronization.py** and make the following modifications:

🧩 Comment out the block that forces CARLA into synchronous mode.

🧩 Remove any **client.load_world()** calls to avoid conflicts with manual map loading.


---

## 🚀 Execution Protocol

  ## 1. Start CARLA Server

  ```bash
  ./CarlaUE4.exe -map=Town04 -quality-level=Low -ResX=800 -ResY=600
  ```

  ## 2. Run Co-Simulation Bridge

  ```bash
  # Activate environment
  .\carla_py312_env\Scripts\activate

  # Navigate to bridge directory
  cd path/to/CARLA_0.9.16/Co-Simulation/Sumo

  # Run synchronized simulation
  python run_synchronization.py examples/Town04.sumocfg --sumo-gui
  ```

---

## ✅ Current System Status

- ✔️ Stable co-simulation between CARLA 0.9.16 and SUMO

- ✔️ Town04 environment loaded in both simulators

- ✔️ Left-hand traffic correctly simulated

- ✔️ Vehicles synchronized and visualized across both systems


---

## 🧭 Future Work

### 1. Scenario Definition

- Create custom route files (.rou.xml) for two-vehicle blind hairpin encounters.

- Identify and map relevant edge sequences from Town04.net.xml.

### 2. Protagonist Vehicle Integration

- Modify co-simulation bridge to allow CARLA-controlled protagonist vehicles.

- Attach multimodal sensors:

  - LiDAR

  - Radar

  - Camera

  - GPS

  - IMU

### 3. Dataset Generation

- Run multiple simulations with varying:

  - Traffic density

  - Environmental conditions

- Generate multimodal datasets for AI model training.

### 4. Data Post-Processing & Analysis

- Use Python (NumPy, Pandas, Matplotlib) to analyze:

  - Simulation integrity

  - Minimum inter-vehicle distance

  - Time-to-collision (TTC)

- Simulate V2V message propagation:

  - Latency

  - Range limitations

  - Packet loss
