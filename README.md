# 🦿 LunarSRL-Brace | Supernumerary-Robotic-Leg System  
*"Augmenting astronaut stability on the Moon through adaptive SRL kinematics"* 🌖🚀  

---

## 🚀 Quick Start  

### 1. Create Conda Environment  
```bash  
conda create -n srl_exo python=3.12.9 -y  
conda activate srl_exo
```

### 2. Install Dependencies

```bash 
pip install -r requirements.txt
```
### 3. Run Simulation

```bash
python run.py --config config.yaml
```

✨ Key Features
🌗 Lunar Gravity Adaptation

🔺 SRL-based Quadrilateral Support Control

🤖 Joint Trajectory Optimization

📈 Real-time Stability Visualization

📂 Project Structure

```
LunarSRL-Brace/
├── __pycache__/             # Python cache files
├── resource/                # Resource folder containing main code
├── constants.py         # SRL parameter definitions (e.g., joint limits)
├── main.py              # Main simulation launcher
├── motion.py            # Motion control-related code
├── README.md                # Project documentation
├── requirements.txt         # Python dependencies
├── simulation_api.py        # Simulation API interaction code
├── test006.ttt              # Test dataset 006
├── test007.ttt              # Test dataset 007
├── test008.ttt              # Test dataset 008
├── test009.ttt              # Test dataset 009
└── visualization.py         # Visualization processing code
```

❓ FAQ

Q: How to modify SRL leg length?
Edit config.yaml → leg_geometry.max_extension

Q: Dependency conflicts with NumPy?
Try conda install numpy=1.21 before pip install!

Q: What is the version of CoppeliaSim?
CoppeliaSim EDU, version 4.9.0, 64bit.

📧 Contribution
Submit issues with 🌕 emoji or PRs tagged with [SRL]!

