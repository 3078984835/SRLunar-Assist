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

LunarSRL-Brace/  
├── src/                  # Core code (kinematics & control)  
├── data/                # Simulation datasets  
├── docs/                # Technical reports & figures  
├── requirements.txt     # Python dependencies  
├── run.py               # Main simulation launcher  
└── config.yaml          # SRL parameters (e.g., joint limits)  
❓ FAQ
Q: How to modify SRL leg length?
Edit config.yaml → leg_geometry.max_extension

Q: Dependency conflicts with NumPy?
Try conda install numpy=1.21 before pip install!

📧 Contribution
Submit issues with 🌕 emoji or PRs tagged with [SRL]!

