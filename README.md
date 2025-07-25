![Roomi banner2](assets/Banner2.png)

*Autonomous AI Robot for Hotels and Homes*
Apache License | [Website](https://v0-robotics-landing-page.vercel.app/) | Docs (soon)

---

🚪 **Roomi: An Affordable Autonomous Cleaning & Housekeeping Robot** 🧼

🧹 Clean. 🛏️ Make beds. 🧽 Organize.
💰 Under \~\$900. 🛠️ Assembly in \~5 hours!

> Inspired by the vision of affordable, reliable, and scalable robotics.

> Powered by: LeRobot, gym-genesis, ManiSkill.

---
![Roomi banner](assets/Roomibanner.png)

📰 **Upcoming Releases**

* **2025.8.4**: Roomi support added to Genesis. Now train your Roomi policy in simulation with ease.
* **2025.7.31**: Roomi teleoperation and imitation learning suite released – collect demonstrations in real-time or from simulation!
* **2025.7.19**: Roomi v0.1 hardware – Full BOM, Demo, stl files.

---

🛒 **Cost Estimate** *(excludes shipping, tools, taxes)*

| US  $700   | EU  $600    | CN  $960   |


---

🚀 **Hardware Features**

* Custom tuned PID controllers based on RP2040 + AS5600 Encoder for each joint
* High-torque, long reach 5DOF robot arm
* Simple teleop from a modified SO100 Arm
* Friendly light-up OLED eyes

https://github.com/user-attachments/assets/0ee9a5cc-6e7f-4fe1-86ab-d5a4582c7d71

👣 **Steps**

1. 💵 [Buy the parts](https://docs.google.com/spreadsheets/d/e/2PACX-1vSSJx5n8vQ6axefWLVfU5Om-5jMr0KQSXPaziQnbyMNoEo5ZU6qLCnzRi-AQa0Jc8n8X-8icOWrnf3P/pubhtml) — Bill of Materials (BOM)
2. 🖨️ [3D Print the parts]
3. 🔧 [Assemble Roomi](#) — Step-by-step build guide
4. 💻 [Run the software](#) — Flash the firmware, load up the code, and ready to go!

**Requirements:**

* Python 3.8+
* Platformio (for flashing firmware)
* MuJoCo (for real or sim)
* Basic Git + pip + terminal skills

---

🎮 **Simulate & Teleoperate Roomi with MuJoCo**

You can launch and play (teleoperate) with Roomi using MuJoCo. We provide two entry-point scripts:

* **Basic teleop:**

  ```bash
  python roomi_mujoco.py
  ```

* **Advanced motor control:**

  ```bash
  python control.py
  ```

**Setup instructions:**

1. Create a conda environment:

   ```bash
   conda create -n roomi python=3.10
   conda activate roomi
   ```

2. Install dependencies and Roomi in editable mode:

   ```bash
   pip install -e .
   ```

---

🧠 **Roomi + ManiSkill**

Roomi is also supported in [ManiSkill](https://github.com/haosulab/ManiSkill).
Check the `roomi.py` file for how to register Roomi in simulation and define keyframes, sensors, and controllers.

---


📚 **Citation**
If referencing Roomi in research or media:

```bibtex
@misc{choghari2025roomi,
    author = {Choghari, Jade and Lee, Kyum},
    title = {Roomi: A Practical Autonomous Cleaning and Housekeeping Robot Built with Open Source Tools},
    howpublished = {\url{https://github.com/jadechoghari/roomi}},
    year = {2025},
    note = {Jade Choghari and Kyum Lee contributed equally to this work.}
}
```

---

⚠️ **Disclaimer**

By building or operating Roomi, you accept full responsibility for its behavior and outcomes. Use with care and never leave unattended in sensitive environments. This is a side project we think is cool and nice to open source!

---
