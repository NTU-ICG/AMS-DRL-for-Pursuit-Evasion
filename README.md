# Learning Multi-Pursuit Evasion for Safe Targeted Navigation of Drones
This is the repository for the paper entitled "Learning Multi-Pursuit Evasion for Safe Targeted Navigation of Drones".
If you are interested, we would appreciate giving us a star and using the following citation:
```
@ARTICLE{xiao_ams,
  author={Xiao, Jiaping and Feroskhan, Mir},
  journal={IEEE Transactions on Artificial Intelligence}, 
  title={Learning Multi-Pursuit Evasion for Safe Targeted Navigation of Drones}, 
  year={2024},
  volume={},
  number={},
  pages={1-14},
  doi={10.1109/TAI.2024.3366871}}
```

## Abstract
Safe navigation of drones in the presence of adversarial physical attacks from multiple pursuers is a challenging task. This paper proposes a novel approach, asynchronous multistage deep reinforcement learning (AMS-DRL), to train adversarial neural networks that can learn from the actions of multiple evolved pursuers and adapt quickly to their behavior, enabling the drone to avoid attacks and reach its target. Specifically, AMS-DRL evolves adversarial agents in a pursuit-evasion game where the pursuers and the evader are asynchronously trained in a bipartite graph way during multiple stages. Our approach guarantees convergence by ensuring Nash equilibrium among agents from the game-theory analysis. We evaluate our method in extensive simulations and show that it outperforms baselines with higher navigation success rates. We also analyze how parameters such as the relative maximum speed affect navigation performance. Furthermore, we have conducted physical experiments and validated the effectiveness of the trained policies in real-time flights. A success rate heatmap is introduced to elucidate how spatial geometry influences navigation outcomes.

## Approach
The proposed AMS-DRL is designed to evolve adversarial agents in a pursuit-evasion game from multi-stage deep reinforcement learning, where the chaser drones (pursuers) and the runner drone (evader) are asynchronously trained in a bipartite graph way during different stages. With perfect information on position observation, the control policy for the runner drone is initially trained to reach the target box (the destination of the runner drone) in the cold-start learning stage ($S_0$). During the following stages, the chaser drones and the runner drone are trained against each other with one team fixed with a pre-trained policy from the prior stage, e.g., during Stage 1 ($S_1$), the chaser drones are trained to attack the runner drone while the runner drone is driven with the control policy from $S_0$. AMS-DRL can be seen as an unsupervised learning algorithm toward open-ended evolution, which approximates the capability limits of agents consistently only if the adversarial agents reach a Nash Equilibrium (NE).

<div style="text-align: center">
<img src="assets/NN.png" width="700" >
</div>


## Physical Experiment
<div style="text-align: center">
<img src="assets/drones-chasing-drones.gif" width="700"  >
</div>
<div style="text-align: center">
<img src="assets/AMS-DRLS2.gif" width="700"  >
</div>
<p>
    <em>Evading multiple pursuers with learned policy. The runner (evader) is labeled as white, and the two chasers (pursuers) are labeled as blue. The target is a box with AprilTag.</em>
</p>

## Code and Procedure
### Requirements:
1. Ubuntu 18.04
2. ROS Melodic
3. Python 2
4. Tello driver from https://github.com/xjp99v5/tello-driver-ros 
5. ONNX

### Steps:
1. roslaunch vrpn_client_ros drone.launch
2. python multiple_drone_laucher.py
3. python swarm_test.py.
4. python landTello.py

********************************************************
### Bugs:
1. cannot install the onnx package  
sol:   export CMAKE_ARGS="-DONNX_USE_PROTOBUF_SHARED_LIBS=ON"

2. cannot run the multiple_drone_laucher.py  
sol: make sure the tello driver is activated and all drones are in the same network.
