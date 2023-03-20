# AMS-DRL-for-Pursuit-Evasion
This is the repository for the paper entitled "AMS-DRL: Learning Multi-Pursuit Evasion for Safe Targeted Navigation of Drones".

## Abstract
Safe navigation of drones in the presence of adversarial physical attacks from multiple pursuers is a challenging task. This paper proposes a novel approach, asynchronous multi-stage deep reinforcement learning (AMS-DRL), to train an adversarial neural network that can learn from the actions of multiple pursuers and adapt quickly to their behavior, enabling the drone to avoid attacks and reach its target. Our approach guarantees convergence by ensuring Nash Equilibrium among agents from the game-theory analysis. We evaluate our method in extensive simulations and show that it outperforms baselines with higher navigation success rates. We also analyze how parameters such as the relative maximum speed affect navigation performance. Furthermore, we have conducted physical experiments and validated the effectiveness of the trained policies in real-time flights. A success rate heatmap is introduced to elucidate how spatial geometry influences navigation outcomes.

## Network Achitecture
<div style="text-align: center">
<img src="assets/NN.png" style="width:60%; height:60%" >
</div>

## Physical Experiment Demo
<div style="text-align: center">
<img src="assets/demo.gif" style="width:200%; height:200%"  >
</div>
