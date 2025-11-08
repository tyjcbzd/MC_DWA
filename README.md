# :microphone: MC_DWA: Multi-Constraint Enhanced DWA for Robust and Smooth Local Navigation
The code for article "Multi-Constraint Enhanced DWA for Robust and Smooth Local Navigation"

The method enhances the trajectory evaluation by introducing several refined constraint metrics, such as
- dual-angle heading deviation；
- global path adherence；
- curvature variation control and jerk suppression. 

## Map with full dynamic obstacles
Here we used Python(for better visualization) to deploy the algorithm onto a map containing dynamic obstacles.
<p align="center">
  <img src="vis/mc-dwa_sim.gif" width="300" />
  <img src="vis/mc-dwa_compare.png" width="320" />
</p>
