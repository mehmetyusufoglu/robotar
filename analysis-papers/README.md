# Analysis papers

This directory collects open-access papers and public technical references used for the branch clamp-shaker design. Files were downloaded only from public publisher or author repositories; no access controls were bypassed.

## Reading order

For the branch clamp-shaker mechanism:

1. Start with the olive mechanical-harvesting review to understand mechanisms, productivity, tree damage and orchard constraints.
2. Read the trunk-shaker power paper for detachment force, frequency, damping and power relationships.
3. Read the branch-shaker dynamics paper for vibration transmission and pruning effects.
4. Read the classical fruit-stem vibration theory for control-model background.

For difficult-terrain mobility:

1. Read the agricultural UGV stability review for terrain, suspension and stability classifications.
2. Read the hilly orchard path-planning paper for slip-inclusive navigation on 5–25° slopes.
3. Read the orchard mowing MPC/anti-slip paper for cascaded MPC and traction control.
4. Read the orchard platform leveling and lifting-platform stability papers for active stabilization.
5. Read the two Papadopoulos/Rey papers for Force-Angle tipover margins and online prevention.
6. Read the Nav2 survey, MPPI theory and 2.5D uneven-terrain MPPI paper for the navigation stack.
7. Use the tracked-locomotion and agriculture-robotics reviews to compare wheeled, tracked and hybrid bases.

For perception:

1. Read the branch segmentation and orchard panoptic segmentation papers for branch targeting.
2. Read the olive detection paper for before/after detachment estimation, not selective picking.

## Olive harvesting and shaker papers

| File | Source | Pages | Relevance |
|---|---|---:|---|
| `olive_trunk_shakers_review_2025.pdf` | [Messina, Sbaglia and Bernardi, *Mechanical Harvesting of Olive Orchards: An Overview on Trunk Shakers*, AgriEngineering 2025](https://www.mdpi.com/2624-7402/7/3/52) | 27 | Main olive-mechanization reference: shaker types, operating principles, fruit detachment, productivity, tree damage and orchard requirements. |
| `olive_trunk_shaker_power_2024.pdf` | [Ghonimy et al., *Power Requirements for Olive Mechanical Harvesting Using Trunk Shaker*, Scientific Reports 2024](https://www.nature.com/articles/s41598-024-77571-8) | 12 | Picual fruit dimensions and detachment force, damping ratio, 18 Hz field operation, power modeling and attachment-height effects. |
| `olive_tree_branch_shaker_dynamics_2023.pdf` | [Camposeo et al., *Re-shaping pruning improves the dynamic response of centuries-old olive trees to branch-shaker vibrations application*, Frontiers in Plant Science 2023](https://www.frontiersin.org/journals/plant-science/articles/10.3389/fpls.2023.1155120/full) | 9 | Branch-shaker vibration transmission, accelerometer measurements, pruning effects and dynamic response of large olive trees. |
| `hoshyarmanesh2017.pdf` | [Hoshyarmanesh et al., *Numerical and experimental vibration analysis of olive tree for optimal mechanized harvesting efficiency and productivity*, Computers and Electronics in Agriculture 2017](https://www.sciencedirect.com/science/article/abs/pii/S0168169916304355) | 15 | Numerical and experimental olive-tree vibration analysis; key reference for the compliant-branch simulation model. |
| `vibratory_fruit_harvesting_theory.pdf` | [Cooke and Rand, *Vibratory Fruit Harvesting: A Linear Theory of Fruit-Stem Dynamics*, 1969](http://audiophile.tam.cornell.edu/randpdf/vibfrut1.pdf) | 16 | Classical linear fruit-stem model, resonance and dynamic-instability background for adaptive shaking control. |
| `olive_fruit_detection_2023.pdf` | [Aljaafreh et al., *A Real-Time Olive Fruit Detection for Harvesting Robot*, 2023](https://sciendo.com/article/10.2478/ata-2023-0017) | 12 | Olive-specific real-time detection benchmark; useful for before/after detachment estimation, not individual picking. |

## Difficult-terrain mobility and stability papers

| File | Source | Pages | Relevance |
|---|---|---:|---|
| `ag_ugv_stability_review_2020.pdf` | [Agricultural unmanned ground vehicles: a review from the stability point of view, SciELO 2020](https://www.scielo.br/j/rca/a/gfPKGgq4MvnmWbpdHhdcQfh/?lang=en) | 12 | Chassis, terrain, suspension and static/dynamic stability design map for agricultural robots. |
| `hilly_orchard_path_planning_2026.pdf` | [Global–local integrated path planning strategy for agricultural robots in complex hilly orchard environments, Frontiers 2026](https://www.frontiersin.org/journals/sustainable-food-systems/articles/10.3389/fsufs.2026.1865546/full) | 16 | Field-tested hilly-orchard navigation with slope/roughness costs, slip-inclusive kinematics and slope-dependent speed limits. |
| `orchard_mowing_mpc_antislip_2023.pdf` | [Research on Path Tracking for an Orchard Mowing Robot Based on Cascaded Model Predictive Control and Anti-Slip Drive Control, Agronomy 2023](https://www.mdpi.com/2073-4395/13/5/1395) | 25 | Orchard robot cascaded MPC path tracking and anti-slip drive control. |
| `orchard_platform_leveling_2025.pdf` | [Design and Test of Adaptive Leveling System for Orchard Operation Platform, Sensors 2025](https://www.mdpi.com/1424-8220/25/5/1319) | 22 | Four-point active leveling on uneven orchard ground using IMU and fuzzy-PID control. |
| `orchard_lifting_platform_stability_2018.pdf` | [Tilting stability analysis and experiment of the 3-DOF lifting platform for hilly orchards, IJABE 2018](https://ijabe.org/index.php/ijabe/article/view/3523) | 8 | Orchard lifting-platform stability factors and experiments on slopes. |
| `force_angle_tipover_mobile_manipulators_1996.pdf` | [Papadopoulos and Rey, *A New Measure of Tipover Stability Margin for Mobile Manipulators*, ICRA 1996](https://nereus.mech.ntua.gr/pdf_ps/ICRA96.pdf) | 6 | Canonical Force-Angle stability margin for mobile manipulators under inertial and external loads. |
| `online_tipover_prevention_1997.pdf` | [Rey and Papadopoulos, *On-line Automatic Tipover Prevention for Mobile Manipulators*, IROS 1997](https://nereus.mech.ntua.gr/pdf_ps/IROS397.pdf) | 6 | Online tipover prediction and prevention using arm/base corrective motion. |
| `tracked_locomotion_review_2022.pdf` | [Tracked Locomotion Systems for Ground Mobile Robots: A Review, Machines 2022](https://www.mdpi.com/2075-1702/10/8/648) | 17 | Tracked locomotion architectures, traction, steering and terrain trade-offs. |
| `agriculture_robotics_review_2021.pdf` | [Advances in Agriculture Robotics: A State-of-the-Art Review and Challenges Ahead, Robotics 2021](https://www.mdpi.com/2218-6581/10/2/52) | 31 | Broad agricultural robotics review including locomotion and field-operation limitations. |

## Control papers

| File | Source | Pages | Relevance |
|---|---|---:|---|
| `ji2021.pdf` | [Ji et al., *Grasping mode analysis and adaptive impedance control for apple harvesting robotic grippers*, Computers and Electronics in Agriculture 2021](https://ui.adsabs.harvard.edu/abs/2021CEAgr.18606210J/abstract) | 13 | Adaptive impedance control and damage reduction for compliant grasping; reference for force-limited branch clamping. |

## Navigation and MPPI papers

| File | Source | Pages | Relevance |
|---|---|---:|---|
| `nav2_survey_2023.pdf` | [A Survey of Modern & Capable Mobile Robotics Algorithms in ROS 2, arXiv 2023](https://arxiv.org/abs/2307.15236) | 25 | Academic reference for Nav2, MPPI, Smac planners and regulated pure pursuit. |
| `mppi_2_5d_uneven_terrain_ros2_2022.pdf` | [2.5D Mapping, Pathfinding and Path Following for Navigation of a Differential Drive Robot in Uneven Terrain, arXiv 2022](https://arxiv.org/abs/2209.07252) | 7 | ROS 2-oriented MPPI over 2.5D elevation maps with traversability costs. |
| `mppi_theory_tro_2018.pdf` | [Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving, IEEE T-RO 2018](https://arxiv.org/abs/1707.02342) | 20 | Canonical MPPI theory and aggressive off-road driving background. |

## Perception papers

| File | Source | Pages | Relevance |
|---|---|---:|---|
| `lightweight_branch_segmentation_2026.pdf` | [LDS-Net: A Lightweight Dual-Branch Network for Slender Tree Branch Segmentation in Complex Natural Scenes, Forests 2026](https://www.mdpi.com/1999-4907/17/7/811) | 23 | Lightweight semantic segmentation for slender branches; relevant to branch masks and shake-point selection. |
| `orchard_panoptic_segmentation_2025.pdf` | [3D Hierarchical Panoptic Segmentation in Real Orchard Environments Across Different Sensors, arXiv 2025](https://arxiv.org/abs/2503.13188) | 8 | 3D orchard instance/panoptic segmentation across sensors; relevant to trunk/branch/fruit spatial understanding. |

## Browser-access or library-access papers

These links are legal open repositories or publisher pages, but automated download was blocked. Download them in a browser or through institutional access if needed:

- [Denis et al., *Slip and Cornering Stiffnesses Observation for the Stability Assessment of Off-road Vehicles*, HAL 2015](https://hal.science/hal-01368574) — lateral/longitudinal load-transfer estimation on a grape harvester.
- [Dieumet et al., *Low cost active devices to estimate and prevent off-road vehicle from rollover*, HAL 2017](https://hal.inrae.fr/hal-02607282) — load-transfer indicator coupled to active slope correction.
- [Fankhauser et al., *Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization*, ETH Research Collection 2018](https://www.research-collection.ethz.ch/handle/20.500.11850/272110) — robot-centric elevation mapping with uncertainty.
- [Vidoni et al., *Evaluation and stability comparison of different vehicle configurations for robotic agricultural operations on side-slopes*, Biosystems Engineering 2015](https://doi.org/10.1016/j.biosystemseng.2014.10.003) — vehicle-configuration comparison on side slopes.
- [Bayar et al., *Improving the trajectory tracking performance of autonomous orchard vehicles using wheel slip compensation*, Biosystems Engineering 2016](https://doi.org/10.1016/j.biosystemseng.2016.02.014) — orchard wheel-slip compensation.

## Notes

- Do not treat trunk-shaker power and force values as direct setpoints for a small arm-mounted branch shaker. Use them as sizing anchors and validate with branch-scale experiments.
- Terrain coefficients and slip parameters are site-specific; papers provide methods, not parameters we can safely copy to a farm.
- Product parameters must be frozen from a combination of papers, manufacturer data, simulation and physical tests—not from one paper alone.
- Add new trial findings to `../DEVELOPMENT_LOG.md` after reading or testing.
