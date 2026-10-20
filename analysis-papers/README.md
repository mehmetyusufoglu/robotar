# Analysis papers

This directory collects open-access papers and public technical references used for the branch clamp-shaker design. Files were downloaded only from public publisher or author repositories; no access controls were bypassed.

## Reading order

1. Start with the olive mechanical-harvesting review to understand mechanisms, productivity, tree damage and orchard constraints.
2. Read the trunk-shaker power paper for detachment force, frequency, damping and power relationships.
3. Read the branch-shaker dynamics paper for vibration transmission and pruning effects.
4. Read the classical fruit-stem vibration theory for control-model background.
5. Read the branch segmentation and orchard perception papers for the automatic targeting pipeline.

## Downloaded papers

| File | Source | Pages | Relevance |
|---|---|---:|---|
| `olive_trunk_shakers_review_2025.pdf` | [Messina, Sbaglia and Bernardi, *Mechanical Harvesting of Olive Orchards: An Overview on Trunk Shakers*, AgriEngineering 2025](https://www.mdpi.com/2624-7402/7/3/52) | 27 | Main olive-mechanization reference: shaker types, operating principles, fruit detachment, productivity, tree damage and orchard requirements. |
| `olive_trunk_shaker_power_2024.pdf` | [Ghonimy et al., *Power Requirements for Olive Mechanical Harvesting Using Trunk Shaker*, Scientific Reports 2024](https://www.nature.com/articles/s41598-024-77571-8) | 12 | Picual fruit dimensions and detachment force, damping ratio, 18 Hz field operation, power modeling and attachment-height effects. |
| `olive_tree_branch_shaker_dynamics_2023.pdf` | [Camposeo et al., *Re-shaping pruning improves the dynamic response of centuries-old olive trees to branch-shaker vibrations application*, Frontiers in Plant Science 2023](https://www.frontiersin.org/journals/plant-science/articles/10.3389/fpls.2023.1155120/full) | 9 | Branch-shaker vibration transmission, accelerometer measurements, pruning effects and dynamic response of large olive trees. |
| `vibratory_fruit_harvesting_theory.pdf` | [Cooke and Rand, *Vibratory Fruit Harvesting: A Linear Theory of Fruit-Stem Dynamics*, 1969](http://audiophile.tam.cornell.edu/randpdf/vibfrut1.pdf) | 16 | Classical linear fruit-stem model, resonance and dynamic-instability background for adaptive shaking control. |
| `olive_fruit_detection_2023.pdf` | [Aljaafreh et al., *A Real-Time Olive Fruit Detection for Harvesting Robot*, 2023](https://sciendo.com/article/10.2478/ata-2023-0017) | 12 | Olive-specific real-time detection benchmark; useful for before/after detachment estimation, not individual picking. |
| `lightweight_branch_segmentation_2026.pdf` | [LDS-Net: A Lightweight Dual-Branch Network for Slender Tree Branch Segmentation in Complex Natural Scenes, Forests 2026](https://www.mdpi.com/1999-4907/17/7/811) | 23 | Lightweight semantic segmentation for slender branches; relevant to branch masks and shake-point selection. |
| `orchard_panoptic_segmentation_2025.pdf` | [3D Hierarchical Panoptic Segmentation in Real Orchard Environments Across Different Sensors, arXiv 2025](https://arxiv.org/abs/2503.13188) | 8 | 3D orchard instance/panoptic segmentation across sensors; relevant to trunk/branch/fruit spatial understanding. |

## Papers still needed from the user or library access

Please upload these to `analysis-papers/` if you can obtain them legally:

- **Dynamic analysis of olive trees under forced vibration** — tree vibration modes and damping.
- **Pneumatic branch-shaker field studies** — branch-shaker frequency, duration and branch force.
- **Vision-based branch detection and shake-point estimation** — especially cherry/fruit-tree shaking research.
- **Adaptive impedance or admittance control for compliant fruit grasping** — force-limited clamp control.
- Relevant **ISO 18497** agricultural autonomous-machinery safety material, if available through institutional access.

## Notes

- Do not treat trunk-shaker power and force values as direct setpoints for a small arm-mounted branch shaker. Use them as sizing anchors and validate with branch-scale experiments.
- Product parameters must be frozen from a combination of papers, manufacturer data, simulation and physical tests—not from one paper alone.
- Add new trial findings to `../DEVELOPMENT_LOG.md` after reading or testing.
