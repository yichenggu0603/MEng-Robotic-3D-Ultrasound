# Experimental Data Index

The directory contains the 19 raw, unfiltered CSV logs used in the final report.
They were recorded at approximately 100 Hz by controller version v5.1 on 30 July
2026.

| Condition | Included trial IDs | Purpose |
|---|---|---|
| Pose 1 | 134806, 134825, 134846 | No-contact baseline |
| Pose 2 | 135505, 135614, 135933, 150749, 151143, 151247, 151323 | Stationary-contact and transient assessment |
| Pose 3 | 140308, 140336, 140402 | Manual XY movement without intended surface contact |
| Pose 4 | 140545, 140700 | Combined guidance and soft-surface contact |
| Directional guidance | 141256 back, 141338 back, 141317 forward, 153653 forward | Interaction-dependent coupling during repeated manual sweeps |

CSV filenames retain their original timestamps and descriptive labels. See Sections 5.8 and 6.1 and Appendix A of the final report for the experimental procedure, selected Pose 2 windows, analysis definitions, and limitations.

The `contact_force` field is derived from the FR3 external-wrench estimate after
startup bias subtraction. It is not a measurement from an independent force
sensor and should not be interpreted as ground-truth probe contact force.
