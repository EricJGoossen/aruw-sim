"""
HWEMU stand-in for robomaster-particle-filters, whose only build is a Jetson (aarch64) wheel.

Import-only: otto's 2026 comp branch imports it at module load (project_otto/target_tracker),
so without it otto can't start on x86. Nothing here filters. In a sim run otto never reaches
it: there are no detections to track (shims/hwemu_detector_stub.py), and with an enemy
(scripts/run.sh --enemy) shims/hwemu_truth_targets.py hands otto the enemy's true state in
the filter's place. Should anything construct or run a ParticleFilter anyway, it raises.
"""
