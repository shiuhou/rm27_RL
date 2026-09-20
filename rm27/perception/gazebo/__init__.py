"""V2 Gazebo ground-truth adapter package.

Replaces the synthetic producer with a perfect virtual camera fed by
Gazebo truth poses. Guidance still sees ONLY TargetEstimate; the truth
lives at this adapter boundary (and in the supervisor), never inside
guidance.
"""
