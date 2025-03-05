#!/bin/bash

rm -r ./data/exp*

python3 scripts/simulate_data.py \
    --exp 0 \
    \
    --landmark_uniform_err 0.0 \
    --pose_uniform_err 0.0 \
    --pose_theta_uniform_err 0.0 

python3 scripts/simulate_data.py \
    --exp 1 \
    \
    --landmark_uniform_err 1.0 \
    --pose_uniform_err 0.5 \
    --pose_theta_uniform_err 0.1 \
    --plot