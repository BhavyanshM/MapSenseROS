#!/bin/bash
export {CL_,COMPUTE_}PROFILE=1
export COMPUTE_PROFILE_CONFIG=nvvp.cfg
python depth_listener.py
