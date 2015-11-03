#!/bin/bash
PS1='$ '
source ~/.bashrc
rosrun tf view_frames
evince frames.pdf&