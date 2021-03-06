#!/bin/bash

if [ -f shape_predictor_68_face_landmarks.dat ]; then
    printf "\033[31mLandmarks file already exists\033[0m\n"
else
    printf "\033[33mDownloading...\033[0m\n"
    wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
    printf "\033[33mExtracting...\033[0m\n"
    bzip2 -d shape_predictor_68_face_landmarks.dat.bz2
fi
