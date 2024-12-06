#!/bin/bash

sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi

echo "Done."
