#! /bin/bash

# Script to launch the main docker instance for the pblf110 car when the container was already created
docker start roboracer_sandbox
docker attach roboracer_sandbox
