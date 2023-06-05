#!/bin/bash
sudo groupadd docker
sudo usermod -aG docker ${USER}

sudo chmod 666 /var/run/docker
