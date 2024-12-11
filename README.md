# _giraffe: a low-cost robotic manipulator_ 🦒

``` text
                                   __             ___  ___       
                            .-----|__.----.---.-.'  _.'  _.-----.
                            |  _  |  |   _|  _  |   _|   _|  -__|
                            |___  |__|__| |___._|__| |__| |_____|
                            |_____|

                               Why should fun be out of reach?
```

A [Koch v1.1](https://github.com/jess-moss/koch-v1-1) inspired even more cost-effective, ROS2-compatible, Open-Source robotic manipulator designed to lower the barriers to entry for Embodied AI and whatever else your robotic dreams may be.



---

## Setup Instructions

> NOTE: Configurator and the rest of the high-level software stack is presently only compatible with Python.  

### Clone the [giraffe](https://github.com/carpit680/giraffe) repository

``` bash
git clone https://github.com/carpit680/giraffe.git -b moveit
cd giraffe
```

### Install dependencies

```bash
pip install -r requirements.txt
pip install .
```

### Setup permissions

```bash
sudo usermod -a -G dialout $USER
sudo newgrp dialout
```