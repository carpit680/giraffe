# _giraffe: a low-cost robotic manipulator_ 🦒

``` text
                                   __             ___  ___       
                            .-----|__.----.---.-.'  _.'  _.-----.
                            |  _  |  |   _|  _  |   _|   _|  -__|
                            |___  |__|__| |___._|__| |__| |_____|
                            |_____|

                               Why should fun be out of reach?
```

A [Koch v1.1](https://github.com/jess-moss/koch-v1-1) inspired even more cost-effective, ROS2-compatible, Open-Source robotic manipulator designed to lower the barriers to entry for Embodied AI.

---

## Setup Instructions

> NOTE: Configurator and the rest of the high-level software stack is presently only compatible with Python.  

### For Python

Create and setup a virtual Environment

``` bash
git clone https://github.com/carpit680/giraffe.git
cd giraffe

python3 -m venv giraffe_env
# echo "import sys; sys.path.append('$(pwd)')" >> giraffe_env/lib/$(python3 -c 'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')/site-packages/sitecustomize.py
source giraffe_env/bin/activate

pip install -r requirements.txt
pip install .
```
