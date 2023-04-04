# Land Yacht

A land yacht model for Gazebo and ArduPilot.

![land_yacht_2](https://user-images.githubusercontent.com/24916364/229939335-6cab605e-ed94-4908-86e6-8886863c62e5.jpg)


## Usage

Gazebo and the plugins should be installed as per the [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo) instructions.

Update the `GZ_SIM_RESOURCE_PATH` to include these models:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/land_yacht/land_yacht_gazebo/models:\
$HOME/land_yacht/land_yacht_gazebo/worlds
```

### `land_yacht`

A land yacht model with standard sails.

#### Run Gazebo

```bash
gz sim -v4 -r land_yacht_beach.sdf
```

#### Run ArduPilot SITL

```bash
sim_vehicle.py -v Rover -f json \
--add-param-file=$HOME/land_yacht/land_yacht_gazebo/config/land_yacht.param \
--console --map \
--custom-location="51.56965364806888,-4.031611898852067,10,-135"
```

