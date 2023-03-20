# Land Yacht

A land yacht model for Gazebo and ArduPilot.

![land-yacht-gazebo](https://user-images.githubusercontent.com/24916364/223479135-2d6bbdd7-9642-4705-a3c2-4ca511b91d44.png)

## Usage

Gazebo and the plugins should be installed as per the [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo) instructions.

Update the `GZ_SIM_RESOURCE_PATH` to include these models:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/land_yacht/land_yacht_gazebo/models:\
$HOME/land_yacht/land_yacht_gazebo/worlds
```

### `land_yacht`

A wing sail land yacht model.

#### Run Gazebo

```bash
gz sim -v4 -r land_yacht_beach.sdf
```

#### Run ArduPilot SITL

```bash
sim_vehicle.py -v Rover -f JSON --add-param-file=$HOME/land_yacht/land_yacht_gazebo/config/land_yacht_wingsail.param --console --map
```

