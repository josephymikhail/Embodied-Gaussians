## Implementation Notes
For Intelligent Robot Motion Lab: Use camera_pose/get_pose.py to get camera poses using checkerboard and Intel RealSense. If necessary, change serial numbers in the script. It would also be nice for this script to be updated such that it uses ChArUco board rather than checkerboard. You can also use camera_pose/get_intrinsics/cameraCalibration.py to get intrinsics for a specific camera.

This repository provides a reference implementation of Embodied Gaussians with some differences from the paper:

- 🔷 **Rigid Bodies Only**: Currently, this implementation only supports rigid body dynamics. The shape matching functionality described in the paper is not included.
- 🔨 **Simplified Physics**: Due to the rigid body constraint, the physics simulation is more straightforward but less flexible than the full implementation described in the paper.

## Getting Started

### Installation

1. First, install [pixi](https://pixi.sh/latest/#installation)

2. Then build the dependencies:
```bash
pixi r build
```

### Running the Demo

To run the included demo:
```bash
pixi r demo
```


## Scene Building

These scripts require Realsense cameras directly connected to your device. Note: Offline image processing is not currently supported.

### 1. Ground Plane Detection
First, detect the ground plane by running:
```bash
python scripts/find_ground.py temp/ground_plane.json --extrinsics scripts/example_extrinsics.json --visualize
```
You will be prompted to segment the ground in the interface. The script will then calculate the ground points and plane parameters.

<div align="left">
    <img src="static/ground_detection_example.png" alt="Ground Detection" width="320">
</div>

### 2. Generate Ground Gaussians
Convert the detected ground plane into Gaussian representations:
```bash
python scripts/build_body_from_pointcloud.py temp/ground_body.json --extrinsics scripts/example_extrinsics.json --points scripts/example_ground_plane.npy --visualize
```

### 3. Object Generation
Generate embodied Gaussian representations of objects using multiple viewpoints (more viewpoints yield better results):

1. Run the scene building script:
```bash
python scripts/build_simple_body.py objects/tblock.json \
    --extrinsics scripts/example_extrinsics.json \
    --ground scripts/example_ground_plane.json \
    --visualize
```

2. For each camera viewpoint:
   - A segmentation GUI will appear
   - Click to select the target object
   - Press `Escape` when satisfied with the selection
   - Repeat for all viewpoints

3. The script will generate a JSON file containing both particle and Gaussian representations of your object.

<div align="left">
    <img src="static/scene_builder_segmentation.png" alt="Segmentation Window" width="640">
</div>

You can visualize the object with
```bash
python scripts/visualize_object.py scripts/example_object.json
```


## Citation

If you find this work useful, please consider citing our paper:

```bibtex
@inproceedings{
    abouchakra-embodiedgaussians,
    title={Physically Embodied Gaussian Splatting: A Realtime Correctable World Model for Robotics},
    author={Jad Abou-Chakra and Krishan Rana and Feras Dayoub and Niko Suenderhauf},
    booktitle={8th Annual Conference on Robot Learning},
    year={2024},
    url={https://openreview.net/forum?id=AEq0onGrN2}
}
```

## More Information

For videos and additional information, visit our [project page](https://embodied-gaussians.github.io/).

## Disclaimer
This software is provided as a research prototype and is not production-quality software. Please note that the code may contain missing features, bugs and errors. RAI Institute does not offer maintenance or support for this software.
=======
