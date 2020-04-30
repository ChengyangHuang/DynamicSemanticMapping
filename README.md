# Dynamic Continuous  Semantic  Counting  Sensor  Model 

This method is originally produced for a final project in Mobile Robotics at the University of Michigan in Winter 2020. Our method extends on the earlier continuous semantic mapping technique BKISemanticMapping ([code](https://github.com/ganlumomo/BKISemanticMapping) and [paper](https://arxiv.org/pdf/1909.04631.pdf)). We extend their method to explicity consider dynamic obstacles in the scene. For more information for the supporting theory and results see our paper (LINK) and video (LINK). Our code is built on top of the BKISemanticMapping codebase.

## Getting Started

This project require [ROS](https://www.ros.org/) and [Catkin](http://wiki.ros.org/catkin)

### Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/ChengyangHuang/DynamicSemanticMapping.git
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Building using Intel C++ compiler (optional for better speed performance)
```bash
catkin_ws$ source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64
catkin_ws$ catkin_make -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

## Semantic Mapping using SemanticKITTI dataset

### Downloading Data

We evaluate our method on the [Semantic KITTI](http://semantic-kitti.org/). Any of the sequences can be evaluated using this method, but only the first 10 sequences also contain ground truth data that we can evaluate our method against.

### Running
```bash
$ roslaunch semantic_bki semantickitti_node.launch
```
You will see semantic map in RViz. It also query each ground truth point for evaluation, stored at data/semantickitti_04/evaluations.
![image](https://github.com/ChengyangHuang/DynamicSemanticMapping/raw/master/github/decay_result.png)

### Evaluation
Evaluation code is provided in semantickitti_evaluation.ipynb. You may modify the directory names to run it, or follow the guideline in [semantic-kitti-api](https://github.com/PRBonn/semantic-kitti-api) for evaluation.

## Quantitative Result

### Jaccard index for each class for our method and the baseline
|Class | Free | Car  | Road | Sidewalk | Other Ground | Building | Fence | Vegetation | Trunk | Terrain|
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|Baseline | 4.453e-02 | 1.292e-02 | 3.130e-03 | 0 | 8.284e-03 | 7.283e-06 | 7.311e-05 | 8.183e-04 | 4.461e-05 | 6.317e-03 |
|Ours | 4.453e-02 | 3.126e-05 | 3.144e-03 | 5.238e-06 | 8.483e-03 | 7.229e-06 | 7.310e-05 | 9.319e-04  | 4.404e-05 | 6.317e-03|

### Time comparison between our method and the baseline
| Ours | Baseline  |  Change |
|Avg. time(s) | 13.84 | 13.50 | 2.52 |
|Total time(s) | 692 | 672 | 2.98 |

## Visual result
