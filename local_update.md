 Local Update:

- use previous laser scans to update count for each node
- as we process a scan, store a node's location if it is dynamic
    - passed to the laser scan processing step

- decay term will aid in correcting the overconfidence at each time step
    - otherwise this approach may yeild worse results than the baseline

Velocity vs. No velocity
- velocity requires assignment between previous frame and current frame
    - could either be done at a pixel-level or blob-level
        - for pixel, could potentially do point cloud alignment
        - for blob, need to cluster pixels in some way and then do assignment between previous blob and next blob
- no velocity could just be covariance about the previous location of the dynamic pixels
    - can see this as simple case of velocity approach, the next predicted location is the same as the previous
    - use a sparse kernel but with sufficient radius
        - covariance should reflect that motion in z is uncommon


Tasks:
- figure out maximum velocity in the dataset (ask Chengyang)
- store previous dynamic pixels
- examine how the probabilistic model changes when introducing dependence on time
    - how does this inform the ybars?
- update counts for node using previous dynamic pixel locations
- finding or writing IoU metric function


Classes for Semantic Kitti Experiment: 
- Car	
- Bicycle	
- Motorcycle	
- Truck	
- Other Vehicle	
- Person	
- Bicyclist	
- Motorcyclist	
- Road	
- Parking	
- Sidewalk	
- Other Ground	
- Building	
- Fence	
- Vegetation	
- Trunk	
- Terrain	
- Pole	
- Traffic Sign
- Free

Classes for Kitti Experiment
