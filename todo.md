
- add enums (example visualization/RenderingProperties)
- generator flag to keep only some point types to speed up building
- add points structs (pcl::PointXYZ) classes
- add isolated functions (meaning, not methods) (ex: computeCentroid in centroid.h)
- some modules need as much as 10 GB of ram to compile (visualization?)
    try to split them or mitigate somehow

### Unsupported for now
- Specialized templated classes
- Class operators
- PCL Exceptions
- Functions taking a function pointer or a boost::function as argument
- opennurbs
- Others: see section "what to skip" in config.py