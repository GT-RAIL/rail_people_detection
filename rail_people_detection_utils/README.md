# RAIL People Detection Utils

Common utility functions and classes for person detection live here. Such utilities include:

1. [Position Mapper](#position-mapper): Maps a centroid in a camera image into a 3D position based on the point cloud corresponding to that image.

The utility functions in this package are not nodes in and of themselves; they are meant to be used as libraries to assist in person (or person feature) detection within other packages. Depending on the state of development, these utilities might have *both* C++ and Python implementations. If you find yourself in need of some functionality in this package that has not been implemented yet, please do contribute!


## Position Mapper

Maps a centroid in a camera image into a 3D position based on the point cloud corresponding to that image.

### Python

TODO

### C++

This does not exist yet


## Planned Functionality

1. A data stream filter/buffer to associate messages from high frequency topics.
    - Such an implementation exists in the internal interruptibility code
    - MSR had plans to release `\psi`, which is a framework aimed at solving exactly this problem. By the time we get to this work item, it might be released?
