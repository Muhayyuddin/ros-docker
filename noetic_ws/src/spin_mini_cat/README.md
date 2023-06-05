# README

In order to compile this package you need download in your workspace also the [spin_base](https://bitbucket.org/spinitalia/spin_base_pkg/src/dev/) package cloning branch `dev` as:

````{bash}
git clone -b dev https://MarMenchetti@bitbucket.org/spinitalia/spin_base_pkg.git
````

## Notes

- As a safety feature, if no valid control input is received on both topics (`rotation` and `thrust`) for more than 2 seconds, the actuation will be put to zero.
    - **THIS FEATURE IS IMPLEMENTED ON `ROS`, NOT ON THE LOW LEVEL BOARDS**
