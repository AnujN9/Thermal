# Calibration

### Installation

Run the following command to build this package to calibrate the Lepton 3.1R and then to calibrate the extrinsics between the Lepton 3.1R the RealSense D435i

```
mkdir build && cd build
cmake ..
make
```
Creates 3 executables, one for the thermal camera intrinsic calibration, one for verification of that calibration and one for extrinsic calibration between the thermal and RGB camera

### Usage

To calibrate the thermal camera, first ensure there are images in the `thermal_images` directory. When running you need to pass in parameters. Then run

```
./camera_calibration 
    -r x    number of rows in the pattern
    -c x    number of columns in the pattern
    -n x    number of images
    -pat x  pattern type (chessboard or symmetric dot)
```

This saves the intrinsics to a calibration.xml file. To see the undistorted images run

```
./verify_calibration
    -n x    number of images
```

To calibrate the extrinsics, ensure there are thermal images in the `thermal_images` directory and color images in the `color_images` directory. When running you need to pass in parameters. Then run

```
./extrinsic 
    -r x    number of rows in the patterns
    -c x    number of columns in the patterns
    -n x    number of image pair (thermal and color)
```
This will save the extrinsics between the thermal and rgb camera into an extrinsic.xml file.
