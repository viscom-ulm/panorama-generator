# Panorama Generator

Creates a cube panorama from multiple images.

## Getting Started

The panorama generator takes intrinsic and extrinsic calibration parameters and derives a cube panorama from the input images.

### Prerequisites
ImmersightTools
```
git clone https://github.com/viscom-ulm/immersight-tools.git
```

To calibrate a multi-camera system, checkout the multicalibration repo. It is based on the tool from Bo Li et al.
For the generation of a panorama you need a xml file, containing intrinsic and extrinsic calibration parameters, as well as a xml file containing paths to the images.

Tested with:
Visual Studio 2015 and 2017
Install OpenCV (3.2)

### Installing

Clone, Build, Compile.

```
git clone https://github.com/viscom-ulm/panorama-generator.git
cd panorama-generator
mkdir build && cd build
cmake .. -G "Visual Studio 15 2017 Win64"
```

Then open Visual Studio and build the solution.

### Usage

```
Usage: ./PanoramaGenerator.exe -[w|d|n] calibration.xml images.xml [outputdir]
w - cube face size in pixel (default 2000)
d - write debug images (default false)
n - number of cameras

./PanoramaGenerator.exe -n=36 calibration.xml images.xml panorama/
```

## Authors

* **Sebastian Hartwig** - [ImmersightTools](https://github.com/viscom-ulm/immersight-tools.git)

## Acknowledgments

* Thanks to Julian Kreiser and Sebastian Maisch for help
