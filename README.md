# Photon-v2 Renderer

<a href="./gallery/bathroom_(based on Salle de bain by nacimus).jpg"><img src="./gallery/bathroom_(based on Salle de bain by nacimus).jpg"></a><br />
<p align="center"> <i>Bathroom Scene Rendered with Photon-v2 (based on Salle de bain scene by nacimus)</i> </p>

## What is Photon-v2

Photon-v2 is a rendering engine made out of my personal interest in computer graphics. A rendering engine is what takes geometrical descriptions and material properties of a 3-D scene as input, and outputs a 2-D image of that scene from a specific point of view. In layman's terms, it is like taking a photograph, except the scene and the camera are both computer simulated. This project is actively being developeed and more features are expected to come.

Project Website: https://tzuchieh.github.io/

|| Master branch | Develop branch |
| ------------- | -----------------------|-----------------------|
| **CI Status**     | [![Build Status](https://travis-ci.org/TzuChieh/Photon-v2.svg?branch=master)](https://travis-ci.org/TzuChieh/Photon-v2) | [![Build Status](https://travis-ci.org/TzuChieh/Photon-v2.svg?branch=develop)](https://travis-ci.org/TzuChieh/Photon-v2) |

## Features

* **Rendering**
  * backward path tracing
  * backward path tracing with next event estimation
  * ~~light/particle tracing~~
  * ~~backward light tracing (next event estimation)~~
  * RGB and spectral rendering modes
  * AOV (normal)

* **Material**
  * Lambertian diffuse
  * microfacet-based opaque & translucent model
  * ideal reflector, transmitter and absorber
  * measured IOR
  * layered surface material

* **Geometry**
  * triangle, rectangle
  * sphere, cuboid, triangle mesh
  * simple 2-D wave, fractal

* **Light**
  * point light
  * area light (sphere, rectangle)
  * geometric light
  * HDR environment map
  * IES light profiles

* **Texture**
  * texture with ordinary file formats (.jpg, .png, etc)
  * mathematical modifiers (add, multiply)

* **Misc.**
  * Blender addon for scene creation
  * easy-to-write custom scene description language
  * simple GUI for rendering

## Build

**Run the setup script before building the project** (depending on your system, choose either "./setup.bat" or "./setup.sh"). Photon uses CMake as its main build system, and the toolkits you need are as follows:

* CMake 3.5.2+ (recommended)
* C++17 compliant compiler (necessary)
* Python 3 (recommended)

If you want to build editor, you will also need:

* JDK 1.8+ and Maven

NOTE: Please use "./build/" as build folder for now, since build scripts are more or less hard-coded to use this path, currently.

## Roadmap

* **Near Future (in 1 ~ 3 months)**
  * stabilize core engine structure
  * fully support common materials
  * basic addon for Blender
  * distributed rendering system
  * binary/modularized version of Photon-SDL

* **Mid-term (in 3 ~ 6 months)**
  * material node editor & parser
  * able to convert Cycles material for use in Photon
  * a complete Blender addon

* **Future Works (~1 year or later)**
  * documentation & website
  * render a short CG film using Photon

## Scene Description Language

The scene descripting language (SDL) designed by T.C. Chang can be stored into a text file (.p2 filename extension) and loaded by Photon-v2 for rendering. To create a scene, you can either open up a text file and start writing right away (visit "./scenes/" for references) or install the Blender addon (in "./BlenderAddon/"), which contains a .p2 file exporter and a material editor currently.

## About the Author

Development Blog: https://tzuchieh.github.io/ <br />
YouTube Channel:  https://www.youtube.com/channel/UCKdlogjqjuFsuv06wajp-2g <br />
Facebook Page:    https://www.facebook.com/tccthedeveloper/ <br />
E-mail:           r05922176@ntu.edu.tw <br />

If you use Photon in your research project, you are hightly encouraged to cite it using the following BibTeX template:

```latex.bib
@misc{Photon,
	Author = {Tzu-Chieh Chang},
	Year   = {2016--2018},
	Title  = {Photon renderer},
	Note   = {https://github.com/TzuChieh/Photon-v2}
} 
```

Feel free to send me an e-mail for any questions or ideas about my project (= <br />

Support the Project:
* BTC: 123ZDDMB38XcbPG3Q1fEH5WWHdMroYVuFm
* ETH: 0x0ea47ec84f6abb0cee20a54dedfa8f286731c1f2

[![HitCount](http://hits.dwyl.io/TzuChieh/Photon-v2.svg)](http://hits.dwyl.io/TzuChieh/Photon-v2)
