# VTS-Mapproxy

[VTS-Mapproxy](https://github.com/melown/vts-mapproxy) is a HTTP server that
converts non-VTS resources (raster or vector) to VTS resources (surface,
boundlayer, freelayer) on the fly.

## User documentation

VTS-Mapproxy user documentation is available at
https://melown.readthedocs.io/

Authoritative resource documentation is available in separated [document](docs/resources.md)
that should be kept in sync with actual implementation.

## Download, build and install

You basically need just 2 steps to get VTS-Mapproxy installed: `git clone` the
source code from the repository and `make` it. But there are some tricky parts
of the process, so read carefully following compilation howto.

### Dependencies

#### Basic deps

Make sure, you have `cmake` and `g++` installed, before you try to compile
anything.

```
sudo apt-get update
sudo apt-get install cmake g++
```

#### VTS dependencies

Before you can run [VTS-Mapproxy](https://github.com/melown/vts-mapproxy), you
need at least [VTS-Registry](https://github.com/melown/vts-registry) downloaded
and installed in your system. Please refer to related
[README.md](https://github.com/Melown/vts-registry/blob/master/README.md) file,
about how to install and compile VTS-Registry.

#### Unpackaged deps

[VTS-Mapproxy](https://github.com/melown/vts-mapproxy) is using (among other
libraries) [OpenMesh](https://www.openmesh.org/). You have to download and
install OpenMesh library and this is, how you do it

```
git clone https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
cd OpenMesh
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

#### Adding UbuntuGIS repo

VTS-Mapproxy needs newer version of [GDAL](http://gdal.org) library, than it is
available in Ubuntu repos. Therefore you need to add [UbuntuGIS](https://wiki.ubuntu.com/UbuntuGIS)
repository to your `apt` sources:

```
sudo add-apt-repository ppa:ubuntugis/ubuntugis-unstable
sudo apt-get update
```

#### Installing packaged dependencies

Now we can download and install rest of the dependencies, which are needed to
get VTS-Mapproxy compiled:

```
sudo apt-get install \
    libboost-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-iostreams-dev\
    libboost-python-dev \
    libopencv-dev libopencv-core-dev libopencv-highgui-dev \
    libopencv-photo-dev libopencv-imgproc-dev libeigen3-dev libgdal-dev \
    libproj-dev libgeographic-dev libjsoncpp-dev \
    libprotobuf-dev protobuf-compiler libprocps-dev libmagic-dev gawk sqlite3
```

### Clone and Download

The source code can be downloaded from
[GitHub repository](https://github.com/melown/vts-mapproxy), but since there are
external dependences, you have to use `--recursive` switch while cloning the
repo.


```
git clone --recursive https://github.com/Melown/vts-mapproxy.git 
```

**NOTE:** If you did clone from GitHub previously without the `--recursive`
parameter, you should probably delete the `vts-mapproxy` directory and clone
again. The build will not work otherwise.

### Configure and build

For building VTS-Mapproxy, you just have to use ``make``

```
cd mapproxy
make -j4 # to compile in 4 threads
```

Default target location (for later `make install`) is `/usr/local/` directory.
You can set the `CMAKE_INSTALL_PREFIX` variable, to change it:

```
make set-variable VARIABLE=CMAKE_INSTALL_PREFIX=/install/prefix
```

You should see compilation progress. Depends, how many threads you allowed for
the compilation (the `-jNUMBER` parameter) it might take couple of minutes to an
hour of compilation time.

The binaries are then stored in `bin` directory. Development libraries are
stored in `lib` directory.

### Installing

You should be able to call `make install`, which will install to either defaul
location `/usr/local/` or to directory defined previously by the
`CMAKE_INSTALL_PREFIX` variable (see previous part).

When you specify the `DESTDIR` variable, resulting files will be saved in
`$DESTDIR/$CMAKE_INSTALL_PREFIX` directory (this is useful for packaging), e.g.

```
make install DESTDIR=/home/user/tmp/
```

## Install from Melown repository

Yes, we [are working on it](https://github.com/Melown/vts-mapproxy/issues/3) ...
but till we do, you have to compile VTS-Mapproxy manually. 

## Run VTS-Mapproxy server

First you need to create `mapproxy.conf` configuration file. You then can run

```
mapproxy --help
mapproxy --config mapproxy.conf
```

**NOTE:** You might need to add also `--registry` parameter, and point it to
previously compiled [VTS-Registry](https://github.com/melown/vts-registry).

Description of the configuration file can be found in our [user documentation](http://melown.readthedocs.io/en/latest/server/mapproxy.html).

The server is not intended to be exposed to the Internet as it is, instead it's
advised to hide it behind e.g. [NGINX](https://www.nginx.com/) server.

## How to contribute

Check the [CONTRIBUTING.md](CONTRIBUTING.md) file.

