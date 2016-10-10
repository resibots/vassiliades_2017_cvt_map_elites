#!/bin/bash
mkdir install
cd sferes2/exp
ln -s ../../experiments/map_elites_hexapod
ln -s ../../experiments/cvt_map_elites_hexapod

cd ../../sferes2/modules
ln -s ../../algorithms/map_elites
ln -s ../../algorithms/cvt_map_elites

cd ../../

echo "map_elites" >> sferes2/modules.conf
echo "cvt_map_elites" >> sferes2/modules.conf

INSTALL="$(realpath ./install)"
DART_PATH=$INSTALL/dart_path/
echo "Install directory: ${INSTALL}"
echo "Dart path: ${DART_PATH}"

# compile dart
cd dart
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$DART_PATH ..
make -j8
make install

# compile hexapod_common
cd ../../hexapod_common/hexapod_controller
./waf configure --prefix=$INSTALL
./waf
./waf install

cd ../hexapod_models
./waf configure --prefix=$INSTALL
./waf
./waf install

# compile hexapod_simu
cd ../../hexapod_simu/hexapod_dart
LINKFLAGS="-L$DART_PATH/lib -ldart -ldart-utils -ldart-utils-urdf" ./waf configure --prefix=$INSTALL --dart=$DART_PATH
./waf
./waf install

# compile sferes
cd ../../sferes2/
./waf configure --cpp11=yes
./waf

# compile map_elites_hexapod
LINKFLAGS="-L$DART_PATH/lib -ldart -ldart-utils -ldart-utils-urdf" ./waf configure --cpp11=yes --dart=$DART_PATH --exp map_elites_hexapod
./waf --exp map_elites_hexapod

# compile cvt_map_elites_hexapod
LINKFLAGS="-L$DART_PATH/lib -ldart -ldart-utils -ldart-utils-urdf" ./waf configure --cpp11=yes --dart=$DART_PATH --exp cvt_map_elites_hexapod
./waf --exp cvt_map_elites_hexapod

cd ..
