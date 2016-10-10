#!/bin/bash

cd dart/build
make clean
cd ..
rm -fr build/

cd ../sferes2/
./waf clean
cd exp/
rm cvt_map_elites_hexapod
rm map_elites_hexapod
cd ..
rm modules.conf
cd modules/
rm cvt_map_elites
rm map_elites
cd ..
git checkout -- .
git clean -df
cd ..

cd hexapod_common/hexapod_controller
./waf clean
cd ..
git checkout -- .
git clean -df
cd ..

cd hexapod_simu/hexapod_dart
./waf clean
cd ..
git checkout -- .
git clean -df
cd ..

cd algorithms/map_elites
git checkout -- .
git clean -df
cd ../..

cd algorithms/cvt_map_elites
git checkout -- .
git clean -df
cd ../..

git checkout -- .
git clean -df
