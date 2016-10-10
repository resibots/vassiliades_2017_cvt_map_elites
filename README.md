# Scaling Up MAP-Elites Using Centroidal Voronoi Tesselations

#### meta-repo for code for the CVT-MAP-Elites experiments

Full reference:


## How to use it

#### How to properly clone this repo

```
git clone --recursive https://github.com/resibots/vassiliades_2016_cvt_map_elites.git
```

#### Dependencies

- [Boost]: C++ Template Libraries
- [Eigen]: Linear Algebra C++ Library
- [realpath]: `sudo apt-get install realpath`

#### How to easily compile everything

**Important:** Make sure you have installed all the dependencies of each repo. Otherwise the build will fail.

From the root of this repo run:

```
sh compile_all.sh
```

#### How to run MAP-Elites experiments

##### 6D descriptor (duty factor)
```
cd sferes2/build/exp/map_elites_hexapod/
./hexa_duty
```

##### 12D descriptor (12D subset of controller parameters)
```
cd sferes2/build/exp/map_elites_hexapod/
./hexa_controller12
```

##### 24D descriptor (24D subset of controller parameters)
```
cd sferes2/build/exp/map_elites_hexapod/
./hexa_controller24
```

#### How to run CVT-MAP-Elites experiments

##### 6D descriptor (duty factor)
```
cd sferes2/build/exp/cvt_map_elites_hexapod/
./hexa_duty ../../../exp/cvt_map_elites_hexapod/generated_points/generated_points_10000_6.dat
```

##### 12D descriptor (12D subset of controller parameters)
```
cd sferes2/build/exp/cvt_map_elites_hexapod/
./hexa_controller12 ../../../exp/cvt_map_elites_hexapod/generated_points/generated_points_10000_12.dat
```

##### 24D descriptor (24D subset of controller parameters)
```
cd sferes2/build/exp/cvt_map_elites_hexapod/
./hexa_controller24 ../../../exp/cvt_map_elites_hexapod/generated_points/generated_points_10000_24.dat
```

##### 36D descriptor (controller parameters)
```
cd sferes2/build/exp/cvt_map_elites_hexapod/
./hexa_controller ../../../exp/cvt_map_elites_hexapod/generated_points/generated_points_10000_36.dat
```

#### How to easily clean everything

From the root of this repo run:

```
sh clear_all.sh
```

## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[Boost]: http://www.boost.org
[Eigen]: http://eigen.tuxfamily.org/
[realpath]: http://manpages.ubuntu.com/manpages/jaunty/man1/realpath.1.html
