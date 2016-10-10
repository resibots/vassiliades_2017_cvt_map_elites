// CVT-MAP-Elites for Hexapod Simulation
// Vassilis Vassiliades - August 2016

#include <boost/random.hpp>
#include <iostream>
// #include <mutex>

#include <sferes/gen/evo_float.hpp>
#include <sferes/gen/sampled.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/phen/parameters.hpp>
#include <sferes/run.hpp>

#include <modules/cvt_map_elites/cvt_map_elites.hpp>
#include <modules/cvt_map_elites/fit_map.hpp>
#include <modules/cvt_map_elites/stat_map.hpp>

#include <hexapod_dart/hexapod_dart_simu.hpp>

#ifdef GRAPHIC
#define NO_PARALLEL
#endif

#define NO_MPI

#ifndef NO_PARALLEL
#include <sferes/eval/parallel.hpp>
#ifndef NO_MPI
#include <sferes/eval/mpi.hpp>
#endif
#else
#include <sferes/eval/eval.hpp>
#endif

using namespace sferes;

struct Params {

  struct ea {
    SFERES_CONST size_t number_of_clusters = 10000;

#if defined(DUTY)
    SFERES_CONST size_t number_of_dimensions = 6;
#elif defined(CONTROLLER)
    SFERES_CONST size_t number_of_dimensions = 36;
#elif defined(CONTROLLER12)
    // 1 value (phase) per joint
    SFERES_CONST size_t number_of_dimensions = 12;
#elif defined(CONTROLLER24)
    // 2 values (amplitude and phase) per joint
    SFERES_CONST size_t number_of_dimensions = 24;
#else
    SFERES_CONST size_t number_of_dimensions = 1;
#endif

    typedef boost::array<double, number_of_dimensions> point_t;
    static std::vector<point_t> centroids;
  };

  struct sampled {
    SFERES_ARRAY(float, values, 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35,
                 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85,
                 0.90, 0.95, 1);
    SFERES_CONST float mutation_rate = 0.05f;
    SFERES_CONST float cross_rate = 0.0f;
    SFERES_CONST bool ordered = false;
  };
  struct pop {
    // number of initial random points
    SFERES_CONST size_t init_size = 10000;
    SFERES_CONST unsigned size = 100;
    SFERES_CONST unsigned nb_gen = 75001;
    SFERES_CONST int dump_period = 75000;
  };
  struct parameters {
    SFERES_CONST float min = 0.0f;
    SFERES_CONST float max = 1.0f;
  };
};

typedef Params::ea::point_t point_t;

namespace global {
std::shared_ptr<hexapod_dart::Hexapod> global_robot;
std::vector<int> brokenLegs;
};

// define the centroids
std::vector<point_t> Params::ea::centroids;

std::vector<point_t> load_centroids(const std::string &centroids_filename) {
  std::vector<point_t> centroids;

  std::ifstream fin(centroids_filename.c_str());

  if (!fin) {
    std::cerr << "ERROR: Could not load the centroids." << std::endl;
    exit(1);
  }

  std::vector<std::string> lines;

  std::string line;
  while (std::getline(fin, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }

  fin.close();

  if (lines.size() != Params::ea::number_of_clusters) {
    std::cerr << "ERROR: The number of clusters "
              << Params::ea::number_of_clusters
              << " is not equal to the number of loaded elements "
              << lines.size() << ".\n";
    exit(1);
  }

  for (size_t i = 0; i < lines.size(); ++i) {
    std::vector<std::string> cols;

    std::string temp;
    std::istringstream stringStream;
    stringStream.str(lines[i]);

    while (stringStream >> temp)
      cols.push_back(temp);

    if (cols.size() != Params::ea::number_of_dimensions) {
      std::cerr << "ERROR: The number of dimensions "
                << Params::ea::number_of_dimensions
                << " is not equal to the dimensionality (" << cols.size()
                << ") of loaded element with index " << i << ".\n";
      exit(1);
    }

    point_t p;
    for (size_t j = 0; j < cols.size(); ++j)
      p[j] = atof(cols[j].c_str());

    centroids.push_back(p);
  }

  std::cout << "\nLoaded " << centroids.size() << " centroids.\n";

  return centroids;
}

void init_simu(std::string robot_file,
               std::vector<int> broken_legs = std::vector<int>()) {
  global::global_robot =
      std::make_shared<hexapod_dart::Hexapod>(robot_file, broken_legs);
}

FIT_MAP(FitAdapt) {
public:
  template <typename Indiv> void eval(Indiv & indiv) {
    this->_dead = false;
    _eval(indiv);
  }

  template <class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    dbg::trace trace("fit", DBG_HERE);

    ar &boost::serialization::make_nvp("_value", this->_value);
    ar &boost::serialization::make_nvp("_objs", this->_objs);
  }

  bool dead() { return _dead; }
  std::vector<double> ctrl() { return _ctrl; }

protected:
  bool _dead;
  std::vector<double> _ctrl;

  template <typename Indiv> void _eval(Indiv & indiv) {
    // copy of controler's parameters
    _ctrl.clear();
    for (size_t i = 0; i < 36; i++)
      _ctrl.push_back(indiv.data(i));

    // launching the simulation
    auto robot = global::global_robot->clone();

    using desc_t = boost::fusion::vector<hexapod_dart::descriptors::DutyCycle>;
    using safe_t =
        boost::fusion::vector<hexapod_dart::safety_measures::BodyColliding,
                              hexapod_dart::safety_measures::MaxHeight,
                              hexapod_dart::safety_measures::TurnOver>;

    hexapod_dart::HexapodDARTSimu<hexapod_dart::safety<safe_t>,
                                  hexapod_dart::desc<desc_t>>
        simu(_ctrl, robot);

    simu.run(5);

    this->_value = simu.covered_distance();

    std::vector<float> desc(Params::ea::number_of_dimensions, 0.0f);

    if (this->_value < -1000) {
      this->_dead = true;
      this->_value = -1000;
    } else {

#if defined(DUTY)
      std::vector<double> v;
      simu.get_descriptor<hexapod_dart::descriptors::DutyCycle>(v);

      desc[0] = v[0];
      desc[1] = v[1];
      desc[2] = v[2];
      desc[3] = v[3];
      desc[4] = v[4];
      desc[5] = v[5];

#elif defined(CONTROLLER)
      for (size_t i = 0; i < Params::ea::number_of_dimensions; ++i)
        desc[i] = (float)_ctrl[i];
#elif defined(CONTROLLER12)
      size_t j = 0; // descriptor index
      for (size_t i = 0; i < 36; i += 3, ++j)
        desc[j] = (float)_ctrl[i];
      assert(j == 12);
#elif defined(CONTROLLER24)
      size_t j = 0;
      for (size_t i = 0; i < 36; ++i) {
        if (i % 3 == 2)
          continue;
        desc[j] = (float)_ctrl[i];
        ++j;
      }
      assert(j == 24);
#endif
    }

    this->set_desc(desc);
  }
};

int main(int argc, char **argv) {

#if !(defined(DUTY) || defined(CONTROLLER) || defined(CONTROLLER12) ||         \
      defined(CONTROLLER24))
  std::cerr << "\nPlease use the other executables\n";
  exit(-1);
#endif

  if (argc == 1) {
    std::cerr << "\nError: please provide the centroids filename.\n";
    exit(1);
  }

  if (argc != 2) {
    std::cerr << "\nError: the number of arguments must be 1.\n";
    exit(1);
  }

  Params::ea::centroids = load_centroids(argv[1]);

  // hide errors/warnings from DART
  std::cerr.setstate(std::ios_base::failbit);

#ifndef NO_PARALLEL
#ifndef NO_MPI
  typedef eval::Mpi<Params> eval_t;
#else
  typedef eval::Parallel<Params> eval_t;
#endif
#else
  typedef eval::Eval<Params> eval_t;
#endif

  typedef gen::Sampled<36, Params> gen_t;
  typedef FitAdapt<Params> fit_t;
  typedef phen::Parameters<gen_t, fit_t, Params> phen_t;

  typedef boost::fusion::vector<sferes::stat::Map<phen_t, Params>> stat_t;
  typedef modif::Dummy<> modifier_t;
  typedef ea::CVTMapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;

  ea_t ea;

  std::cout << "init SIMU" << std::endl;

  const char *env_p = std::getenv("RESIBOTS_DIR");

  if (env_p) // if the environment variable exists
    init_simu(std::string(env_p) + "/share/hexapod_models/URDF/pexod.urdf",
              global::brokenLegs);
  else // if it does not exist, we might be running this on the cluster
    init_simu(
        "/nfs/hal01/vvassili/ResiBots/share/hexapod_models/URDF/pexod.urdf",
        global::brokenLegs);

  std::cout << "debut run" << std::endl;

  run_ea(argc, argv, ea);
  std::cout << "fin run" << std::endl;

  std::cout << "fin" << std::endl;
  return 0;
}
