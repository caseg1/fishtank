//File: fishtank.h

class Props {
public:
  float size;
  float neighbor_radius;
  float num_neighbors;
  float mass;
  float collision;
  float centering;
  float velocity;
  float hunger;
  float damping;
  double dt;
  int length;
};

class Fish {
public:
  Eigen::Vector3d center;
  Eigen::Vector3d force;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  int neighbors;
};

class Food {
public:
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  int t;
};



