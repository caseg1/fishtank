//File: fishtank.cpp
#include <fstream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "fishtank.h"

#define FRAMES_PER_SEC 30

void parseInput(char *fname, Props &props, std::vector<Fish> &fish, std::vector<Food> &food) {
  unsigned int nfish, nfood;
  std::ifstream in(fname, std::ios::in);
  char junk;
  
  in >> props.size >> props.neighbor_radius >> props.num_neighbors 
	 >> props.mass >> props.collision >> props.centering 
	 >> props.velocity >> props.hunger >> props.damping 
	 >> props.dt >> props.length;
  in >> nfish;
  for (unsigned int i=0; i<nfish; i++) {
	Fish f;
	in >> junk >> f.pos[0] >> junk >> f.pos[1] >> junk >> f.pos[2] >> junk;
	in >> junk >> f.vel[0] >> junk >> f.vel[1] >> junk >> f.vel[2] >> junk;
	fish.push_back(f);
  }
  in >> nfood;
  for (unsigned int i=0; i<nfood; i++) {
	Food f;
	in >> junk >> f.pos[0] >> junk >> f.pos[1] >> junk >> f.pos[2] >> junk;
	in >> junk >> f.vel[0] >> junk >> f.vel[1] >> junk >> f.vel[2] >> junk;
	in >> f.t;
	food.push_back(f);
  }
}

void writeOutput (std::ofstream &out, const std::vector<Fish> &fish, const std::vector<Food> &food) {  
  out<<fish.size()<<std::endl;
  for (unsigned int i=0; i<fish.size(); i++) {
    out<<"["<<fish[i].pos[0]<<","<<fish[i].pos[1]<<","<<fish[i].pos[2]<<"] ";
    out<<"["<<fish[i].vel[0]<<","<<fish[i].vel[1]<<","<<fish[i].vel[2]<<"]"<<std::endl;
  }
  out<<food.size()<<std::endl;
  for (unsigned int i=0; i<food.size(); i++) {
    out<<"["<<food[i].pos[0]<<","<<food[i].pos[1]<<","<<food[i].pos[2]<<"]"<<std::endl;
  }
}

// bounding box, flip velocity if outside
void checkBounds(std::vector<Fish> &fish) {
  for (unsigned int e=0; e<fish.size(); e++) {
    if ( fish[e].pos[0] > 0.5   || fish[e].pos[0] < -0.5  ||
	 fish[e].pos[1] > 0.25  || fish[e].pos[1] < -0.25 ||
	 fish[e].pos[2] > 0.125 || fish[e].pos[2] < -0.125 )
      fish[e].vel *= -1;
  }
}

int main(int argc, char *argv[]) {
  
  char * fname = argv[1];
  std::ofstream out;
  Props props;
  std::vector<Fish> fish;
  std::vector<Food> food;
  
  parseInput(fname, props, fish, food);

  int nframes = FRAMES_PER_SEC * props.length;
  out.open("sample.out");
  out<<nframes<<std::endl;
  
  for (int f=0; f<nframes; f++) {
   
    for (unsigned int a=0; a<fish.size(); a++){
      fish[a].center << 0,0,0;
      fish[a].force << 0,0,0;
    }
    
    for (unsigned int b=0; b<fish.size(); b++){
      for (unsigned int c=0; c<fish.size(); c++){
	if (b != c) {
	  if ( (fish[b].pos - fish[c].pos).norm() < props.neighbor_radius ) {
	    fish[b].neighbors++;
	    fish[b].center += fish[c].pos;
	    
	    // check if div by zero
	    double denom = (fish[b].pos - fish[c].pos).norm();
	    if (denom > 0.00001)    
	      fish[b].force += (fish[c].vel - fish[b].vel) * props.velocity
		+ props.collision * (fish[b].pos - fish[c].pos) / pow(denom, 3.0);
	  }
	}
	fish[b].force += props.centering
	  * ((fish[b].center / fish[b].neighbors) - fish[b].pos);
      }
    } 
    for (unsigned int d=0; d<fish.size(); d++) {
      //fish[d].vel += (fish[d].force / props.mass) * props.dt;
      fish[d].vel *= props.damping;
      fish[d].pos += fish[d].vel * props.dt;
    }
    checkBounds(fish);
    writeOutput(out, fish, food);
  }
  out.close();
  
  return 0;
}
