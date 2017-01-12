#pragma once

#include "Avion.h"
#include <vector>

class Joueur : public Avion{

public:
	Joueur();
	Joueur(osg::Vec3f pos, osg::Vec3f dir, int num);
	~Joueur();

	void avancer(int cube_size);
	void strategie(std::vector<Avion*> &v);
};
