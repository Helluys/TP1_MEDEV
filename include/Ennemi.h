#pragma once

#include "Avion.h"

class Ennemi : public Avion{

public:
	Ennemi();
	Ennemi(osg::Vec3f pos, osg::Vec3f dir, int num);
	~Ennemi();

	void avancer(int cube_size);
	void strategie(std::vector<Avion*> &v);
	//Avion* trouverJoueurProche(std::vector<Avion*> &avions);
	float dist(osg::Vec3f a,osg::Vec3f b);
};
