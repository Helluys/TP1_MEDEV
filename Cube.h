#pragma once

#include <iostream>
#include <ctime>
#include <cstdlib>
#include "Avion.h"
#include "Joueur.h"
#include "Ennemi.h"
#include <vector>
//#include "afficher.h"
#include <osgGA/TrackballManipulator>
// Base
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osgDB/ReadFile>

// Keyboard input
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

using namespace std;


class Cube {
private:
    Joueur* J1;
    Joueur* J2;
    Ennemi* E1;
    Ennemi* E2;

	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionAmi1;
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionAmi2;
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAlignement;
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionEnnemi1;
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionEnnemi2;
	osg::ref_ptr<osg::PositionAttitudeTransform>  patSCALE;
	void ConstructionAvion();
	int compteur;
    std::vector<Avion*> ListeAvion ;
	int n;
	osg::ref_ptr<osg::Group> root;
	osgViewer::Viewer* viewer;
	osg::Vec3f getSubCubePosition(int i,int j,int k);
	osg::ref_ptr<osg::Node> createSubCube(int i,int j,int k);
	osg::ref_ptr<osg::Group> createCube();
	void afficherAvion();
	void elimination(vector<int> ListeTouches, vector<Avion*> &ListeAvion);
	bool VerificationFin(std::vector<Avion*> &ListeAvion);
public:
	osgViewer::Viewer* getViewer() { return viewer; }
	void mainLoop();
	Cube(int _n);
	void afficherCube();
	int getSize() { return n; }

	bool fin;
};
