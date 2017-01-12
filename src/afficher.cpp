/* INCLUDES */

#include "afficher.h"

using namespace std;

int afficherAvion(vector<Avion*> escadron)
{
	/* OBJECTS CREATION */

	/* VIEWER CREATION */

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	/* ROOT CREATION */

	osg::ref_ptr<osg::Group> root (new osg::Group);

	/* AIRCRAFT PATS CREATION */

	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionAmi1 = new osg::PositionAttitudeTransform();
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionAmi2 = new osg::PositionAttitudeTransform();
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAlignement = new osg::PositionAttitudeTransform();
	patAlignement->setAttitude(osg::Quat(osg::DegreesToRadians(-60.0),osg::Vec3f(0,0,1)));
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionEnnemi1 = new osg::PositionAttitudeTransform();
	osg::ref_ptr<osg::PositionAttitudeTransform>  patAvionEnnemi2 = new osg::PositionAttitudeTransform();

	/* PAT INITIALISATION */ //corriger echelle position
	osg::Quat attitude;
	patAvionAmi1->setPosition(escadron[0]->getPosition());
	patAvionAmi2->setPosition(escadron[1]->getPosition());
	patAvionEnnemi1->setPosition(escadron[2]->getPosition());
	patAvionEnnemi2->setPosition(escadron[3]->getPosition());
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(escadron[0]->getDirection()));
	patAvionAmi1->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(escadron[1]->getDirection()));
	patAvionAmi2->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(escadron[2]->getDirection()));
	patAvionEnnemi1->setAttitude(attitude);
	attitude.makeRotate(osg::Vec3d(1,0,0),osg::Vec3d(escadron[3]->getDirection()));
	patAvionEnnemi2->setAttitude(attitude);

	patAvionEnnemi1->setScale(osg::Vec3f(3,3,3));
	patAvionEnnemi2->setScale(osg::Vec3f(3,3,3));

	/* MATERIAL CREATION & INITIALISATION */

	osg::ref_ptr<osg::Material> mat (new osg::Material);
	osg::ref_ptr<osg::Material> mat2 (new osg::Material);
	mat->setAmbient(mat->FRONT_AND_BACK,osg::Vec4f(1,1,1,1));
	mat2->setAmbient(mat2->FRONT_AND_BACK,osg::Vec4f(1,1,1,1));

	/* AIRCRAFTS IMPORTATION */

	osg::ref_ptr<osg::Node> avionAmi(osgDB::readNodeFile("Avion.3ds"));
	osg::ref_ptr<osg::Node> avionEnnemi(osgDB::readNodeFile("TIE-fighter.3ds"));

	/* STATE SETS CREATION & INITIALISATION */

	osg::ref_ptr<osg::StateSet> StateSetAmi1(patAvionAmi1->getOrCreateStateSet());
	osg::ref_ptr<osg::StateSet> StateSetAmi2(patAvionAmi2->getOrCreateStateSet());
	osg::ref_ptr<osg::StateSet> StateSetEnnemi1(patAvionEnnemi1->getOrCreateStateSet());
	osg::ref_ptr<osg::StateSet> StateSetEnnemi2(patAvionEnnemi2->getOrCreateStateSet());

	StateSetAmi1->setAttribute(mat);
	StateSetAmi2->setAttribute(mat);
	StateSetEnnemi1->setAttribute(mat2);
	StateSetEnnemi2->setAttribute(mat2);

	/* SCENE GRAPH */

	root->addChild(patAlignement.get());
	root->addChild(patAvionEnnemi1.get());
	root->addChild(patAvionEnnemi2.get());

	patAlignement->addChild(patAvionAmi1.get());
	patAlignement->addChild(patAvionAmi2.get());

	patAvionAmi1->addChild(avionAmi.get());
	patAvionAmi2->addChild(avionAmi.get());
	patAvionEnnemi1->addChild(avionEnnemi.get());
	patAvionEnnemi2->addChild(avionEnnemi.get());



	return (viewer->run());
}

