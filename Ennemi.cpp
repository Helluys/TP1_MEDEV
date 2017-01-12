#include "Ennemi.h"
#include<cmath>
#include<osg/Vec3f>


Ennemi::Ennemi() : Avion()
{
	camp=true;
}


Ennemi::Ennemi(osg::Vec3f pos, osg::Vec3f dir, int num) : Avion()
{
	position = pos;
	direction = dir;
	camp=false;
	id=num;
}

Ennemi::~Ennemi(){
}

void Ennemi::avancer(int cube_size){
    position += direction/(sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2]));
    float taille = (float)cube_size;
    if (position[0] < 0){
        position[0] += taille;
    }
    if (position[0] > taille){
        position[0] -= taille;
    }
    if (position[1] < 0){
        position[1] += taille;
    }
    if (position[1] > taille){
        position[1] -= taille;
    }
    if (position[2] < 0){
        position[2] += taille;
    }
    if (position[2] > taille){
        position[2] -= taille;
    }
}

void Ennemi::strategie(std::vector<Avion*> &v){

	//Trouver ennemi le plus proche
	//Avion *plusProche;
	//plusProche=trouverJoueurProche(v);

	//

//Verifier murs

}

/*Avion* Ennemi::trouverJoueurProche(std::vector<Avion*> &avions){
	int i,j=0;
	for(i=1;i<avions.size();i++)
	{
		if(dist(this->position,avions[i]->getPosition())<dist(this->position,avions[j]->getPosition()))
		{

		}
	}
	return 
}*/

float Ennemi::dist(osg::Vec3f a,osg::Vec3f b){
	return (std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2])));
}
