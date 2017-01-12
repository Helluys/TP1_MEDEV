
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "Avion.h"
#include "Joueur.h"
#include "Ennemi.h"
#include <vector>
#include "Cube.h"
//#include "afficher.h"

int main(){

    srand(time(NULL));
    int cube_size = 13;
    Cube world(cube_size);
    while ( !world.fin )
	{
        world.mainLoop();
    }

    return 0;
    }

