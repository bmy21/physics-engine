#pragma once

#include "Utils.h"

using real = float;
using vec2 = sf::Vector2<real>;

class Game
{
public:
	Game();
	void run();

private:

	sf::RenderWindow window;
	sf::Clock frameTimer;

	bool vsync = true;
	int fpsLimit = 144;
	real dtPhysics = 1.0 / 100;
	real dtMax = 1.0 / 10;

};

