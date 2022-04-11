#include "Game.h"


Game::Game()
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	window.create(sf::VideoMode(1920, 1080),
		"Physics",
		sf::Style::Default,
		settings);

	window.setVerticalSyncEnabled(vsync);
	window.setFramerateLimit(fpsLimit);
	window.setMouseCursorVisible(true);


	std::unique_ptr<RigidBody> rb = std::make_unique<ConvexPolygon>(5, 1);
	rb->grav = 5;
	RigidBodies.push_back(std::move(rb));

	rb = std::make_unique<ConvexPolygon>(5, 1);
	RigidBodies.push_back(std::move(rb));
}



void Game::run()
{
	real accTime = 0;
	real dt = 0;
	real fraction = 0;

	while (window.isOpen())
	{
		window.clear(sf::Color::White);

		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}

		dt = frameTimer.restart().asSeconds();

		// Don't try to simulate too much time 
		if (dt > dtMax)
		{
			dt = dtMax;
		}

		accTime += dt;


		while (accTime >= dtPhysics)
		{
			// Step simulation forward by dtPhysics seconds 

			for (auto& rb : RigidBodies)
			{
				rb->integrateVel(dtPhysics);
			}

			for (auto& rb : RigidBodies)
			{
				rb->integratePos(dtPhysics);
			}


			RigidBodies[0]->pos = vec2(sf::Mouse::getPosition(window).x/pixPerUnit, sf::Mouse::getPosition(window).y/pixPerUnit);

			accTime -= dtPhysics;
		}

		fraction = accTime / dtPhysics;

		// Draw world
		for (auto& rb : RigidBodies)
		{
			rb->draw(window, pixPerUnit, fraction);
		}


		window.display();
	}
}
