#include "Game.h"


Game::Game()
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	window.create(sf::VideoMode(1920, 1080),
		"Physics",
		sf::Style::Close,
		settings);

	window.setVerticalSyncEnabled(vsync);
	window.setFramerateLimit(fpsLimit);
	window.setMouseCursorVisible(true);

	if (!font.loadFromFile("Fonts/saxmono/saxmono.ttf"))
	{
		std::cerr << "Couldn't load font 'Sax Mono'";
	}

	text.setFont(font);
	text.setFillColor(sf::Color::Blue);


	std::unique_ptr<RigidBody> rb = std::make_unique<ConvexPolygon>(6, 1.2);
	rb->grav = 5;
	//rb->rotateTo(20 * pi / 180);
	RigidBodies.push_back(std::move(rb));

	rb = std::make_unique<ConvexPolygon>(7, 1);
	rb->moveTo({1920/(2*pixPerUnit), 1080/(2*pixPerUnit)});
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

			// Snap first RB to mouse
			RigidBodies[0]->moveTo(vec2(sf::Mouse::getPosition(window).x / pixPerUnit, sf::Mouse::getPosition(window).y / pixPerUnit));


			// Check for collisions
			for (auto it1 = RigidBodies.begin(); it1 != RigidBodies.end(); ++it1)
			{
				for (auto it2 = RigidBodies.begin(); it2 != it1; ++it2)
				{
					std::unique_ptr<ContactConstraint> result = (*it1)->checkCollision(it2->get());
					//std::cout << (result ? "Overlaps" : " ") << "\n";

					if (result)
					{
						//ContactConstraints.push_back(std::move(result));
						NewContactConstraints.push_back(std::move(result));
					}
				}
			}

			
			// TODO: store a vector of ContactConstraints in each rigid body to reduce number to check?
			for (auto it = ContactConstraints.begin(); it != ContactConstraints.end(); )
			{
				bool matched = false;

				for (auto newIt = NewContactConstraints.begin(); newIt != NewContactConstraints.end(); ++newIt)
				{
					if ((*newIt)->matches(it->get()))
					{
						++(*it)->numPersist;
						(*it)->rebuild();
						matched = true;

						NewContactConstraints.erase(newIt);
						break;
					}
				}

				if (matched)
				{
					++it;
				}
				else
				{
					it = ContactConstraints.erase(it);
				}
			}


			// TODO: Use move_iterator?
			for (auto newIt = NewContactConstraints.begin(); newIt != NewContactConstraints.end(); )
			{
				ContactConstraints.push_back(std::move(*newIt));
				newIt = NewContactConstraints.erase(newIt);
			}

			/*
			for each existing contact:

				for each new contact:

					if new matches existing (mostly fast!):
						rebuild existing contact & increment persist count
						remove this new contact


				if this existing contact didn't match any new contact:
					remove this existing contact
			
			for each remaining new contact:
				move into existing contact vector
			*/
			
			//NewContactConstraints.clear();

			std::cout << ContactConstraints.size() << " " << NewContactConstraints.size() << '\n';

			accTime -= dtPhysics;
		}

		fraction = accTime / dtPhysics;

		// Draw world
		for (auto& rb : RigidBodies)
		{
			rb->draw(window, pixPerUnit, fraction, true, &text);
		}

		for (auto& cc : ContactConstraints)
		{
			cc->draw(window, pixPerUnit, fraction, true, &text);
		}

		// May not want to do this when warm starting implemented!
		// ContactConstraints.clear();

		window.display();
	}
}
