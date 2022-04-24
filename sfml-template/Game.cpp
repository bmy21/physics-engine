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

			vec2 mousePos = vec2(sf::Mouse::getPosition(window).x / pixPerUnit, sf::Mouse::getPosition(window).y / pixPerUnit);
			std::unique_ptr<DistanceConstraint> dc = std::make_unique<DistanceConstraint>();
			dc->point = mousePos;
			dc->rb = RigidBodies[0].get();
			Constraints.push_back(std::move(dc));


			for (auto& rb : RigidBodies)
			{
				rb->integrateVel(dtPhysics);
			}

			for (auto& c : Constraints)
			{
				c->warmStart();
			}

			for (int i = 0; i < velIter; ++i)
			{
				for (auto& c : Constraints)
				{
					c->correctVel();
				}
			}

			for (int i = 0; i < velIter; ++i)
			{
				for (auto& cc : ContactConstraints)
				{
					cc->correctVel();
				}
			}

			for (auto& rb : RigidBodies)
			{
				rb->integratePos(dtPhysics);
			}

			for (int i = 0; i < posIter; ++i)
			{
				for (auto& c : Constraints)
				{
					c->correctPos();
				}
			}

			// Snap first RB to mouse
			// RigidBodies[0]->moveTo(vec2(sf::Mouse::getPosition(window).x / pixPerUnit, sf::Mouse::getPosition(window).y / pixPerUnit));


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
						// *it represents the same contact constraint as *newIt
						// *newIt is not required; just keep and rebuild *it

						++(*it)->numPersist;

						// TODO: don't rebuild each time, use the calculated contact points from *newIt?
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
					// The contact handled by *it no longer exists
					it = ContactConstraints.erase(it);
				}
			}

			// Move any new contact constraints into the main vector
			ContactConstraints.insert(ContactConstraints.end(),
				std::make_move_iterator(NewContactConstraints.begin()),
				std::make_move_iterator(NewContactConstraints.end()));

			NewContactConstraints.clear();

			//std::cout << ContactConstraints.size() << " " << NewContactConstraints.size() << '\n';

			std::cout << RigidBodies[0]->position().x << ", " << RigidBodies[0]->position().y << '\n';


			for (int i = 0; i < posIter; ++i)
			{
				for (auto& cc : ContactConstraints)
				{
					cc->correctPos();
				}
			}

			Constraints.clear();


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
