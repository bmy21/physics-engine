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


	std::unique_ptr<RigidBody> rb;
	
	
	real len = 0.4;
	int nsides = 15;
	rb = std::make_unique<ConvexPolygon>(nsides, len);

	//for (int i = 0; i < 10; ++i)
	//{
	//	rb = std::make_unique<ConvexPolygon>(6, 0.5);
	//	rb->moveTo({ 1920 / (2 * pixPerUnit), i * 1.f}); //+ i*0.02f
	//	rb->grav = 6;
	//	//rb->rotateTo(3 * pi / 180);
	//	RigidBodies.push_back(std::move(rb));
	//}
	//
	//RigidBodies.back()->mInv = RigidBodies.back()->IInv = RigidBodies.back()->grav = 0;

	//rb->grav = 10;
	rb->rotateTo(9 * pi / 180);
	rb->moveTo({ 1920 / (2 * pixPerUnit), 100 / (pixPerUnit) });
	rb->mInv = 2;
	rb->IInv = regularPolyInvMOI(rb->mInv, len, nsides);
	rb->applyDeltaVel({ -1, 0 }, 0);
	//std::cout << regularPolyInvMOI(rb->mInv, 0.6, 12) << '\n';
	RigidBodies.push_back(std::move(rb));


	rb = std::make_unique<ConvexPolygon>(6, 2.5);
	rb->moveTo({1920/(2*pixPerUnit), .75f*1080/(pixPerUnit)});
	rb->rotateTo(0 * pi / 180);
	rb->mInv = rb->IInv = 0;
	RigidBodies.push_back(std::move(rb));

	//rb = std::make_unique<ConvexPolygon>(7, 1);
	//rb->moveTo({ 1920 / (2 * pixPerUnit), .75f * 1080 / (pixPerUnit) });
	//rb->mInv = rb->IInv = 0;
	//RigidBodies.push_back(std::move(rb));

	rb = std::make_unique<ConvexPolygon>(7, 1);
	rb->moveTo({ 1920 / (4 * pixPerUnit), .75f*1080 / (pixPerUnit) });
	rb->mInv = rb->IInv = 0;
	RigidBodies.push_back(std::move(rb));

	rb = std::make_unique<ConvexPolygon>(7, 1);
	rb->moveTo({ .75f*1920 / (pixPerUnit), .75f*1080 / (pixPerUnit) });
	rb->mInv = rb->IInv = 0;
	RigidBodies.push_back(std::move(rb));
}



void Game::run()
{
	real accTime = 0;
	real dt = 0;
	real fraction = 0;

	vec2 mousePos = vec2(sf::Mouse::getPosition(window).x / pixPerUnit, sf::Mouse::getPosition(window).y / pixPerUnit);
	std::unique_ptr<SoftDistanceConstraint> dc;
	RigidBodies[0]->onMove();
	dc = std::make_unique<SoftDistanceConstraint>(RigidBodies[0].get(), mousePos, vec2(0, 0), 0.f, 100.f, 8.f, 1 / dtPhysics);
	Constraints.push_back(std::move(dc));


	while (window.isOpen())
	{
		window.clear(sf::Color::White);

		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}

		dt = frameTimer.restart().asSeconds()/1;

		// Don't try to simulate too much time 
		if (dt > dtMax)
		{
			dt = dtMax;
		}

		accTime += dt;


		

		while (accTime >= dtPhysics)
		{
			// Step simulation forward by dtPhysics seconds 
			static_cast<SoftDistanceConstraint*>(Constraints[0].get())->fixedPoint = vec2(sf::Mouse::getPosition(window).x / pixPerUnit, sf::Mouse::getPosition(window).y / pixPerUnit);
			

			


			for (auto& rb : RigidBodies)
			{
				rb->integrateVel(dtPhysics);
			}

			for (auto& c : Constraints)
			{
				c->warmStart();
			}

			for (auto& cc : ContactConstraints)
			{
				cc->updateCache();
				cc->warmStart();
			}

			// TODO: Constraints and ContactConstraints in the same vector?
			// Or at least in the same outer loop?
			for (int i = 0; i < velIter; ++i)
			{
				for (auto& c : Constraints)
				{
					//c->correctVel();
				}
			}

			for (int i = 0; i < velIter; ++i)
			{
				for (auto& c : Constraints)
				{
					c->correctVel();
				}

				for (auto& cc : ContactConstraints)
				{
					cc->correctVel();
				}

				for (auto& c : Constraints)
				{
					//c->correctVel();
				}
			}

			for (int i = 0; i < velIter; ++i)
			{
				for (auto& c : Constraints)
				{
					//c->correctVel();
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
					//c->correctPos();
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

					if (result)
					{
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
					// TODO: ensure a pair is always checked in the same order?
					// e.g. by assigning a unique id
					if ((*newIt)->matches(it->get()))
					{
						// *it represents the same contact constraint as *newIt
						// *newIt is not required; just keep and rebuild *it

						++(*it)->numPersist;
						//std::cout << (*it)->numPersist << '\n';

						(*it)->rebuildFrom(newIt->get());
						NewContactConstraints.erase(newIt);

						matched = true;

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
			//std::cout << RigidBodies[0]->position().x << ", " << RigidBodies[0]->position().y << '\n';


			for (int i = 0; i < posIter; ++i)
			{
				for (auto& cc : ContactConstraints)
				{
					cc->correctPos();
				}
			}

			//Constraints.clear();


			//std::cout << RigidBodies[0]->position().x << "\t\t" << RigidBodies[0]->position().y << '\n';
			//std::cout << RigidBodies[0]->angle()*180./pi << "\n"; 
			//std::cout << Constraints.size() << '\n';
			accTime -= dtPhysics;
		}

		fraction = accTime / dtPhysics;

		// Draw world
		for (auto& rb : RigidBodies)
		{
			rb->draw(window, pixPerUnit, 0 * fraction, 0, &text);
		}

		for (auto& cc : ContactConstraints)
		{
			//cc->draw(window, pixPerUnit, fraction, true, &text);
		}


		window.display();
	}
}
