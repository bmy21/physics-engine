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
	//rb->rotateTo(9 * pi / 180);
	rb->moveTo({ 1920 / (2 * pixPerUnit), 1 });
	rb->mInv = 1;
	rb->IInv = regularPolyInvMOI(rb->mInv, len, nsides);
	rb->angularDamp = decayConstant(1.5);

	//rb->grav = 800;
	//rb->applyDeltaVel({ -1, 0 }, 0);
	//std::cout << regularPolyInvMOI(rb->mInv, 0.6, 12) << '\n';
	rigidBodies.push_back(std::move(rb));


	rb = std::make_unique<ConvexPolygon>(6, 2.5);
	rb->moveTo({1920/(2*pixPerUnit), .75f*1080/(pixPerUnit)});
	rb->rotateTo(0 * pi / 180);
	rb->mInv = rb->IInv = 0;
	rigidBodies.push_back(std::move(rb));

	//rb = std::make_unique<ConvexPolygon>(7, 1);
	//rb->moveTo({ 1920 / (2 * pixPerUnit), .75f * 1080 / (pixPerUnit) });
	//rb->mInv = rb->IInv = 0;
	//RigidBodies.push_back(std::move(rb));

	rb = std::make_unique<ConvexPolygon>(7, 1);
	rb->moveTo({ 1920 / (4 * pixPerUnit), .75f*1080 / (pixPerUnit) });
	rb->mInv = rb->IInv = 0;
	rigidBodies.push_back(std::move(rb));

	rb = std::make_unique<ConvexPolygon>(7, 1);
	rb->moveTo({ .75f*1920 / (pixPerUnit), .75f*1080 / (pixPerUnit) });
	rb->mInv = rb->IInv = 0;
	rigidBodies.push_back(std::move(rb));
}



void Game::run()
{
	real accTime = 0;
	real dt = 0;
	real fraction = 0;

	updateMousePos();
	
	std::unique_ptr<MouseConstraint> dc;
	rigidBodies[0]->onMove();

	//mousePos = { 4.f,1.f };

	dc = std::make_unique<MouseConstraint>(rigidBodies[0].get(), mousePos, vec2{}, dtPhysics, .1f, 4.f, 500.f);
	constraints.push_back(std::move(dc));


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

			updateMousePos();

			//mousePos = { 1.f,1.f };
			static_cast<MouseConstraint*>(constraints[0].get())->fixedPoint = mousePos;
			
			

			updateConstraintCaches();

			warmStart();

			integrateVelocities();

			correctVelocities();
			
			integratePositions();

			detectCollisions();

			correctPositions();


			//std::cout << ContactConstraints.size() << " " << NewContactConstraints.size() << '\n';
			//std::cout << RigidBodies[0]->position().x << ", " << RigidBodies[0]->position().y << '\n';
			//std::cout << magnitude(RigidBodies[0]->velocity()) << "\n";
			
			//std::cout << RigidBodies[0]->position().x << "\t\t" << RigidBodies[0]->position().y << '\n';
			//std::cout << RigidBodies[0]->angle()*180./pi << "\n"; 
			//std::cout << Constraints.size() << '\n';


			accTime -= dtPhysics;
		}

		fraction = accTime / dtPhysics;

		// Draw world
		for (auto& rb : rigidBodies)
		{
			rb->draw(window, pixPerUnit, fraction, 0, &text);
		}

		for (auto& cc : contactConstraints)
		{
			//cc->draw(window, pixPerUnit, fraction, true, &text);
		}


		window.display();
	}
}

void Game::integrateVelocities()
{
	for (auto& rb : rigidBodies)
	{
		rb->integrateVel(dtPhysics);
		rb->applyDamping(dtPhysics);
	}
}

void Game::integratePositions()
{
	for (auto& rb : rigidBodies)
	{
		rb->integratePos(dtPhysics);
	}
}

void Game::updateConstraintCaches()
{
	for (auto& c : constraints)
	{
		c->updateCache();
	}
	for (auto& cc : contactConstraints)
	{
		cc->updateCache();
	}
}

void Game::warmStart()
{
	for (auto& c : constraints)
	{
		c->warmStart();
	}

	for (auto& cc : contactConstraints)
	{
		cc->warmStart();
	}
}

void Game::correctVelocities()
{
	for (int i = 0; i < velIter; ++i)
	{
		for (auto& c : constraints)
		{
			c->correctVel();
		}

		for (auto& cc : contactConstraints)
		{
			cc->correctVel();
		}
	}
}

void Game::correctPositions()
{
	for (int i = 0; i < posIter; ++i)
	{
		for (auto& c : constraints)
		{
			c->correctPos();
		}
		
		for (auto& cc : contactConstraints)
		{
			cc->correctPos();
		}
	}
}

void Game::detectCollisions()
{
	// Check for collisions
	for (auto it1 = rigidBodies.begin(); it1 != rigidBodies.end(); ++it1)
	{
		for (auto it2 = rigidBodies.begin(); it2 != it1; ++it2)
		{
			std::unique_ptr<ContactConstraint> result = (*it1)->checkCollision(it2->get());

			if (result)
			{
				newContactConstraints.push_back(std::move(result));
			}
		}
	}

	// TODO: store a vector of ContactConstraints in each rigid body to reduce number to check?
	for (auto it = contactConstraints.begin(); it != contactConstraints.end(); )
	{
		bool matched = false;

		for (auto newIt = newContactConstraints.begin(); newIt != newContactConstraints.end(); ++newIt)
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
				newContactConstraints.erase(newIt);

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
			it = contactConstraints.erase(it);
		}
	}

	// Move any new contact constraints into the main vector
	contactConstraints.insert(contactConstraints.end(),
		std::make_move_iterator(newContactConstraints.begin()),
		std::make_move_iterator(newContactConstraints.end()));

	newContactConstraints.clear();
}

void Game::updateMousePos()
{
	auto pix = sf::Mouse::getPosition(window);
	mousePos = { pix.x / pixPerUnit, pix.y / pixPerUnit };
}
