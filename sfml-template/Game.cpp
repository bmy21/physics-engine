#include "Game.h"


Game::Game():
	mh(window, pixPerUnit)
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	window.create(sf::VideoMode(pixWidth, pixHeight),
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

	real len = 0.5;
	int nsides = 12;

	addConvexPolygon(nsides, len, pixToCoords(pixWidth * 0.5, 100), 0.1f);
	addConvexPolygon(6, 2.5, pixToCoords(pixWidth * 0.5, pixHeight * 0.75));
	addConvexPolygon(7, 1, pixToCoords(pixWidth * 0.25, pixHeight * 0.75));
	addConvexPolygon(7, 1, pixToCoords(pixWidth * 0.75, pixHeight * 0.75));
}



void Game::run()
{
	real accTime = 0;
	real dt = 0;
	real fraction = 0;

	while (window.isOpen())
	{
		dt = frameTimer.restart().asSeconds() / 1;

		// Don't try to simulate too much time 
		if (dt > dtMax)
		{
			dt = dtMax;
		}

		accTime += dt;

		window.clear(sf::Color::White);

		mh.update();

		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}

			if (event.type == sf::Event::MouseButtonPressed)
			{
				if (event.mouseButton.button == sf::Mouse::Left)
				{
					setupMouseConstraint();
				}
			}

			if (event.type == sf::Event::MouseButtonReleased)
			{
				if (event.mouseButton.button == sf::Mouse::Left)
				{
					removeMouseConstraint();
				}
			}
		}
		

		while (accTime >= ps.dt)
		{
			// Step simulation forward by dtPhysics seconds 
			
			updateConstraints();

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


			accTime -= ps.dt;
		}

		fraction = accTime / ps.dt;

		// Draw world
		for (auto& rb : rigidBodies)
		{
			rb->draw(window, pixPerUnit, fraction, false, &text);
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
		rb->integrateVel(ps.dt);
		rb->applyDamping(ps.dt);
	}
}

void Game::integratePositions()
{
	for (auto& rb : rigidBodies)
	{
		rb->integratePos(ps.dt);
	}
}

void Game::updateConstraints()
{
	// Remove any constraints that were marked for deletion
	for (auto it = constraints.begin(); it != constraints.end(); )
	{
		if ((*it)->removeFlagSet())
		{
			it = constraints.erase(it);
		}
		else
		{
			++it;
		}
	}

	// Update cached data before correcting velocities
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
	for (int i = 0; i < ps.velIter; ++i)
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
	for (int i = 0; i < ps.posIter; ++i)
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

void Game::setupMouseConstraint()
{
	if (!mc)
	{
		for (auto& rb : rigidBodies)
		{
			if (rb->pointInside(mh.coords()))
			{
				vec2 local = { 0,0 };
				real fMax = 300.f / rb->mInv;

				//local = invTransform(mh->coords(), rb->position(), rb->angle());

				// TODO: Consider force/acceleration limit & contact breaking

				auto newMC = std::make_unique<MouseConstraint>(rb.get(), mh, ps, local, .1f, 4.f, fMax);
				mc = newMC.get();
				constraints.push_back(std::move(newMC));

				break;
			}
		}
	}
}

void Game::removeMouseConstraint()
{
	if (mc)
	{
		mc->markForRemoval();
		mc = nullptr;
	}
}

void Game::addConvexPolygon(int nsides, real len, vec2 coords, real mInv)
{
	auto rb = std::make_unique<ConvexPolygon>(ps, nsides, len, mInv);
	rb->moveTo(coords);

	rigidBodies.push_back(std::move(rb));
}

vec2 Game::pixToCoords(const vec2& pix) const
{
	return { pix.x / pixPerUnit, pix.y / pixPerUnit };
}
