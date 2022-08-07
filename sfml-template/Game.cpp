#include "Game.h"


Game::Game():
	mh(window, ps)
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	window.create(sf::VideoMode(pixWidth, pixHeight),
		"Physics", //sf::Style::Fullscreen,
		sf::Style::Close,
		settings);

	//window.setVerticalSyncEnabled(vsync);
	//window.setFramerateLimit(fpsLimit);
	window.setMouseCursorVisible(true);

	if (!font.loadFromFile("Fonts/saxmono/saxmono.ttf"))
	{
		std::cerr << "Couldn't load font 'Sax Mono'";
	}

	text.setFont(font);
	text.setFillColor(sf::Color::Blue);

	real len = 0.5;
	int nsides = 12;

	//addConvexPolygon(nsides, len, pixToCoords(pixWidth * 0.5, 200), 1.f);
	//addConvexPolygon(6, 2.5, pixToCoords(pixWidth * 0.5, pixHeight * 0.75));
	//addConvexPolygon(7, 1, pixToCoords(pixWidth * 0.25, pixHeight * 0.75));
	//addConvexPolygon(7, 1, pixToCoords(pixWidth * 0.75, pixHeight * 0.75));

	real w = pixWidth / ps.pixPerUnit;
	real h = pixHeight / ps.pixPerUnit;

	real scale = 2;
	addConvexPolygon(4, w*scale, { w / 2, h + w*scale / 2 })->setAsUnremovable();
	addConvexPolygon(4, w*scale, { w / 2, -w*scale / 2 })->setAsUnremovable();
	addConvexPolygon(4, h*scale, { w + h*scale / 2, h / 2})->setAsUnremovable();
	addConvexPolygon(4, h*scale, { - h*scale / 2, h / 2 })->setAsUnremovable();


	// TODO: check memory usage by repeatedly removing and adding RBs?
	// TODO: Moment of inertia calculations

	int n = 35;
	int m = 35;
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			real x = w * (i + 1) / (n + 1);
			real y = h * (j + 1) / (m + 1);

			if (rand() % 2 == 0)
				addConvexPolygon(5, 0.15, { x, y }, 10);
			else
				addCircle(0.15, { x, y }, 10);
		}
	}

	//addCircle(2, pixToCoords(pixWidth * 0.5, pixHeight * 0.75));
	//addCircle(1, pixToCoords(pixWidth * 0.25, pixHeight * 0.75));
	//addCircle(1, pixToCoords(pixWidth * 0.75, pixHeight * 0.75));
	//addCircle(0.5, { 3,3 }, 2);
}



void Game::run()
{
	real accTime = 0;
	real dt = 0;
	real fraction = 0;

	while (window.isOpen())
	{
		dt = frameTimer.restart().asSeconds() / 1;

		//std::cout << 1 / dt << "\n";

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

				else if (event.mouseButton.button == sf::Mouse::Right)
				{
					//removeClickedRigidBody();
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

		if (sf::Mouse::isButtonPressed(sf::Mouse::Right))
		{
			removeClickedRigidBody();
		}

		while (accTime >= ps.dt)
		{
			// Step simulation forward by ps.dt seconds 

			updateCollidingPairs();
			
			prepareVelSolvers();

			warmStart(); 

			integrateVelocities();

			correctVelocities();
			
			integratePositions();

			correctPositions();

			// Handle constraint removal before rigid body removal, because a
			// constraint's destructor accesses all the rigid bodies it acts on

			handleConstraintRemoval();

			handleRigidBodyRemoval();

			accTime -= ps.dt;
		}

		fraction = accTime / ps.dt;

		// Draw world
		for (auto& rb : rigidBodies)
		{
			rb->draw(window, fraction, false, &text);
		}

		if (ps.showContactConstraints)
		{
			for (auto& cp : collidingPairs)
			{
				cp.second->draw(window, fraction, false, &text);
			}
		}

		if (ps.showAABBTree)
		{
			tree.draw(window, ps.pixPerUnit);
		}

		window.display();
	}
}

void Game::integrateVelocities()
{
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](const std::unique_ptr<RigidBody>& rb)
	{
		rb->integrateVel(ps.dt);
		rb->applyDamping(ps.dt);
	});
}

void Game::integratePositions()
{
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](const std::unique_ptr<RigidBody>& rb)
	{
		rb->integratePos(ps.dt);
	});
}

void Game::prepareVelSolvers()
{
	// Update cached data before correcting velocities
	std::for_each(std::execution::unseq, constraints.begin(), constraints.end(), [&](const std::unique_ptr<Constraint>& c)
	{
		c->prepareVelSolver();
	});

	std::for_each(std::execution::unseq, collidingPairs.begin(), collidingPairs.end(), [&](const auto& cp)
	{
		cp.second->prepareVelSolver();
	});
}

void Game::warmStart()
{
	for (auto& c : constraints)
	{
		c->warmStart();
	}

	for (auto& cp : collidingPairs)
	{
		cp.second->warmStart();
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

		for (auto& cp : collidingPairs)
		{
			cp.second->correctVel();
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
		
		for (auto& cp : collidingPairs)
		{
			cp.second->correctPos();
		}
	}

	// The onMove() function is not called every iteration
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [](const std::unique_ptr<RigidBody>& rb)
	{
		rb->onMove();
	});
}

void Game::updateCollidingPairs()
{
	// By default, remove all contact constraints
	std::for_each(std::execution::unseq, collidingPairs.begin(), collidingPairs.end(), [](const auto& cp)
	{
		cp.second->markForRemoval();
	});

	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](const std::unique_ptr<RigidBody>& rb)
	{
		rb->updateAABB();
		tree.update(rb.get());
	});

	// Note: parallel processing actually slows this down, presumably because of the
	// requirement for a mutex lock before accessing the contact constraint map

	std::for_each(std::execution::seq, rigidBodies.begin(), rigidBodies.end(), 
	[&](const std::unique_ptr<RigidBody>& rb)
	{
		// colliders will only contain rigid bodies with id less than rb's id
		auto colliders = tree.getPossibleColliders(rb.get());
		
		for (auto& c : colliders)
		{
			checkCollision(c, rb.get());
		}
	});

	// Any previously colliding pairs that are no longer in contact should be removed
	std::erase_if(collidingPairs, [](const auto& cp) { return cp.second->removeFlagSet(); });
}

void Game::checkCollision(RigidBody* rb1, RigidBody* rb2)
{
	// Assumes that the ordering of rb1 and rb2 is always consistent, e.g. rb1->id < rb2->id

	std::unique_ptr<ContactConstraint> result = rb1->checkCollision(rb2);

	if (result)
	{
		result->init();

		idPair pair = { rb1->id, rb2->id };

		auto it = collidingPairs.find(pair);

		if (it == collidingPairs.end())
		{
			// This pair is newly colliding
			collidingPairs.insert({ pair, std::move(result) });
		}
		else
		{
			// This pair was already colliding, so take the impulses for warm starting
			result->getImpulsesFrom(it->second.get());
			it->second = std::move(result);
		}
	}
}

void Game::removeClickedRigidBody()
{
	auto containers = tree.getPossibleContainers(mh.coords());

	for (auto& rb : containers)
	{
		if (rb->pointInside(mh.coords()))
		{
			rb->markForRemoval();
			break;
		}
	}
}

void Game::handleConstraintRemoval()
{
	// Remove any constraints that were marked for deletion
	//std::erase_if(constraints, [](const auto& c) { return c->removeFlagSet(); });

	for (auto it = constraints.begin(); it != constraints.end(); )
	{
		if ((*it)->removeFlagSet())
		{
			if (it->get() == mc)
			{
				// Remove the mouse constraint
				mc = nullptr;
			}

			it = constraints.erase(it);
		}
		else
		{
			++it;
		}
	}

	std::cout << constraints.size() << "\n";
}

void Game::handleRigidBodyRemoval()
{
	for (auto it = rigidBodies.begin(); it != rigidBodies.end(); )
	{
		if ((*it)->removeFlagSet())
		{
			tree.remove(it->get());
			it = rigidBodies.erase(it);
		}
		else
		{
			++it;
		}
	}
}

void Game::setupMouseConstraint()
{
	if (!mc)
	{
		auto containers = tree.getPossibleContainers(mh.coords());

		for (auto& rb : containers)
		{
			if (rb->pointInside(mh.coords()))
			{
				vec2 local = { 0,0 };
				real fMax = 300.f / rb->mInv();

				// TODO: Consider force/acceleration limit & contact breaking
				auto newMC = std::make_unique<MouseConstraint>(rb, mh, ps, local, .1f, 4.f, fMax);
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

ConvexPolygon* Game::addConvexPolygon(int nsides, real len, vec2 coords, real mInv)
{
	auto rb = std::make_unique<ConvexPolygon>(ps, nsides, len, mInv);
	rb->moveTo(coords);

	ConvexPolygon* rawPointer = rb.get();

	addToAABBTree(rawPointer);
	rigidBodies.push_back(std::move(rb));

	return rawPointer;
}

Circle* Game::addCircle(real rad, vec2 coords, real mInv)
{
	auto rb = std::make_unique<Circle>(ps, rad, mInv);
	rb->moveTo(coords);

	Circle* rawPointer = rb.get();

	addToAABBTree(rawPointer);
	rigidBodies.push_back(std::move(rb));

	return rawPointer;
}

void Game::addToAABBTree(RigidBody* rb)
{
	rb->updateFatAABB();
	tree.insert(rb);
}

vec2 Game::pixToCoords(real xPix, real yPix) const
{
	return { xPix / ps.pixPerUnit, yPix / ps.pixPerUnit };
}