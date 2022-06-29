#include "Game.h"


Game::Game():
	mh(window, ps.pixPerUnit)
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 8;

	window.create(sf::VideoMode(pixWidth, pixHeight),
		"Physics",
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
	addConvexPolygon(4, w, { w / 2, h + w / 2 });
	addConvexPolygon(4, w, { w / 2, -w / 2 });
	addConvexPolygon(4, h, { w + h / 2, h / 2});
	addConvexPolygon(4, h, { - h / 2, h / 2 });
	
	int n = 15;
	int m = 15;
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			real x = w * (i + 1) / (n + 1);
			real y = h * (j + 1) / (m + 1);
			//addCircle(0.2, { x, y }, 1);
			if (0)//rand() % 2 == 0)
				addConvexPolygon(4, 0.2, { x, y }, 10);
			else
				addCircle(0.25, { x, y }, 10);
		}
	}

	//addCircle(2, pixToCoords(pixWidth * 0.5, pixHeight * 0.75));
	//addCircle(1, pixToCoords(pixWidth * 0.25, pixHeight * 0.75));
	//addCircle(1, pixToCoords(pixWidth * 0.75, pixHeight * 0.75));
	//addCircle(0.5, { 3,3 }, 2);

	for (auto& rb : rigidBodies)
	{
		rb->updateFatAABB(0.1);
		tree.insert(rb.get());
	}
}



void Game::run()
{
	real accTime = 0;
	real dt = 0;
	real fraction = 0;

	while (window.isOpen())
	{
		dt = frameTimer.restart().asSeconds() / 1;

		//std::cout << tree.root.get() << "\n";
		//std::cout << tree.rbNodeMap.size() << "\n";

		std::cout << 1 / dt << "\n";

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

			updateCollidingPairs();

			correctPositions();

			accTime -= ps.dt;
		}

		fraction = accTime / ps.dt;

		// Draw world
		for (auto& rb : rigidBodies)
		{
			rb->draw(window, fraction, false, &text);
		}

		for (auto& cp : collidingPairs)
		{
			//cp.second->draw(window, pixPerUnit, fraction, false, &text);
		}

		window.display();
	}
}

void Game::integrateVelocities()
{
	std::for_each(std::execution::par_unseq, rigidBodies.begin(), rigidBodies.end(), [&](const std::unique_ptr<RigidBody>& rb)
	{
		rb->integrateVel(ps.dt);
		rb->applyDamping(ps.dt);
	});
}

void Game::integratePositions()
{
	std::for_each(std::execution::par_unseq, rigidBodies.begin(), rigidBodies.end(), [&](const std::unique_ptr<RigidBody>& rb)
	{
		rb->integratePos(ps.dt);
	});
}

void Game::updateConstraints()
{
	// Remove any constraints that were marked for deletion
	std::erase_if(constraints, [](const auto& c) { return c->removeFlagSet(); });

	// Update cached data before correcting velocities
	for (auto& c : constraints)
	{
		// TODO: rename this function
		c->updateCache();
	}
	for (auto& cp : collidingPairs)
	{
		cp.second->prepareVelSolver();
	}
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

	// The onMove() and onRotate() functions are not called every iteration
	std::for_each(std::execution::par_unseq, rigidBodies.begin(), rigidBodies.end(), [](const std::unique_ptr<RigidBody>& rb)
	{
		rb->onMove();
		rb->onRotate();
	});
}

void Game::updateCollidingPairs()
{
	//for (auto& rb : rigidBodies)
	//{
	//	rb->updateAABB();
	//}

	//auto compare = [](const std::unique_ptr<RigidBody>& a, const std::unique_ptr<RigidBody>& b)
	//{
	//	return a->left() < b->left();
	//};

	//std::sort(rigidBodies.begin(), rigidBodies.end(), compare);
	//std::vector<RigidBody*> active;

	////using RBPair = std::pair<RigidBody*, RigidBody*>;
	////std::set<RBPair> possibleColliders;

	//int nCheck = 0;
	//for (auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it)
	//{
	//	// Check this rigid body against all others that could possibly be colliding
	//	for (auto activeIt = active.begin(); activeIt != active.end(); )
	//	{
	//		// Remove any rigid bodies that have already been passed
	//		if ((*activeIt)->right() < (*it)->left())
	//		{
	//			activeIt = active.erase(activeIt);
	//			continue;
	//		}

	//		checkCollision(it->get(), *activeIt);
	//		++nCheck;

	//		++activeIt;
	//	}

	//	// This rigid body is now a potential collider with the next
	//	active.push_back(it->get());
	//}

	/*for (auto& cp : collidingPairs)
	{
		cp.second->markForRemoval();
	}*/

	std::for_each(std::execution::par_unseq, collidingPairs.begin(), collidingPairs.end(), [](const auto& cp)
	{
		cp.second->markForRemoval();
	});

	std::for_each(std::execution::par_unseq, rigidBodies.begin(), rigidBodies.end(), [](const std::unique_ptr<RigidBody>& rb)
	{
		rb->updateAABB();
	});

	std::for_each(std::execution::seq, rigidBodies.begin(), rigidBodies.end(), [&](const std::unique_ptr<RigidBody>& rb)
	{
		tree.update(rb.get());
	});
	

	//int nCheck = 0;
	//for (auto& rb : rigidBodies)
	//{
	//	auto colliders = tree.getPossibleColliders(rb.get());

	//	/*std::for_each(std::execution::seq, colliders.begin(), colliders.end(), [&](RigidBody* c)
	//	{
	//		if (c->id < rb->id)
	//		{
	//			checkCollision(c, rb.get());
	//		}
	//	});*/

	//	for (auto& c : colliders)
	//	{
	//		++nCheck;

	//		// Avoid checking a pair twice
	//		if (c->id < rb->id) 
	//		{
	//			checkCollision(c, rb.get());
	//		}
	//	}
	//}

	//#pragma omp parallel for
	//for (auto& rb : rigidBodies)
	//{
	//	auto colliders = tree.getPossibleColliders(rb.get());
	//	for (auto& c : colliders)
	//	{

	//		// Avoid checking a pair twice
	//		if (c->id < rb->id) 
	//		{
	//			checkCollision(c, rb.get());
	//		}
	//	}
	//}

	// Note: parallel processing actually slows this down, presumably because of the
	// requirement for a mutex lock before accessing the contact constraint map

	std::for_each(std::execution::seq, rigidBodies.begin(), rigidBodies.end(), 
	[&](const std::unique_ptr<RigidBody>& rb)
	{
		auto colliders = tree.getPossibleColliders(rb.get());
		
		for (auto& c : colliders)
		{
			// Avoid checking a pair twice
			if (c->id < rb->id)
			{;
				checkCollision(c, rb.get());
			}
		}
	});


	/*int nCheck = 0;
	for (auto it1 = rigidBodies.begin(); it1 != rigidBodies.end(); ++it1)
	{
		for (auto it2 = rigidBodies.begin(); it2 < it1; ++it2)
		{
			++nCheck;
			checkCollision(it1->get(), it2->get());
		}
	}*/

	//std::cout << nCheck << "\n";

	// TODO: also for removing vertices from a simplex...
	std::erase_if(collidingPairs, [](const auto& cp) { return cp.second->removeFlagSet(); });

	//std::cout << collidingPairs.size() << "\n";
}

void Game::checkCollision(RigidBody* rb1, RigidBody* rb2)
{
	// Ensure a given pair of rigid bodies is always checked in a consistent order
	RigidBody* largerID = rb1;
	RigidBody* smallerID = rb2;

	if (smallerID->id > largerID->id)
	{
		std::swap(smallerID, largerID);
	}

	std::unique_ptr<ContactConstraint> result = smallerID->checkCollision(largerID);

	if (result)
	{
		result->init();

		idPair pair = { smallerID->id, largerID->id };

		auto it = collidingPairs.find(pair);

		if (it == collidingPairs.end())
		{
			// This pair is newly colliding
			collidingPairs.insert({ pair, std::move(result)});
		}
		else
		{
			// This pair was already colliding
			if (it->second->matches(result.get())) // TODO: match check in getImpulsesFrom()?
			{
				//std::cout << "match\n";
				result->getImpulsesFrom(it->second.get());
			}
			else
			{
				//std::cout << "no match\n";
			}
			
			
			it->second = std::move(result);
		}
	}
}

void Game::setupMouseConstraint()
{
	if (!mc)
	{
		// TODO: use AABB tree
		for (auto& rb : rigidBodies)
		{
			if (rb->pointInside(mh.coords()))
			{
				vec2 local = { 0,0 };
				real fMax = 300.f / rb->mInv;

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

void Game::addCircle(real rad, vec2 coords, real mInv)
{
	auto rb = std::make_unique<Circle>(ps, rad, mInv);
	rb->moveTo(coords);

	rigidBodies.push_back(std::move(rb));
}

vec2 Game::pixToCoords(real xPix, real yPix) const
{
	return { xPix / ps.pixPerUnit, yPix / ps.pixPerUnit };
}
