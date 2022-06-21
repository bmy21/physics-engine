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

	real w = pixWidth / pixPerUnit;
	real h = pixHeight / pixPerUnit;
	addConvexPolygon(4, w, { w / 2, h + w / 2 });
	addConvexPolygon(4, w, { w / 2, -w / 2 });
	addConvexPolygon(4, h, { w + h / 2, h / 2});
	addConvexPolygon(4, h, { - h / 2, h / 2 });

	int n = 22;
	int m = 22;
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			real x = w * (i + 1) / (n + 1);
			real y = h * (j + 1) / (m + 1);
			//addCircle(0.2, { x, y }, 1);
			if (rand() % 2 == 0)
				addConvexPolygon(4, 0.3, { x, y }, 1);
			else
				addCircle(0.15, { x, y }, 1);
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
		
		//detectCollisions();
		//std::cout << tree.getPossibleColliders(rigidBodies[100].get()).size() << "\n";

		while (accTime >= ps.dt)
		{
			// Step simulation forward by dtPhysics seconds 

			
			
			
			updateConstraints();

			warmStart(); 

			integrateVelocities();

			correctVelocities();
			
			integratePositions();

			/*for (auto& rb : rigidBodies)
			{
				tree.remove(rb.get());
			}
			for (auto& rb : rigidBodies)
			{
				rb->updateAABB();
				tree.insert(rb.get());
			}*/

			updateCollidingPairs();

			correctPositions();

			//std::cout << ContactConstraints.size() << " " << NewContactConstraints.size() << '\n';
			//std::cout << RigidBodies[0]->position().x << ", " << RigidBodies[0]->position().y << '\n';
			//std::cout << magnitude(RigidBodies[0]->velocity()) << "\n";
			
			//std::cout << RigidBodies[0]->position().x << "\t\t" << RigidBodies[0]->position().y << '\n';
			//std::cout << RigidBodies[0]->angle()*180./pi << "\n"; 
			//std::cout << contactConstraints.size() << '\n';

			//std::cout << rigidBodies.back()->velocity().x << "\t" << rigidBodies.back()->velocity().y << "\t"
			//	<< rigidBodies.back()->angVel() << "\n";

			//std::cout << (rigidBodies.back()->position() - rigidBodies.back()->prevPosition()).x << "\t" 
			//	<< (rigidBodies.back()->position() - rigidBodies.back()->prevPosition()).y << "\t"
			//	<< (rigidBodies.back()->angle() - rigidBodies.back()->prevAngle()) << "\n";

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
			//cc->draw(window, pixPerUnit, fraction, false, &text);
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
		// TODO: rename this function
		c->updateCache();
	}
	for (auto& cc : contactConstraints)
	{
		cc->prepareVelSolver();
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

	// The onMove() and onRotate() functions are not called every iteration
	for (auto& rb : rigidBodies)
	{
		rb->onMove();
		rb->onRotate();
	}
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

	for (auto& rb : rigidBodies)
	{
		rb->updateAABB();
		tree.update(rb.get());
	}

	int nCheck = 0;
	for (auto& rb : rigidBodies)
	{
		auto colliders = tree.getPossibleColliders(rb.get());
		//std::cout << colliders.size() << "\n";
		for (auto& c : colliders)
		{
			++nCheck;

			if (c->id < rb->id)
			{
				checkCollision(c, rb.get());
			}
		}
	}


	std::cout << nCheck << "\n";


	// TODO: store a vector of ContactConstraints in each rigid body to reduce number to check?
	for (auto newIt = newContactConstraints.begin(); newIt != newContactConstraints.end(); ++newIt)
	{
		for (auto it = contactConstraints.begin(); it != contactConstraints.end(); ++it)
		{
			if ((*it)->matches(newIt->get()))
			{
				(*newIt)->getImpulsesFrom(it->get());
				break;
			}
		}
	}

	contactConstraints = std::move(newContactConstraints);
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
		newContactConstraints.push_back(std::move(result));
	}
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
	return { xPix / pixPerUnit, yPix / pixPerUnit };
}
