#include "Game.h"


Game::Game():
	mh(window, ps)
{
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;

	window.create(sf::VideoMode(pixWidth, pixHeight),
		"Physics", //sf::Style::Fullscreen,
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

	real w = pixWidth / ps.pixPerUnit;
	real h = pixHeight / ps.pixPerUnit;

	real scale = 2;
	addConvexPolygon(4, w*scale, { w / 2, h + w*scale / 2 })->setAsUnremovable();
	addConvexPolygon(4, w*scale, { w / 2, -w*scale / 2 })->setAsUnremovable();
	addConvexPolygon(4, h*scale, { w + h*scale / 2, h / 2})->setAsUnremovable();
	addConvexPolygon(4, h*scale, { - h*scale / 2, h / 2 })->setAsUnremovable();

	int n = 0;
	int m = 0;
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			real x = w * (i + 1) / (n + 1);
			real y = h * (j + 1) / (m + 1);

			if (0)//rand() % 2 == 0)
			{
				//std::vector<vec2> pts = { {0, 0}, {0.7, 0}, {0.7, 0.07}, {0, 0.07} };
				//addConvexPolygon(pts, { x, y }, 10);
				addConvexPolygon(5, 0.15, { x, y }, 10);
			}
			else
			{
				addCircle(0.15, { x, y }, 10);
			}
		}
	}

	//addChain(20, 0.07, 0.15, { 1, 1 }, 10, pi/4);

	addSoftBody({ 1, 1 }, 12, 12, 0.1, 0.1, 0.045, 100, 0.06, 1);
	addSoftBody({ 6, 1 }, 8, 10, 0.3, 0.3, 0.14, 80, 0.06, 1);


	//int s = rigidBodies.size();
	//auto c1 = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(0.35, 0), vec2(-0.35, 0), 0., ps);
	//c1->makeSpringy(.1f, .3f);
	//constraints.push_back(std::move(c1));
	//auto c2 = std::make_unique<DistanceConstraint>(rigidBodies[s - 2].get(), rigidBodies[s - 3].get(), vec2(0.35, 0), vec2(-0.35, 0), 0., ps);
	//c1->makeSpringy(.1f, .3f);
	//constraints.push_back(std::move(c2));

	//rigidBodies[s - 1]->setCollType(0b0000000000000010);
	//rigidBodies[s - 2]->setCollType(0b0000000000000010);
	//rigidBodies[s - 3]->setCollType(0b0000000000000010);
	//rigidBodies[s - 1]->setCollidables(0b0000000000000001);
	//rigidBodies[s - 2]->setCollidables(0b0000000000000001);
	//rigidBodies[s - 3]->setCollidables(0b0000000000000001);

	// TODO: continuous collision
	// TODO: chain shape equivalent
	// TODO: setup mass based on density
	// TODO: test springy zero-distance constraint
	// TODO: 2x2 peg constraint
	// TODO: compound RigidBody made of multiple shapes

	

	/*real rad = 0.3;
	addCircle(rad, { x0 + (nLinks - 1) * linkLength, y0 }, 1);
	int s = rigidBodies.size();

	auto c = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(-rad, 0), vec2(linkLength / 2, 0), 0., ps);
	constraints.push_back(std::move(c));

	auto c2 = std::make_unique<LineConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(0, 0), vec2(0, 0), vec2(-1, 0), ps);
	constraints.push_back(std::move(c2));

	auto c3 = std::make_unique<AngleConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), static_cast<real>(0), ps);
	constraints.push_back(std::move(c3));*/

	// TODO: auto-initialization of direction & angle difference?

	////auto c1 = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(-0.35, 0), vec2(-0.35, 0), 0.7, ps);
	////c1->makeSpringy(.1f, .3f);
	////c1->setRange(0.2, 0.8);
	////constraints.push_back(std::move(c1));

	////auto c1b = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(0.7, 0.035), vec2(0.7, 0.035), 0.7, ps, true);
	////c1b->makeSpringy(.1f, .3f);
	////c1b->setRange(0.2, 0.8);
	////constraints.push_back(std::move(c1b));

	////auto c1c = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(0.35, 0), vec2(-0.35, 0), 0.7, ps);
	////c1c->makeSpringy(.1f, .3f);
	//////c1c->setRange(0.2, 0.8);
	////constraints.push_back(std::move(c1c));

	////auto c1d = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(-0.35, 0), vec2(0.35, 0), 0.7, ps);
	////c1d->makeSpringy(.1f, .3f);
	//////c1d->setRange(0.2, 0.8);
	////constraints.push_back(std::move(c1d));


	//auto c2 = std::make_unique<LineConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(0, 0), vec2(0, 0), vec2(0, -1), ps);
	////constraints.push_back(std::move(c2));

	//auto c3 = std::make_unique<AngleConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), static_cast<real>(0), ps);
	////c3->makeSpringy(.5f, .1f);
	////c3->setRange(-pi / 4, pi / 4);
	////constraints.push_back(std::move(c3));
	//addConvexPolygon(4, 2, { w / 2, h / 2 })->setAsUnremovable();




	/*addCircle(2, pixToCoords(pixWidth * 0.5, pixHeight * 0.75));
	auto circ1 = addCircle(1, pixToCoords(pixWidth * 0.25, pixHeight * 0.75));
	auto circ2 = addCircle(1, pixToCoords(pixWidth * 0.75, pixHeight * 0.75));


	circ1->setIInv(1);
	auto c = std::make_unique<AngleConstraint>(circ1, circ2, 0., ps);
	c->enableMotor(6*pi, 1000);
	constraints.push_back(std::move(c));*/

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

		for (auto& c : constraints)
		{
			c->draw(window, fraction);
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

		showStats(dt);

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
			// Don't try to collide two rigid bodies of infinite mass
			if (c->canCollideWith(rb.get()) && (c->mInv() || rb->mInv()))
			{
				checkCollision(c, rb.get());
			}
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

	//std::cout << constraints.size() << "\n";
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
				real fMax = 100.f; // / rb->mInv();

				// TODO: Consider force/acceleration limit & contact breaking
				auto newMC = std::make_unique<MouseConstraint>(rb, mh, ps, local, .05f, 4.f, fMax);
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

void Game::showStats(real frameTime)
{
	text.setCharacterSize(30);
	text.setFillColor(sf::Color::Blue);

	std::stringstream ss;
	ss << "FPS: " << std::fixed << std::setprecision(0) << 1. / frameTime << '\n'
		<< "Rigid bodies: " << rigidBodies.size() << '\n'
		<< "Constraints: " << constraints.size() << '\n'
		<< "Collisions: " << collidingPairs.size() << '\n'
		<< "Physics frequency: " << 1. / ps.dt << " Hz" << '\n'
		<< "Velocity iterations: " << ps.velIter << '\n'
		<< "Position iterations: " << ps.posIter << '\n';

	text.setString(ss.str());
	text.setPosition(0, 0);
	window.draw(text);
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

ConvexPolygon* Game::addConvexPolygon(const std::vector<vec2>& points, vec2 coords, real mInv)
{
	auto rb = std::make_unique<ConvexPolygon>(ps, points, mInv);
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

void Game::addChain(int nLinks, real linkWidth, real linkLength, vec2 start, real linkmInv, real angle)
{
	std::vector<vec2> pts = { {0, 0}, {linkLength, 0}, {linkLength, linkWidth}, {0, linkWidth} };

	for (int i = 0; i < nLinks; ++i)
	{
		real x = start.x + (i + 0.5) * linkLength * std::cos(angle);
		real y = start.y + (i + 0.5) * linkLength * std::sin(angle);

		RigidBody* rb = addConvexPolygon(pts, { x, y }, linkmInv);

		rb->rotateTo(angle);

		// TODO: pass in collision types as a parameter?
		rb->setCollType(0b0000000000000010);
		rb->setCollidables(0b0000000000000001);

		if (i == 0) continue;

		int s = rigidBodies.size();
		auto c = std::make_unique<DistanceConstraint>(rb, rigidBodies[s - 2].get(), vec2(-linkLength / 2, 0), vec2(linkLength / 2, 0), 0., ps);
		constraints.push_back(std::move(c));
	}
}

void Game::addSoftBody(vec2 minVertex, int nx, int ny, real xSpace, real ySpace, real particleRad, real particlemInv, real tOsc, real dampingRatio)
{
	std::vector<std::vector<Circle*>> particles;

	real frac = 0.1;

	for (int j = 0; j < ny; ++j)
	{
		particles.push_back({});

		for (int i = 0; i < nx; ++i)
		{
			vec2 coords(minVertex.x + xSpace * i, minVertex.y + ySpace * j);
			Circle* c = addCircle(particleRad, coords, particlemInv);
			c->setIInv(0);

			//c->setCollType(0b0000000000000010);
			//c->setCollidables(0b0000000000000001);

			particles[j].push_back(c);

			// TODO: helper functions for adding constraints
			if (i > 0)
			{
				Circle* left = particles[j][i - 1];
				
				auto constraint = std::make_unique<DistanceConstraint>(c, left, vec2{}, vec2{}, xSpace, ps);
				constraint->makeSpringy(tOsc, dampingRatio);
				constraint->allowFractionalChange(frac);
				constraints.push_back(std::move(constraint));
			}

			if (j > 0)
			{
				Circle* above = particles[j-1][i];

				auto constraint = std::make_unique<DistanceConstraint>(c, above, vec2{}, vec2{}, ySpace, ps);
				constraint->makeSpringy(tOsc, dampingRatio);

				constraint->allowFractionalChange(frac);

				constraints.push_back(std::move(constraint));
			}


			//auto c1 = std::make_unique<DistanceConstraint>(rigidBodies[s - 1].get(), rigidBodies[s - 2].get(), vec2(0.35, 0), vec2(-0.35, 0), 0., ps);
			
		}
	}



	real diagonalDist = std::sqrt(xSpace * xSpace + ySpace * ySpace);

	for (int j = 0; j < ny; ++j)
	{
		for (int i = 0; i < nx; ++i)
		{
			Circle* c = particles[j][i];

			if (i > 0 && j > 0)
			{
				Circle* aboveLeft = particles[j - 1][i - 1];

				auto constraint = std::make_unique<DistanceConstraint>(c, aboveLeft, vec2{}, vec2{}, diagonalDist, ps);
				constraint->makeSpringy(tOsc, dampingRatio);


				constraint->allowFractionalChange(frac);

				constraints.push_back(std::move(constraint));
			}
			
			if (i < nx - 1 && j > 0)
			{
				Circle* aboveRight = particles[j - 1][i + 1];

				auto constraint = std::make_unique<DistanceConstraint>(c, aboveRight, vec2{}, vec2{}, diagonalDist, ps);
				constraint->makeSpringy(tOsc, dampingRatio);

				constraint->allowFractionalChange(frac);

				constraints.push_back(std::move(constraint));
			}

			if (i > 0 && j < ny - 1)
			{
				Circle* belowLeft = particles[j + 1][i - 1];

				auto constraint = std::make_unique<DistanceConstraint>(c, belowLeft, vec2{}, vec2{}, diagonalDist, ps);
				constraint->makeSpringy(tOsc, dampingRatio);


				constraint->allowFractionalChange(frac);

				constraints.push_back(std::move(constraint));
			}

			if (i < nx - 1 && j < ny - 1)
			{
				Circle* belowRight = particles[j + 1][i + 1];

				auto constraint = std::make_unique<DistanceConstraint>(c, belowRight, vec2{}, vec2{}, diagonalDist, ps);
				constraint->makeSpringy(tOsc, dampingRatio);

				constraint->allowFractionalChange(frac);

				constraints.push_back(std::move(constraint));
			}
		}
	}

	/*real diagonalDist = std::sqrt(xSpace * xSpace + ySpace * ySpace);

	for (int tot = 0; tot < ny; ++tot)
	{
		int j = tot, i = 0;
		Circle* first = particles[j][i];
		Circle* second = nullptr;

		while (--j >= 0)
		{
			++i;

			second = particles[j][i];

			auto constraint = std::make_unique<DistanceConstraint>(first, second, vec2{}, vec2{}, diagonalDist, ps);
			constraints.push_back(std::move(constraint));

			first = second;
		}
	}*/


	/*for (int j = 1; j < ny-1; ++j)
	{
		for (int i = 1; i < nx-1; ++i)
		{
			Circle* c = particles[j][i];

			Circle* aboveLeft = particles[j - 1][i - 1];
			Circle* aboveRight = particles[j - 1][i + 1];
			Circle* belowLeft = particles[j + 1][i - 1];
			Circle* belowRight = particles[j + 1][i + 1];

			real diagonalDist = std::sqrt(xSpace * xSpace + ySpace * ySpace);

			auto constraint = std::make_unique<DistanceConstraint>(c, aboveLeft, vec2{}, vec2{}, diagonalDist, ps);
			constraint->makeSpringy(tOsc, dampingRatio);
			constraints.push_back(std::move(constraint));

			constraint = std::make_unique<DistanceConstraint>(c, aboveRight, vec2{}, vec2{}, diagonalDist, ps);
			constraint->makeSpringy(tOsc, dampingRatio);
			constraints.push_back(std::move(constraint));

			constraint = std::make_unique<DistanceConstraint>(c, belowLeft, vec2{}, vec2{}, diagonalDist, ps);
			constraint->makeSpringy(tOsc, dampingRatio);
			constraints.push_back(std::move(constraint));

			constraint = std::make_unique<DistanceConstraint>(c, belowRight, vec2{}, vec2{}, diagonalDist, ps);
			constraint->makeSpringy(tOsc, dampingRatio);
			constraints.push_back(std::move(constraint));
		}
	}*/
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