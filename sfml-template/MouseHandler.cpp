#include "MouseHandler.h"

MouseHandler::MouseHandler(const sf::RenderWindow& window, const PhysicsSettings& ps):
	window(window), ps(ps)
{
}

void MouseHandler::update()
{
	sf::Vector2i result = sf::Mouse::getPosition(window);

	mPixCoords = vec2(result.x, result.y);
	mCoords = { mPixCoords.x / ps.pixPerUnit, mPixCoords.y / ps.pixPerUnit };
}
