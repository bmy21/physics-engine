#include "MouseHandler.h"

MouseHandler::MouseHandler(const sf::RenderWindow& window, real pixPerUnit) :
	window(window), pixPerUnit(pixPerUnit)
{
}

void MouseHandler::update()
{
	sf::Vector2i result = sf::Mouse::getPosition(window);

	mPixCoords = vec2(result.x, result.y);
	mCoords = { mPixCoords.x / pixPerUnit, mPixCoords.y / pixPerUnit };
}
