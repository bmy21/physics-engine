#include <iostream>

#include "Game.h"

int main()
{
	//_control87(_EM_INVALID, _EM_ZERODIVIDE | _EM_OVERFLOW);

	Game game;
	game.run();

	return 0;
}