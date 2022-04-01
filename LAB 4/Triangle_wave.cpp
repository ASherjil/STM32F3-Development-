#include <iostream>
#include <cmath>

int main()
{
 
	int y{}, counter{}, i{};
/*
	for (int i{}; i < 100000;i+=50)
	{
		y = abs((i % (2*3640)) - 3640);
		std::cout << y << "	"<<++counter<<"\n";

		if (y==0) // if y==0
		{
			break;
		}

	}
*/
	do
	{
		y = abs((i % (2 * 3640)) - 3640);
		std::cout << y << "	" << ++counter << "\n";
		i += 100;

	} while (y > 0);


}


/*
1- y = abs((i % scale) - amplitude);

	scale = 2* amplitude 

2-To alter the resolution increment the variable by more than 1, 

	for (int i{}; i < 10000;i+=5)
	{
		y = abs((i % (2*3640)) - 3640);
		std::cout << y << "\n";
	}

*/
