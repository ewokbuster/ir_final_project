# Alternative Code
from random import *

def alternative (obj robot1, obj robot2, pizza):
	# Test 1- which robot is closer to target pizza ?
	if distance(robot1, pizza) < distance(robot2, pizza):
		# Lock target pizza for retrieval and execute with Robot1
		pizza.locked = True
		retrieve(robot1, pizza)
	elif distance(robot1, pizza) > distance(robot2, pizza):
		# Lock target pizza for retrieval and execute with Robot2
		pizza.locked = True
		retrieve(robot2, pizza)
	# If both robots are in equal distance from target pizza - instantiate alternatives.
	else:
		# Target pizza is deliberately locked so it will be ignored for the new search.
		pizza.locked = True
		# New pizzas will be sought by the robots from scratch, using newPizza1 and 2. 
		newPizza1 = Search(robot1)
		alter1 = distance(robot1, newPizza1)
		newPizza2 = Search(robot2)
		alter2 = distance(robot2, newPizza2)
		# Calculate the alternatives- if robot 1 is closer to its alternative rather than
		# robot 2 - newPizza1 will be locked for retrieval by robot 1 and robot 2 will
		# execute retrieval for the original target pizza.
		if alter1 < alter2:
			newPizza1.locker = True
			retrieve(robot1, newPizza1)
			retrieve(robot2, pizza)
		# Vice Versa
		elif alter1 > alter2:
			newPizza2.locker = True
			retrieve(robot2, newPizza2)
			retrieve(robot1, pizza)
		# Extremely rare occasion where the robots are in equal distances to their
		# alternatives- they will be chosen at random.
		else:
			winner =  random.randint(1, 2)
			if winner == 1:
				# Robot 1 goes for target pizza, robot 2 to its alternative.
				newPizza2.locker = True
				retrieve(robot2, newPizza2)
				retrieve(robot1, pizza)
			else:
				# Robot 2 goes for target pizza, robot 1 to its alternative.
				newPizza1.locker = True
				retrieve(robot1, newPizza1)
				retrieve(robot2, pizza)
			
