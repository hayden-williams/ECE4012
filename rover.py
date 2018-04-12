import requests
import time
import json



while True: 
	r = requests.get('http://128.61.14.86:3000/rover').json()
	print(r)

	direction = r['direction']
	length = r['length']
	bearing = r['bearing']
	emergency = r['emergency']
	end = r['ended']
	arrived = r['arrived']
	home = r['gotHome']

	time.sleep(1)








