import ardrone
import time

drone = ardrone.ARDrone()

while True:
    data = drone.navdata 
    time.sleep(1)
    # print(data)
    # print(data.get('demo'))

    baza = data.get('demo', {}).get('battery')
    print('Зарад баттареи ', baza, " %")
    
    if data != {}:
        break

    
