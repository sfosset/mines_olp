from BackProp_Python_v2 import NN
from vrep_pioneer_simulation import VrepPioneerSimulation
from rdn import Pioneer
import rospy
from online_trainer import OnlineTrainer
import json
import threading


robot = VrepPioneerSimulation()
#robot = Pioneer(rospy)

choice = input('Do you want to load previous network? (y/n) --> ')
if choice == 'y':
    with open('last_w.json') as fp:
        json_obj = json.load(fp)

    hidden_layer_size = json_obj["hidden_neurons"]
    network = NN(3, hidden_layer_size, 2)

    for i in range(3):
        for j in range(hidden_layer_size):
            network.wi[i][j] = json_obj["input_weights"][i][j]
    for i in range(hidden_layer_size):
        for j in range(2):
            network.wo[i][j] = json_obj["output_weights"][i][j]
else:
    hidden_layer_size = "0"
    while not hidden_layer_size.isdigit() or int(hidden_layer_size) == 0 :
        hidden_layer_size = input('Enter hidden layout size. --> ')

    network = NN(3, int(hidden_layer_size), 2)


trainer = OnlineTrainer(robot, network)

choice = ''
while choice!='y' and choice !='n':
    choice = input('Do you want to learn? (y/n) --> ')

if choice == 'y':
    trainer.training = True
elif choice == 'n':
    trainer.training = False

target = input("Enter the first target : x y radian --> ")
target = target.split()
for i in range(len(target)):
    target[i] = float(target[i])
print('New target : [%d, %d, %d]'%(target[0], target[1], target[2]))

continue_running = True
while(continue_running):

    thread = threading.Thread(target=trainer.train, args=(target,))
    trainer.running = True
    thread.start()

    #Ask for stop running
    input("Press Enter to stop the current training")
    trainer.running = False
    choice = ''
    while choice!='y' and choice !='n':
        choice = input("Do you want to continue ? (y/n) --> ")

    if choice == 'y':
        choice_learning = ''
        while choice_learning != 'y' and choice_learning !='n':
            choice_learning = input('Do you want to learn ? (y/n) --> ')
        if choice_learning =='y':
            trainer.training = True
        elif choice_learning == 'n':
            trainer.training = False
        target = input("Move the robot to the initial point and enter the new target : x y radian --> ")
        target = target.split()
        for i in range(len(target)):
            target[i] = float(target[i])
        print('New target : [%d, %d, %d]'%(target[0], target[1], target[2]))
    elif choice == 'n':
        continue_running = False


json_obj = { "hidden_neurons": hidden_layer_size, "input_weights": network.wi, "output_weights": network.wo}
with open('last_w.json', 'w') as fp:
    json.dump(json_obj, fp)

print("The last weights have been stored in last_w.json")
