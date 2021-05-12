import serial
import serial.tools.list_ports

import serial
import serial.tools.list_ports

import csv

print("Recherche d'un port serie...")

ports = serial.tools.list_ports.comports(include_links=False)

if (len(ports) != 0): # on a trouvé au moins un port actif

    if (len(ports) > 1):     # affichage du nombre de ports trouvés
        print (str(len(ports)) + " ports actifs ont ete trouves:") 
    else:
        print ("1 port actif a ete trouve:")

    ligne = 1

    for port in ports :  # affichage du nom de chaque port
        print(str(ligne) + ' : ' + port.device)
        ligne = ligne + 1
    
    baud = 115200

    # on établit la communication série
    #arduino = serial.Serial(ports[portChoisi - 1].device, baud)
    arduino = serial.Serial('COM3', baud)

    print('Connexion a ' + arduino.name + ' a un baud rate de ' + str(baud))

    # si on reçoit un message, on l'affiche
    file = open('data.csv', 'w')
    writer = csv.writer(file)
    nomcolonne = "gpsfix;fixquality;latitude;longitude;status;packetNum;data;RSSIpacket;packetReceived;SNR;RSSI"
    writer.writerow([nomcolonne])
    while True:
        data = arduino.readline()[:-2]
        print(data.decode('utf-8'))
        if data:
            #print(data)
            writer = csv.writer(file)
            writer.writerow([data.decode('utf-8')])
            #file.write(data.decode('utf-8'))

else: # on n'a pas trouvé de port actif
    print("Aucun port actif n'a ete trouve")
