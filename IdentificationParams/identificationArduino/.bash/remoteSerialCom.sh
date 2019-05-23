#!/bin/bash

# $1: Premier argument est le nom d'utilisateur sur le RPI
# $2: Deuxieme argument est l'adresse du RPI
# $3: Troisieme argument est le port du Arduino
# $4: Quatrieme argument vitesse de communication
# transfert du fichier compile "firmware.hex" vers le RPI
echo ssh -t $1@$2 'screen '$3' '$4
ssh -t $1@$2 'minicom -D '$3' -b '$4

