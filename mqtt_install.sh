#!/bin/bash

dir="./test.conf"
pub="./pub"
sub="./sub"
message="$1"
ip=$(hostname -I | cut -d ' ' -f1)

echo "Creando servidor con ip $ip"

which mosquitto > /dev/null ||
	(sudo apt-get install -y mosquitto &&
	sudo apt-get install -y mosquitto-clients)

echo "listener 1883
listener 8080
protocol websockets
allow_anonymous true" > $dir

echo "Creando fichero sub en $sub"
echo "mosquitto_sub -h $ip -t test" > $sub
chmod +x $sub
echo "Creando fichero pub en $pub"
echo "mosquitto_pub -h $ip -t test -m "'$@'> $pub
chmod +x $pub

mosquitto -c $dir -v
