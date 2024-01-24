#ª/bin/bash

while :; do
  read line < /dev/ttyACM0
  mosquitto_pub -h $1 -t $2 -m "$line"
  echo $line
done
