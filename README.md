# ic-final-server

## Pasos
Lo primero que se debe hacer es ejecutar el script "mqtt_install.sh", una vez hecho esto se nos generarán los siguientes ficheros: test.conf, sub y pub. Además de esto, también se habrá arrancado el servicio de mosquitto. Para probar que todo funciona correctamente lo único que se debe hacer es ejecutar por separado el script sub y el pub. El sub se suscribirá y nos mostrará por consola si se ha recibido algún mensaje. Para poder enviar un mensaje se deberá ejecutar el pub pasando un texto como parámetro.