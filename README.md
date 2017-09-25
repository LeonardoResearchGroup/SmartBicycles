# SmartBicycles

SmartBicycles es una iniciativa para el desarrollo de un pelotón de ciclistas que implementan un algoritmo de control de velocidad sobre un hardware especializado para conexion inalámbrica vehicular.  

En esta carpeta estan los códigos de los modulos controladores de una pareja de líder y seguidor, y el código del módulo controlador de un prototipo de simulador de escritorio.

Para implementar la solución en **biciletas urbanas** debe utilizar los códigos *FollowerFull* y *LeaderFull* que se instalarían en el módulo de control de cada bicileta. A cada seguidor se le instala un *LeaderFull*

Para implementar la solución en un **simulador de escritorio** debe utilizar los códigos *Prototipo2.0* y *Simulador3D*. El código de *Prototipo2.0* va en el modulo de control de la bicicleta. Dentro de la carpeta *Simulador3D* encuentra:

- Una applicacion *bike_prototype_v3_32bit.app*: Es una mundo en 3D con al representación de calles urbanas por las que rueda una bicicleta controlada por un módulo de control.
- Un servidor *bike_serial_to_...* en tres versiones: *TCP*, *JOYSTIC*, *KEYBOARD*. Escoja la versión a correr dependiendo si está utilizando una bicicleta estática, un timón con motor de estimulos hápticos, o simplemente un teclado.

### Descrición del funcionamiento del código del servidor *simulador de escritorio*

Este prototipo esta constituido de 3 compunentes: i) una interfaz fisica que va instalada en la bicileta
ii) un servidor y iii) una aplicación que simula el desplazamiento de una bicicleta en la ciudad. 

La interfaz fisica esta controlada por un arduino que tiene conectados un sensor de velocidad y 
tres actuadores: un led rojo, uno verde y un motor bidireccional de rotacion continua.

El arduino envia la velocidad unicamente cuando hay un cambio. De esta forma el canal solo se ocupa cuando 
hay cambios que notificar. El servidor lee el puerto serial solamente cuando se produce un evento, es decir
que lo lee solo cuando hay una señal enviada desde el arduino. Cuando lee la señal la convierte en un numero
flotante y lo envia al simulador 3D a través del servidor. De acuerdo al valor recibido el simulador 3D acelera o
desacelera la bicicleta ene l mundo virtual. 

El simulador 3D esta constantemente detectando si la bicicleta esta dentro de de los límites de una ola verde. Cuando la sobrepasa envia una señal de -1 que significa desaceleracion. Cuando se rezaga manda una 
señal positiva de aceleración y finalmente cuando esta en la ola verde manda una señal igual a 0 que indica 
que esta dentro de la ola verde. Esos datos los recibe el servidor y se los envia al arduino por el canal de
bluetooth. Dependiendo del valor arduino enciende los leds rojo para desacelerar, verde para acelerar o azul
para indicar que esta en la velocidad correcta. Con base en la misma señal hace girar el motor hacia adelante
cuando la señal es positiva y hacia atrás cuando es negativa.

Ma información y detalles técnicos disponibles en: http://www.smartartifact.com/smart-bicycles/

