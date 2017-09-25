# SmartBicycles

SmartBicycles es una iniciativa para el desarrollo de un pelotón de ciclistas que implementan un algoritmo de control de velocidad sobre un hardware especializado para conexion inalámbrica vehicular.  

En esta carpeta estan los códigos de los modulos controladores de una pareja de líder y seguidor, y el código del módulo controlador de un prototipo de simulador de escritorio.

Para implementar la solución en **biciletas urbanas** debe utilizar los códigos *FollowerFull* y *LeaderFull* que se instalarían en el módulo de control de cada bicileta. A cada seguidor se le instala un *LeaderFull*

Para implementar la solución en un **simulador de escritorio** debe utilizar los códigos *Prototipo2.0* y *Simulador3D*. El código de *Prototipo2.0* va en el modulo de control de la bicicleta. Dentro de la carpeta *Simulador3D* encuentra:

- Una applicacion *bike_prototype_v3_32bit.app*: Es una mundo en 3D con al representación de calles urbanas por las que rueda una bicicleta controlada por un módulo de control.
- Un servidor *bike_serial_to_...* en tres versiones: *TCP*, *JOYSTIC*, *KEYBOARD*. Escoja la versión a correr dependiendo si está utilizando una bicicleta estática, un timón con motor de estimulos hápticos, o simplemente un teclado.
