# Dexter
El proyecto consiste en la construcción de un robot autónomo capaz de navegar por un laberinto utilizando una combinación de hardware y software especializado. El robot estará equipado con un sensor de distancia VL6180X, el cual será fundamental para detectar obstáculos y calcular distancias mientras se desplaza por el laberinto. Este sensor permitirá al robot tomar decisiones informadas sobre la dirección a seguir para evitar colisiones y encontrar la ruta más eficiente hacia la salida del laberinto.
Para controlar los movimientos del robot, se utilizarán motores Pololu de 6V, los cuales serán gestionados por un driver TA 6586. Este driver proporcionará la potencia necesaria para que los motores puedan desplazar el robot con precisión y control, permitiéndole moverse de manera suave y estable a través del laberinto.
La placa microprocesadora ESP32 será el cerebro del robot, encargada de procesar los datos del sensor de distancia y tomar decisiones en tiempo real para guiar al robot por el laberinto. Esta placa estará programada con algoritmos de navegación y control que permitirán al robot realizar maniobras complejas, como girar en esquinas y evitar obstáculos, con el objetivo de encontrar la salida del laberinto de la manera más rápida y eficiente posible.
Además, para garantizar la autonomía del robot durante su recorrido por el laberinto, se utilizará una batería de 600 miliamperios a 7.4 voltios. Esta batería proporcionará la energía necesaria para alimentar todos los componentes del robot, asegurando que pueda completar su misión sin interrupciones.

