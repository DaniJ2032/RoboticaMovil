# TP1: Modelo cinemático y odometrı́a en robot de tracción diferencial

## Parte 1: cálculos analíticos para robot de tracción diferencial

#### 1. Determinar de forma analítica el radio del camino circular que realiza el robot al ajustar la velocidad lineal y angular a valores constantes.

Para calcular analíticamente el radio de la trayectoria circular que describe un robot al mantener constantes su velocidad lineal $v$ y su velocidad angular $ω$, se puede emplear la relación entre estas dos velocidades.

La velocidad lineal $v$ de un objeto que se mueve en un camino circular está vinculada a su velocidad angular $ω$ y al radio $R$ de dicho camino mediante la fórmula:

$$v=w*R$$

A partir de esta relación, se puede despejar el radio $R$ de la siguiente manera:

$$R=\frac{v}{w}$$

#### 2. Realizar el cálculo con valores numéricos para dos velocidades cualesquiera teniendo en cuenta las velocidades máximas del robot. Nota: Tener en cuenta los límites de velocidad y los parámetros cinemáticos (el radio de la rueda $R$ y la distancia entre ruedas $b$) del robot.

Para una $v=0,1 m/s$ y una $w=1 rad/s$ y utilizando la fórmula del punto anterior se obtiene R:

$$R=\frac{0,1}{1}=0,1$$

Utilizando las ecucaciones del centro odométrico en R (sist. de coord. del robot), se puede obtener la velocidad angular de cada rueda:

$$v=R(\frac{w_R+w_L}{2})$$ $$w=R(\frac{w_R-w_L}{b})$$

Depejando $w_L$ de la primera ecucación y despejando $w_R$ de la segunda, se obtinen las siguientes ecuaciones:

$$w_L=\frac{2.v}{R}-w_R$$ 

$$w_R=\frac{w.b}{R}+w_L$$

Se tienen 2 ecuaciones con 2 incógnitas, con lo cual reemplazando una en otra se obtienen ambas velocidades angulares. Tener en cuenta que $b=0,16m$.

$$w_R=\frac{w.b}{R}+\frac{2.v}{R}-w_R$$

$$w_R=\frac{\frac{w.b}{R}+\frac{2.v}{R}}{2}$$

$$w_R=1,8$$

$$w_L=\frac{2.v}{R}-w_R$$ 

$$w_L=0,2$$

Teneniendo en cuenta el radio de la rueda (0,033 m) y con las velocidades angulares se calculan las velocidades de cada rueda con las siguientes ecuaciones:

$$v_L=R*w_L=-0,007$$

$$v_R=R*w_R=0,059$$

#### 3. Calcular la velocidad lineal y angular para que el robot realice un camino circular con un radio a elección entre 0,5m y 2,0m.

Se elige un $R=2m$. Esto indica que $v$ y $w$ son iguales para que la relación entre ambos de 1. Por ej: 

$$v=0,1$$ $$w=0,1$$

Partiendo de las ecuaciones obtenidas en el punto anterior:

$$w_R=\frac{\frac{w.b}{R}+\frac{2.v}{R}}{2}$$

$$w_R=0,054$$

$$w_L=\frac{2.v}{R}-w_R$$ 

$$w_L=0,046$$



#### 4. Calcular las velocidades lineales y angulares de las ruedas (izquierda y derecha) del robot para el camino circular del punto anterior.

Considerando el radio de la rueda (0,033 m) y utilizando las velocidades angulares, se pueden calcular las velocidades de cada rueda mediante las siguientes ecuaciones:

$$v_L=R*w_L=0,0015$$

$$v_R=R*w_R=0,0018$$

#### 5. ¿Qué sucede si se intercambian las velocidades de ruedas entre izquierda y derecha?

Si se intercambian las velocidades de las ruedas izquierda y derecha, el robot se desplazará en una trayectoria circular en la dirección opuesta.
