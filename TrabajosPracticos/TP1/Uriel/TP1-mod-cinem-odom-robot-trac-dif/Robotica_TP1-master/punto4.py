import pandas as pd
import matplotlib.pyplot as plt

def graficar_ruta_circular(ruta_archivo, etiqueta, ax):
    # Cargar los datos del archivo de registro
    datos = pd.read_csv(ruta_archivo, sep='\s+', header=None, names=['timestamp', 'x', 'y', 'orientation', 'linear_velocity', 'angular_velocity'])

    # Filtrar las filas donde todos los datos son cero
    datos = datos.loc[~(datos == 0).all(axis=1)]

    # Ajustar los timestamps para que comiencen en cero
    datos['timestamp'] -= datos['timestamp'].min()

    # Convertir columnas a numpy arrays para evitar problemas de indexación multidimensional
    coord_x = datos['x'].to_numpy()
    coord_y = datos['y'].to_numpy()

    # Graficar el camino seguido por el robot
    ax.plot(coord_x, coord_y, marker='o', linestyle='-', label=etiqueta)
    # Agregar una flecha indicando el sentido de avance
    ax.quiver(coord_x[:-1], coord_y[:-1], coord_x[1:]-coord_x[:-1], coord_y[1:]-coord_y[:-1], scale_units='xy', angles='xy', scale=1, color='green')

# Configuración de la figura
figura, ejes = plt.subplots(2, 2, figsize=(14, 14))

# Graficar cada combinación
graficar_ruta_circular('circular_1.txt', 'Lineal +, Angular +', ejes[0, 0])
ejes[0, 0].set_title('Lineal +, Angular +')

graficar_ruta_circular('circular_2.txt', 'Lineal +, Angular -', ejes[0, 1])
ejes[0, 1].set_title('Lineal +, Angular -')

graficar_ruta_circular('circular_3.txt', 'Lineal -, Angular +', ejes[1, 0])
ejes[1, 0].set_title('Lineal -, Angular +')

graficar_ruta_circular('circular_4.txt', 'Lineal -, Angular -', ejes[1, 1])
ejes[1, 1].set_title('Lineal -, Angular -')

# Ajustar la relación de aspecto y etiquetas
for ax in ejes.flat:
    ax.set_xlabel('Coordenada X')
    ax.set_ylabel('Coordenada Y')
    ax.axis('equal')
    ax.legend()

# Mostrar los gráficos
plt.tight_layout()
plt.show()
