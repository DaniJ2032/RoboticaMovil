import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del archivo de registro
data = pd.read_csv('circular.txt', sep='\s+', header=None, names=['timestamp', 'x', 'y', 'orientation', 'linear_velocity', 'angular_velocity'])

# Filtrar las filas donde todos los datos son cero
data = data.loc[~(data == 0).all(axis=1)]

# Ajustar los timestamps para que comiencen en cero
data['timestamp'] -= data['timestamp'].min()

# Convertir columnas a numpy arrays para evitar problemas de indexación multidimensional
time_stamps = data['timestamp'].to_numpy()
pos_x = data['x'].to_numpy()
pos_y = data['y'].to_numpy()

# Configuración de la figura
plt.figure(figsize=(14, 7))

# i) Gráfico del camino seguido por el robot
plt.subplot(1, 2, 1)
plt.plot(pos_x, pos_y, marker='o', color='blue', linestyle='--')
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')
plt.title('Camino del robot')
plt.axis('equal')  # Relación de aspecto 1:1

# ii) Gráfico de la trayectoria (pose respecto al tiempo)
plt.subplot(1, 2, 2)
plt.plot(time_stamps, pos_x, label='Posición X', color='green')
plt.plot(time_stamps, pos_y, label='Posición Y', color='red')
plt.xlabel('Tiempo (segundos)')
plt.ylabel('Posición')
plt.title('Trayectoria en función del tiempo')
plt.legend()

# Mostrar los gráficos
plt.tight_layout()
plt.show()