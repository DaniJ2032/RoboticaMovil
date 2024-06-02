import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del archivo de registro
data = pd.read_csv('log.txt', sep='\s+', header=None, names=['timestamp', 'x', 'y', 'orientation', 'linear_velocity', 'angular_velocity'])

# Filtrar las filas donde todos los datos son cero
data = data.loc[~(data == 0).all(axis=1)]

# Ajustar los timestamps para que comiencen en cero
data['timestamp'] -= data['timestamp'].min()

# Convertir columnas a numpy arrays para evitar el problema de indexación multidimensional
time_stamps = data['timestamp'].to_numpy()
pos_x = data['x'].to_numpy()
pos_y = data['y'].to_numpy()
lin_vel = data['linear_velocity'].to_numpy()
ang_vel = data['angular_velocity'].to_numpy()

# Configuración de la figura
plt.figure(figsize=(20, 8))

# i) Gráfico del camino seguido por el robot
plt.subplot(1, 3, 1)
plt.plot(pos_x, pos_y, marker='o', color='blue', linestyle='--')
plt.xlabel('Posición X')
plt.ylabel('Posición Y')
plt.title('Trayectoria del robot')
plt.axis('equal')  # Relación de aspecto 1:1

# ii) Gráfico de la trayectoria (pose respecto al tiempo)
plt.subplot(1, 3, 2)
plt.plot(time_stamps, pos_x, label='Posición X', color='green')
plt.plot(time_stamps, pos_y, label='Posición Y', color='red')
plt.xlabel('Tiempo (segundos)')
plt.ylabel('Posición')
plt.title('Posición en función del tiempo')
plt.legend()

# iii) Gráfico de la velocidad del robot respecto al tiempo
plt.subplot(1, 3, 3)
plt.plot(time_stamps, lin_vel, label='Velocidad lineal', color='purple')
plt.plot(time_stamps, ang_vel, label='Velocidad angular', color='orange')
plt.xlabel('Tiempo (segundos)')
plt.ylabel('Velocidad')
plt.title('Velocidades en función del tiempo')
plt.legend()

# Mostrar los gráficos
plt.tight_layout()
plt.show()
