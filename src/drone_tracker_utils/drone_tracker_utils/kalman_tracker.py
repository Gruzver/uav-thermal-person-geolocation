"""
kalman_tracker.py

Filtro de Kalman con modelo de velocidad constante para suavizar la secuencia
de posiciones GPS de un mismo objetivo a lo largo del tiempo.

Lo usan tanto el georeferenciador sincronizado por timestamp como la variante
NRT, que comparten el mismo modelo de movimiento.
"""

import numpy as np

# Intervalo nominal entre frames a 30 Hz, usado cuando no se puede derivar un
# dt valido de los timestamps.
DEFAULT_DT = 1.0 / 30.0


class KalmanTracker:
    """
    Filtro de Kalman con modelo de velocidad constante para suavizar GPS.

    Estado: [lat, lon, vel_lat, vel_lon]
    Observacion: [lat, lon]

    Args:
        lat: Latitud inicial de la deteccion.
        lon: Longitud inicial de la deteccion.
        init_time: Timestamp de la primera deteccion (segundos).
        process_noise: Varianza del ruido de proceso (aceleracion no modelada).
        measurement_noise: Varianza del ruido de medicion (calculo pixel->GPS).
    """

    def __init__(
        self,
        lat: float,
        lon: float,
        init_time: float,
        process_noise: float = 1e-10,
        measurement_noise: float = 1e-8,
    ) -> None:
        self.last_time = init_time

        # Estado: posicion inicial conocida, velocidad 0
        self.x = np.array([lat, lon, 0.0, 0.0])

        # Covarianza inicial: alta incertidumbre en velocidad
        self.P = np.diag([1e-10, 1e-10, 1e-8, 1e-8])

        # Parametros de ruido guardados para actualizar Q cuando cambia dt
        self._pn = process_noise
        self._mn = measurement_noise

        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])
        self.R = np.diag([measurement_noise, measurement_noise])
        self.I = np.eye(4)

        self._build_FQ(DEFAULT_DT)

    def _build_FQ(self, dt: float) -> None:
        """Reconstruye F y Q para el dt dado."""
        self.F = np.array([
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        q = self._pn
        self.Q = np.diag([q, q, q * 100.0, q * 100.0])

    def predict(self, current_time: float) -> tuple:
        """
        Avanza el estado al tiempo indicado.

        Args:
            current_time: Timestamp actual en segundos.

        Returns:
            Tupla (lat, lon) predicha.
        """
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = DEFAULT_DT
        self._build_FQ(dt)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return float(self.x[0]), float(self.x[1])

    def update(self, lat: float, lon: float, current_time: float) -> tuple:
        """
        Incorpora la medicion GPS y devuelve la posicion filtrada.

        Args:
            lat: Latitud medida.
            lon: Longitud medida.
            current_time: Timestamp de la medicion en segundos.

        Returns:
            Tupla (lat_filtrada, lon_filtrada).
        """
        z = np.array([lat, lon])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
        self.last_time = current_time
        return float(self.x[0]), float(self.x[1])
