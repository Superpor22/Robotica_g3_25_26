# 🤖 Repositorio de Robótica - G3 (2025-2026)

Este repositorio contiene las **actividades y proyectos** desarrollados para la asignatura de **Robótica** del **Grupo 3** durante el curso **2025-2026**.

Los proyectos están construidos sobre el framework **[RoboComp](https://robocomp.github.io/robocomp/)** y el simulador **[Webots](https://cyberbotics.com/)**.

---

## 🧩 Estructura del Proyecto

```bash
robotica_G3/
├── actividad1/             # Código principal de la práctica 1
├── config/                 # Archivos de configuración
└── README.md               # Este archivo
```

Se asume que:
- RoboComp está instalado en: `~/robocomp/`
- Este repositorio está clonado en: `~/robotica_G3/`

---

## ⚙️ Configuración y Compilación

### 🕹️ 1. Compilar el componente **Joystick** (si es necesario)

Este componente publica los datos del **joystick** para que puedan ser utilizados por el resto del sistema.

```bash
cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish/
cmake .
make -j21
```

---

### 🤖 2. Compilar el componente **Actividad 1**

Este es el componente **principal** del grupo G3 que controla el robot en la simulación.

```bash
cd ~/robotica_G3/actividad1/
cmake .
make
```

---

## 🚀 Ejecución de la Actividad 1

Para lanzar la simulación completa, es necesario ejecutar **5 componentes** en **5 terminales separadas**.

> ⚠️ **Importante:** Antes de ejecutar los componentes, asegúrate de que el **gestor de nodos de RoboComp** (`rcnode`) esté corriendo.

```bash
rcnode
```

Una vez que `rcnode` esté activo, abre **una terminal por cada componente** en el orden que se indica a continuación 👇

---

### 🧱 1. Webots (Simulador)

Abre el simulador Webots, que servirá como entorno de ejecución del robot.

```bash
cd ~/robocomp/components/webots-bridge/
webots
```

> 💡 Desde la interfaz gráfica de Webots, abre el mundo (`.wbt`) correspondiente a la Actividad 1.

---

### 🌉 2. Webots-Bridge (Puente de Comunicación)

Este componente conecta **Webots** con **RoboComp**, permitiendo el intercambio de datos.

```bash
cd ~/robocomp/components/webots-bridge/
bin/Webots2Robocomp etc/config
```

---

### 🛰️ 3. LIDAR (Sensor 3D)

Este componente simula el **sensor LIDAR 3D**, que proporciona información del entorno al robot.

```bash
cd ~/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D/
bin/Lidar3D etc/config_helios_webots
```

---

### 🎮 4. Joystick (Control Manual)

Este componente lee los valores del joystick y los envía al sistema para controlar el robot.

```bash
cd ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish/
bin/JoystickPublish etc/config_shadow
```

> ⚠️ Asegúrate de tener un **joystick físico conectado** antes de lanzar este componente.

---

### 🤖 5. Componente Principal (Actividad 1)

Finalmente, ejecuta el componente principal de la Actividad 1 — el que controla el robot en base a los datos del LIDAR y el joystick.

```bash
cd ~/robotica_G3/actividad1/
bin/actividad1 etc/config
```

> 💡 Si tu binario o archivo de configuración tiene otro nombre, ajústalo en el comando anterior.

---

## 🧠 Consejos Útiles

- ✅ Lanza los componentes **en el orden indicado**.  
- 💬 Cada componente debe ejecutarse en una **terminal diferente**.  
- 🔍 Si alguno falla, revisa los archivos de configuración en la carpeta `etc/`.  
- 🧩 Puedes detener todos los procesos con `Ctrl + C` en cada terminal.

---

## 👥 Autores

**Grupo G3 - Robótica (2025-2026)**  
Universidad de [Extremadura]  
Aurotes: [Guadalupe González Santos, Máximo Bueno Martínez y José Antonio Bravo Romero]

---

## 🪪 Licencia

Este proyecto está protegido bajo la licencia **Creative Commons Atribución-NoComercial-CompartirIgual 4.0 Internacional (CC BY-NC-SA 4.0)**.  
© 2025 Universidad de Extremadura (UEx) - EPCC - Grupo G3.

Más información en 👉 [creativecommons.org/licenses/by-nc-sa/4.0/](https://creativecommons.org/licenses/by-nc-sa/4.0/)

---
