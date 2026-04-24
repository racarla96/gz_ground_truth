# gz_ground_truth

Paquete ROS 2 **Jazzy** que publica la **TF de corrección entre el ground truth de
Gazebo y el frame raíz de localización del robot** (odometría o mapa).

---

## Concepto

Gazebo proporciona la pose exacta del robot en el mundo (`T_world_base`).
El árbol TF de ROS tiene la pose estimada del robot desde el frame de localización
(`T_loc_base`). La **corrección** es:

```
T_world_loc = T_world_base · inv(T_loc_base)
```

La TF resultante `world → localization_frame` cuantifica cuánto ha derivado la
odometría (o el localizador) respecto al ground truth perfecto del simulador.

---

## Escenarios

### A — Robot único, sin localizador (solo odometría)

```
Árbol TF ROS:  caddy_ai2/odom → caddy_ai2/base_footprint
TF publicada:  world → caddy_ai2/odom
```

```yaml
# config/caddy_ai2_single_odom.yaml
ground_truth_tf_publisher:
  ros__parameters:
    use_sim_time: true
    multi_robot: false
    gz_model_name: caddy_ai2
    tf_prefix: caddy_ai2
    ground_truth_frame: world
    base_suffix: base_footprint
    localization_frame: caddy_ai2/odom
    publish_rate: 50.0
```

```bash
ros2 launch gz_ground_truth single_odom.launch.py
# o con robot personalizado:
ros2 launch gz_ground_truth single_odom.launch.py \
    gz_model_name:=my_robot tf_prefix:=my_robot
```

---

### B — Robot único, con localizador (mapa global)

```
Árbol TF ROS:  map → caddy_ai2/odom → caddy_ai2/base_footprint
TF publicada:  world → map
```

```yaml
# config/caddy_ai2_single_map.yaml
ground_truth_tf_publisher:
  ros__parameters:
    use_sim_time: true
    multi_robot: false
    gz_model_name: caddy_ai2
    tf_prefix: caddy_ai2
    ground_truth_frame: world
    base_suffix: base_footprint
    localization_frame: map
    publish_rate: 50.0
```

```bash
ros2 launch gz_ground_truth single_map.launch.py
# o con mapa por robot:
ros2 launch gz_ground_truth single_map.launch.py \
    localization_frame:=caddy_ai2/map
```

---

### C — Multi-robot, sin localizador (odometría independiente)

```
Árbol TF ROS:  robot1/odom → robot1/base_footprint
               robot2/odom → robot2/base_footprint
TFs publicadas: world → robot1/odom
                world → robot2/odom
```

```yaml
# config/caddy_ai2_multi_odom.yaml
ground_truth_tf_publisher:
  ros__parameters:
    use_sim_time: true
    multi_robot: true
    robots:
      - robot1:robot1:robot1/odom   # gz_model_name:tf_prefix:localization_frame
      - robot2:robot2:robot2/odom
    ground_truth_frame: world
    base_suffix: base_footprint
    publish_rate: 50.0
```

```bash
ros2 launch gz_ground_truth multi_odom.launch.py
# o con yaml propio:
ros2 launch gz_ground_truth multi_odom.launch.py \
    params_file:=/ruta/a/mi_config.yaml
```

---

### D — Multi-robot, mapa independiente por robot

```
Árbol TF ROS:  robot1/map → robot1/odom → robot1/base_footprint
               robot2/map → robot2/odom → robot2/base_footprint
TFs publicadas: world → robot1/map
                world → robot2/map
```

```yaml
# config/caddy_ai2_multi_per_robot_map.yaml
ground_truth_tf_publisher:
  ros__parameters:
    use_sim_time: true
    multi_robot: true
    robots:
      - robot1:robot1:robot1/map
      - robot2:robot2:robot2/map
    ground_truth_frame: world
    base_suffix: base_footprint
    publish_rate: 50.0
```

```bash
ros2 launch gz_ground_truth multi_per_robot_map.launch.py
```

---

### E — Multi-robot, mapa global compartido

```
Árbol TF ROS:  map → robot1/odom → robot1/base_footprint
                     robot2/odom → robot2/base_footprint
TFs publicadas: world → map          (robot1 actúa de maestro)
                world → robot2/odom  (corrección residual de odometría)
```

> **Nota:** Solo un robot debe publicar la corrección del frame `map` compartido
> para evitar que dos fuentes compitan en el mismo topic `/tf`. En este escenario
> `robot1` corrige `map` y `robot2` corrige su propio `odom`.

```yaml
# config/caddy_ai2_multi_shared_map.yaml
ground_truth_tf_publisher:
  ros__parameters:
    use_sim_time: true
    multi_robot: true
    robots:
      - robot1:robot1:map          # maestro del frame compartido 'map'
      - robot2:robot2:robot2/odom  # corrige su odom (map ya corregido)
    ground_truth_frame: world
    base_suffix: base_footprint
    publish_rate: 50.0
```

```bash
ros2 launch gz_ground_truth multi_shared_map.launch.py
```

---

## Prerequisito: plugin PosePublisher en el SDF

El nodo se suscribe al topic gz-transport `/model/<gz_model_name>/pose`.
Para que Gazebo lo publique, **cada modelo SDF debe incluir el plugin
`PosePublisher`**:

```xml
<plugin filename="libignition-gazebo-pose-publisher-system"
        name="ignition::gazebo::systems::PosePublisher">
  <publish_model_pose>true</publish_model_pose>
  <update_frequency>100</update_frequency>
</plugin>
```

> `update_frequency` debe ser igual o mayor que el `publish_rate` del nodo.
> Sin este plugin Gazebo no publica la pose y el nodo no recibirá datos.

---

## Instalación

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gz_ground_truth
source install/setup.bash
```

---

## Parámetros

| Parámetro            | Tipo       | Default              | Descripción |
|----------------------|------------|----------------------|-------------|
| `multi_robot`        | `bool`     | `false`              | Activa modo multi-robot |
| `gz_model_name`      | `string`   | `caddy_ai2`          | Nombre del modelo en Gazebo *(robot único)* |
| `tf_prefix`          | `string`   | `''`                 | Prefijo TF *(vacío → usa gz_model_name)* |
| `localization_frame` | `string`   | `caddy_ai2/odom`     | Frame raíz a corregir (`odom` o `map`) *(robot único)* |
| `robots`             | `string[]` | —                    | Lista `gz:prefix:loc_frame` *(multi-robot)* |
| `ground_truth_frame` | `string`   | `world`              | Frame de ground truth (origen del mundo en Gazebo) |
| `base_suffix`        | `string`   | `base_footprint`     | Sufijo del frame del robot: `<tf_prefix>/<base_suffix>` |
| `publish_rate`       | `double`   | `50.0`               | Frecuencia de publicación (Hz) |
| `use_sim_time`       | `bool`     | `true`               | Usar reloj del simulador |

---

## Verificación

```bash
# Ver árbol de TFs
ros2 run tf2_tools view_frames

# Inspeccionar la corrección publicada (escenario A)
ros2 run tf2_ros tf2_echo world caddy_ai2/odom

# Inspeccionar la corrección publicada (escenario B)
ros2 run tf2_ros tf2_echo world map

# Confirmar el nombre exacto del modelo en Gazebo
gz model -l

# Ver topics gz disponibles
gz topic -l | grep pose
```

---

## Notas

- La pose de Gazebo en `/model/<n>/pose` es la posición absoluta del modelo en el
  frame del mundo (`world`), que coincide con el ground truth perfecto.
- El nodo se suscribe directamente a gz-transport sin necesidad de `ros_gz_bridge`.
- Se asume que el nodo arranca **después** de que el árbol TF de ROS ya esté disponible.
  Si el localizador aún no ha publicado sus TFs al arrancar, el nodo emitirá warnings
  y reintentará el lookup en cada ciclo hasta que estén disponibles.
- Para confirmar el nombre exacto del modelo en Gazebo:
  ```bash
  gz model -l
  ```
