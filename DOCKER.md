# forklift – Docker ile Çalıştırma

Bu belge, projeyi **ROS 2 Jazzy** ve **Gazebo Sim** ortamında Docker içinde nasıl derleyip çalıştıracağınızı açıklar.

## Gereksinimler

- Docker (ve isteğe bağlı Docker Compose)
- Linux’ta GUI için: X11 (`xhost +local:docker` ile paylaşım, aşağıda)

## Image oluşturma

Proje kökünde:

```bash
docker build -t forklift:latest .
```

İlk build birkaç dakika sürebilir (ROS ve Gazebo paketleri indirilir, workspace derlenir).

## Çalıştırma

### 1) Etkileşimli kabuk (önerilen)

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  forklift:latest
```

Konteyner içinde ortam zaten source edilmiş olur. Örnek:

```bash
ros2 launch offset_tricycle_robot tricycle_robot_sim.launch.py
```

Joystick kullanacaksanız cihazı paylaşın:

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/input/js0 \
  --network host \
  forklift:latest
```

### 2) Docker Compose

```bash
# Önce X11 erişimi (bir kez)
xhost +local:docker

# Sabit konteyner adı istiyorsanız (örn. "forklift"); yoksa Compose "forklift-forklift-run-xxxx" üretir
docker compose run --name forklift --rm forklift bash
```

İçeride yine `ros2 launch offset_tricycle_robot tricycle_robot_sim.launch.py` çalıştırabilirsiniz.

### 3) Doğrudan launch komutu

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  forklift:latest \
  ros2 launch offset_tricycle_robot tricycle_robot_sim.launch.py
```
